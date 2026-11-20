"""Runs the RPLIDAR driver (sllidar_node) over the selected connection.

  usb   the lidar's USB adapter; the port is found automatically under /dev/serial/by-id
  gpio  no adapter: the T3 Gemstone O1's second UART (/dev/ttyS0, pin 18/26)
        + PWM for the lidar motor (ecap2)

The connection is changed with the /ugv/set_lidar service (the Lidar panel in the UI)
and saved to `state_file`; the same connection is used when the robot restarts.
`busy_port` is a UART used by another device (e.g. the Deneyap line sensor on
/dev/ttyS3); a GPIO connection that lands on that port is not started.
If the driver exits (e.g. the USB cable came out) it is restarted after `restart_delay`.
Status is published on /ugv/lidar.

On the old robot, 100% duty on the GPIO connection corrupted the UART with motor
noise and gave an "operation timeout" error; 85% is the value verified in the field.
"""

import ctypes
import os
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, PackageNotFoundError
from ugv_interfaces.msg import LidarStatus
from ugv_interfaces.srv import SetLidar
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from ugv_hal.lidar import (choose_source, find_usb_port, load_choice, RateMeter, save_choice,
                           sllidar_arguments, SOURCES)
from ugv_hal.pwm import open_pwm

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
PR_SET_PDEATHSIG = 1


def _die_with_parent():
    # If the manager dies unexpectedly, don't leave the driver orphaned holding the port.
    try:
        ctypes.CDLL('libc.so.6', use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGTERM)
    except (OSError, AttributeError):
        pass


class LidarManagerNode(Node):

    def __init__(self):
        super().__init__('lidar_manager_node')
        for name, default in {
            'backend': 'gpiod',
            'source': 'usb',
            'source_override': '',
            'port_override': '',
            'busy_port': '',
            'state_file': '~/.ugv/lidar_source',
            'baudrate': 115200,
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
            'restart_delay': 3.0,
            'usb.port': '',
            'gpio.port': '/dev/ttyS0',
            'gpio.pwm': 'ecap2',
            'gpio.period_ns': 1_000_000,
            'gpio.duty_percent': 85.0,
        }.items():
            self.declare_parameter(name, default)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._p = p
        self._state_file = os.path.expanduser(p('state_file'))

        try:
            prefix = get_package_prefix('sllidar_ros2')
            self._driver = os.path.join(prefix, 'lib', 'sllidar_ros2', 'sllidar_node')
        except PackageNotFoundError:
            self._driver = ''

        self._proc = None
        self._pwm = None
        self._port = ''
        self._message = ''
        self._retry_at = 0.0
        self._rate = RateMeter()

        self._status_pub = self.create_publisher(LidarStatus, 'ugv/lidar', LATCHED)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_service(SetLidar, 'ugv/set_lidar', self._on_set)
        self.create_timer(1.0, self._watch)

        self._source = choose_source(p('source_override'), load_choice(self._state_file),
                                     p('source'))
        self.get_logger().info(f'lidar connection: {self._source}')
        self._start()

    # --- driver ----------------------------------------------------------------
    def _resolve_port(self) -> str:
        if self._p('port_override'):
            return self._p('port_override')
        if self._source == 'usb':
            return self._p('usb.port') or find_usb_port()
        return self._p('gpio.port')

    def _port_busy(self, port: str) -> bool:
        busy = self._p('busy_port')
        return bool(busy) and os.path.realpath(port) == os.path.realpath(busy)

    def _set_motor(self, on: bool):
        if on and self._pwm is None:
            self._pwm = open_pwm(self._p('gpio.pwm'), self._p('gpio.period_ns'),
                                 self._p('backend'))
            if self._pwm is not None:
                self._pwm.set_duty_percent(self._p('gpio.duty_percent'))
        elif not on and self._pwm is not None:
            self._pwm.release()
            self._pwm = None

    def _start(self):
        self._retry_at = 0.0
        try:
            self._set_motor(self._source == 'gpio' and not self._port_busy(self._resolve_port()))
        except (OSError, ValueError) as e:
            return self._fail(f"could not open lidar motor PWM ({self._p('gpio.pwm')}): {e}")

        self._port = self._resolve_port()
        if self._port_busy(self._port):
            return self._fail(f'{self._port} is reserved for the line sensor (Deneyap); '
                              'select the USB connection for the lidar')
        if not self._driver:
            return self._fail('sllidar_ros2 package is not installed')
        if not self._port or not os.path.exists(self._port):
            hint = ('is the USB adapter plugged in?' if self._source == 'usb'
                    else 'UART overlay and pin 18/26 wiring?')
            return self._fail(f"no lidar port: {self._port or '(not found)'}; {hint}")

        args = sllidar_arguments(self._port, self._p('baudrate'), self._p('frame_id'),
                                 self._p('inverted'), self._p('angle_compensate'))
        try:
            self._proc = subprocess.Popen([self._driver, *args], preexec_fn=_die_with_parent)
        except OSError as e:
            return self._fail(f'could not start sllidar_node: {e}')
        self._message = f'{self._source}: {self._port}'
        self.get_logger().info(f'sllidar_node started ({self._message})')
        self._publish()

    def _fail(self, message: str):
        if message != self._message:
            self.get_logger().warn(message)
        self._message = message
        self._retry_at = time.monotonic() + self._p('restart_delay')
        self._publish()

    def _stop(self):
        proc, self._proc = self._proc, None
        if proc is None or proc.poll() is not None:
            return
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

    def _watch(self):
        if self._proc is not None and self._proc.poll() is not None:
            code = self._proc.returncode
            self._proc = None
            self._fail(f'sllidar_node exited (code {code}); will restart')
        elif self._proc is None and self._retry_at and time.monotonic() >= self._retry_at:
            self._start()
        self._publish()

    # --- ROS interface -------------------------------------------------------
    def _on_scan(self, _msg):
        self._rate.tick(time.monotonic())

    def _on_set(self, request, response):
        source = request.source.strip().lower()
        if source not in SOURCES:
            response.success = False
            response.message = f"unknown connection '{request.source}' (usb | gpio)"
            return response
        try:
            save_choice(self._state_file, source)
            saved = ''
        except OSError as e:
            saved = f' (could not save: {e})'
        if source == self._source and self._proc is not None:
            response.success = True
            response.message = f'lidar already {source}{saved}'
            return response

        self.get_logger().info(f'lidar connection changing: {self._source} -> {source}')
        self._stop()
        self._source = source
        self._rate = RateMeter()
        self._start()
        response.success = self._proc is not None
        response.message = (
            f'lidar: {self._message}' if response.success
            else f'lidar {source} selected but did not start: {self._message}') + saved
        return response

    def _publish(self):
        self._status_pub.publish(LidarStatus(
            source=self._source, port=self._port, running=self._proc is not None,
            rate_hz=float(self._rate.rate(time.monotonic())), message=self._message))

    def destroy_node(self):
        self._stop()
        self._set_motor(False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
