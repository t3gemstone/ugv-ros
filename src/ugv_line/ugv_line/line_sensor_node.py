"""line_sensor_node: reads the line sensor array from GPIO or from a Deneyap Kart.

type: serial   Analog sensor wired to the Deneyap Kart's ADC; the board sends
               millivolts over UART (firmware/deneyap_line). The per-channel threshold
               comes from calibration; without calibration no channel sees the line.
type: qtr_rc   Pololu QTR-8RC / QTR-xRC (RC type, 1 GPIO per channel).
               Discharge time above `qtr.threshold_us` = dark surface.
type: digital  Digital (DO) output of TCRT5000 and similar modules;
               `line_level` = level read when the sensor sees the line.

`pins` are ordered left to right (from behind the robot, facing forward). 3.3 V logic!
Publishes: /line/state (ugv_interfaces/LineState; for serial, raw = mV)
Services (serial only; called while the sensor rests on the surface for ~1 s):
  /line/calibrate_white, /line/calibrate_black  (std_srvs/Trigger)
  The black reading is taken after white; if the two separate, the calibration is saved.
"""

import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ugv_hal.gpio import open_chip, resolve_pin
from ugv_interfaces.msg import LineState
from ugv_line import adc_serial
from ugv_line.calibration import CalibrationSession
from ugv_line.line_math import read_line, sensor_positions
from ugv_line.qtr import dark_channels, read_decay_us

SERIAL_TIMEOUT = 0.5      # no frame for this long stops publishing (and the follow command)


class PinGroup:
    """Uses pins spread over different GPIO chips as a single group, keeping
    their order (one request per chip)."""

    def __init__(self, pins, backend, mode, bias='as-is'):
        self._requests = []
        self._index = []          # per pin: (request index, index within the request)
        by_chip = {}
        for chip_name, offset in pins:
            by_chip.setdefault(chip_name, []).append(offset)
        positions = {}
        for chip_name, offsets in by_chip.items():
            chip = open_chip(chip_name, backend)
            if mode == 'flex':
                req = chip.request_flex(offsets, consumer='ugv_line')
            else:
                req = chip.request_inputs(offsets, consumer='ugv_line', bias=bias)
            for j, offset in enumerate(offsets):
                positions[(chip_name, offset)] = (len(self._requests), j)
            self._requests.append(req)
        self._index = [positions[pin] for pin in pins]

    def drive_high(self):
        for req in self._requests:
            req.drive_high()

    def to_input(self):
        for req in self._requests:
            req.to_input()

    def get_all(self):
        values = [req.get_all() for req in self._requests]
        return [values[r][j] for r, j in self._index]

    def release(self):
        for req in self._requests:
            req.release()


class LineSensorNode(Node):

    def __init__(self):
        super().__init__('line_sensor_node')
        self.declare_parameter('backend', 'gpiod')
        self.declare_parameter('type', 'qtr_rc')
        self.declare_parameter('pins', [''])
        self.declare_parameter('emitter_pin', '')
        self.declare_parameter('spacing', 0.009525)
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('frame_id', 'line_sensor_link')
        self.declare_parameter('line_is_dark', True)
        self.declare_parameter('qtr.threshold_us', 1000)
        self.declare_parameter('qtr.timeout_us', 2500)
        self.declare_parameter('qtr.charge_us', 10)
        self.declare_parameter('digital.line_level', 1)
        self.declare_parameter('digital.bias', 'as-is')
        self.declare_parameter('serial.port', '/dev/ttyS3')
        self.declare_parameter('serial.baudrate', 115200)
        self.declare_parameter('serial.calibration', '~/.ugv/line_calibration.json')

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        backend = p('backend')
        self.type = p('type')
        if self.type not in ('serial', 'qtr_rc', 'digital'):
            raise ValueError(f"type must be serial, qtr_rc or digital, got: {self.type!r}")
        self.frame_id = p('frame_id')
        self.pub = self.create_publisher(LineState, 'line/state', 10)
        if self.type == 'serial':
            self._init_serial(p)
            return

        pins = [resolve_pin(s, backend) for s in p('pins') if s.strip()]
        if not pins and backend == 'mock':
            pins = [resolve_pin(f'GPIO{100 + i}', 'mock') for i in range(8)]
        if not pins:
            raise ValueError("line_sensor_node: 'pins' is empty (e.g. ['GPIO6', 'GPIO19', ...])")

        self.count = len(pins)
        self.positions = sensor_positions(self.count, p('spacing'))
        self.line_is_dark = p('line_is_dark')
        self.threshold = p('qtr.threshold_us')
        self.timeout = p('qtr.timeout_us')
        self.charge = p('qtr.charge_us')
        self.line_level = p('digital.line_level')
        self.group = PinGroup(pins, backend, 'flex' if self.type == 'qtr_rc' else 'input',
                              p('digital.bias'))

        self.emitter = None
        emitter = resolve_pin(p('emitter_pin'), backend)
        if emitter:
            self.emitter = open_chip(emitter[0], backend).request_outputs(
                [emitter[1]], consumer='ugv_line')
            self.emitter.set(emitter[1], 1)

        self.create_timer(1.0 / p('rate'), self._tick)
        self.get_logger().info(
            f'{self.type}: {self.count} channels, spacing {p("spacing") * 1000:.1f} mm'
            + (f', threshold {self.threshold} µs' if self.type == 'qtr_rc' else ''))

    # --- type: serial (Deneyap Kart) ---
    def _init_serial(self, p):
        self.group = None
        self.emitter = None
        self.count = adc_serial.CHANNELS
        self.positions = sensor_positions(self.count, p('spacing'))
        self.port_name = p('serial.port')
        self.baudrate = p('serial.baudrate')
        self.calibration_path = os.path.expanduser(p('serial.calibration'))
        self.calibration_stamp = None
        self.calibration = None
        self.port = None
        self.reader = adc_serial.FrameReader()
        self.last_frame = time.monotonic()
        self.session = CalibrationSession(self.calibration_path)
        self.warned = set()
        self._load_calibration()
        self.create_service(Trigger, 'line/calibrate_white', self._srv_white)
        self.create_service(Trigger, 'line/calibrate_black', self._srv_black)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'ugv/mode',
                                 lambda msg: self.session.set_mode(msg.data), latched)
        self.create_subscription(Twist, 'cmd_vel', self._calibration_command, 10)
        self.calibration_pub = self.create_publisher(String, 'line/calibration_status', 1)
        self.create_timer(0.5, self._publish_calibration_status)
        self.create_timer(1.0 / p('rate'), self._tick_serial)
        self.get_logger().info(
            f'serial: Deneyap {self.port_name} @ {self.baudrate}, calibration '
            + ('loaded' if self.calibration else f'MISSING ({self.calibration_path})'))

    def _warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warn(text)

    def _load_calibration(self):
        """Re-reads the file if it changed (a manually copied calibration also takes effect)."""
        try:
            stamp = os.stat(self.calibration_path).st_mtime_ns
        except OSError:
            stamp = None
        if stamp == self.calibration_stamp:
            return
        self.calibration_stamp = stamp
        try:
            self.calibration = adc_serial.load_calibration(self.calibration_path)
        except (OSError, ValueError) as exc:
            self.calibration = None
            self.get_logger().error(f'invalid calibration file: {exc}')

    def _open_port(self):
        import serial   # only needed for the serial type (python3-serial)
        try:
            self.port = serial.Serial(self.port_name, self.baudrate, timeout=0)
            self.port.reset_input_buffer()
            self.reader = adc_serial.FrameReader()
            self.last_frame = time.monotonic()
            self.warned.discard('port')
            self.get_logger().info(f'{self.port_name} opened')
        except (serial.SerialException, OSError) as exc:
            self.port = None
            self._warn_once('port', f'could not open {self.port_name}: {exc}')

    def _tick_serial(self):
        if self.port is None:
            self._open_port()
            if self.port is None:
                return
        try:
            waiting = self.port.in_waiting
            values = self.reader.feed(self.port.read(waiting)) if waiting else None
        except OSError as exc:
            self.get_logger().error(f'could not read {self.port_name}: {exc}')
            self.port.close()
            self.port = None
            self.session.clear()
            return
        now = time.monotonic()
        if values is None:
            if now - self.last_frame > SERIAL_TIMEOUT:
                self.session.clear()
                self._warn_once('timeout', "no data from the Deneyap (cable, baud, firmware)")
            return
        self.last_frame = now
        self.warned.discard('timeout')
        self.session.feed(values, now)
        self._load_calibration()
        if self.calibration is None:
            self._warn_once('calibration', 'line sensor is not calibrated: '
                            'call /line/calibrate_white and /line/calibrate_black')
        self._publish(values, adc_serial.dark_channels(values, self.calibration))

    def _calibration_command(self, msg):
        self.session.set_command([msg.linear.x, msg.linear.y, msg.linear.z,
                                  msg.angular.x, msg.angular.y, msg.angular.z], time.monotonic())

    def _publish_calibration_status(self):
        now = time.monotonic()
        if not self.session.stationary(now):
            self.session.clear()
        data = self.session.status(self.calibration, now)
        self.calibration_pub.publish(String(data=json.dumps(data)))

    def _srv_white(self, request, response):
        return self._capture_surface('white', response)

    def _srv_black(self, request, response):
        return self._capture_surface('black', response)

    def _capture_surface(self, surface, response):
        response.success = False
        try:
            calibration = self.session.capture(surface, time.monotonic())
        except (ValueError, OSError) as exc:
            response.message = f'calibration not saved: {exc}'
            self.get_logger().error(response.message)
            return response
        if calibration is not None:
            self.calibration = calibration
            self.calibration_stamp = None
            self.warned.discard('calibration')
        response.success = True
        response.message = self.session.message
        self.get_logger().info(response.message)
        self._publish_calibration_status()
        return response

    # --- GPIO types ---
    def _read(self):
        """Returns: (raw values, channels that see the line)."""
        if self.type == 'qtr_rc':
            raw = read_decay_us(self.group, self.count, self.timeout, self.charge)
            dark = dark_channels(raw, self.threshold)
            return raw, [d == self.line_is_dark for d in dark]
        raw = self.group.get_all()
        return raw, [v == self.line_level for v in raw]

    def _tick(self):
        self._publish(*self._read())

    def _publish(self, raw, active):
        reading = read_line(active, self.positions)
        msg = LineState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.sensors = reading.sensors
        msg.raw = [min(int(v), 65535) for v in raw]
        msg.detected = reading.detected
        msg.position = float(reading.position)
        msg.intersection = reading.intersection
        self.pub.publish(msg)

    def destroy_node(self):
        if self.group is not None:
            self.group.release()
        if getattr(self, 'port', None) is not None:
            self.port.close()
        if self.emitter is not None:
            self.emitter.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
