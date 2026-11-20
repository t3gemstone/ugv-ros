"""mode_manager_node: the robot's single /cmd_vel source.

Inputs     : /cmd_vel/teleop, /cmd_vel/line, /cmd_vel/nav, /scan, /goal_pose
Outputs    : /cmd_vel, /ugv/mode (latched String), /ugv/estop_active (latched Bool),
             /ugv/obstacle_distance (Float32, distance from the body to the nearest
             obstacle in the corridor along the direction of travel)
Services   : /ugv/set_mode (ugv_interfaces/SetMode), /ugv/estop (std_srvs/SetBool)

When a goal arrives on /goal_pose (GCS or RViz) the node switches to 'nav' mode
automatically. Nav2 goals are cancelled when leaving nav mode or on emergency stop.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool

from ugv_control.mux import CommandMux
from ugv_control.safety import corridor_gap, Footprint, LidarPose, limit_linear
from ugv_interfaces.srv import SetMode

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ModeManagerNode(Node):

    def __init__(self):
        super().__init__('mode_manager_node')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('teleop_takeover', True)
        self.declare_parameter('safety.enabled', True)
        self.declare_parameter('safety.stop_distance', 0.10)
        self.declare_parameter('safety.slow_distance', 0.35)
        self.declare_parameter('safety.side_margin', 0.01)
        self.declare_parameter('safety.scan_timeout', 0.5)
        # Body limits; robot.launch.py fills them in from robot.yaml.
        self.declare_parameter('footprint.front', 0.17)
        self.declare_parameter('footprint.rear', 0.09)
        self.declare_parameter('footprint.half_width', 0.09)
        # Fixed lidar pose relative to base_link; robot.launch.py fills it in from robot.yaml.
        # (Listening to TF meant processing every /tf message in Python.)
        self.declare_parameter('lidar.x', 0.0)
        self.declare_parameter('lidar.y', 0.0)
        self.declare_parameter('lidar.yaw', 0.0)
        p = lambda name: self.get_parameter(name).value  # noqa: E731

        self.mux = CommandMux(p('cmd_timeout'), p('teleop_takeover'))
        self.safety = p('safety.enabled')
        self.stop_distance = p('safety.stop_distance')
        self.slow_distance = p('safety.slow_distance')
        self.side_margin = p('safety.side_margin')
        self.scan_timeout = p('safety.scan_timeout')
        self.body = Footprint(p('footprint.front'), p('footprint.rear'),
                              p('footprint.half_width'))
        self.scan = None
        self.scan_time = 0.0
        self.lidar = LidarPose(p('lidar.x'), p('lidar.y'), p('lidar.yaw'))

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, 'ugv/mode', LATCHED)
        self.estop_pub = self.create_publisher(Bool, 'ugv/estop_active', LATCHED)
        self.obstacle_pub = self.create_publisher(Float32, 'ugv/obstacle_distance', 10)

        for source in ('teleop', 'line', 'nav'):
            self.create_subscription(
                Twist, f'cmd_vel/{source}',
                lambda msg, s=source: self._on_cmd(s, msg), 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal, 10)

        self.create_service(SetMode, 'ugv/set_mode', self._srv_set_mode)
        self.create_service(SetBool, 'ugv/estop', self._srv_estop)
        self.cancel_client = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')

        self._publish_state()
        self.create_timer(1.0 / p('rate'), self._tick)
        self.get_logger().info('mode: idle (the host changes it via /ugv/set_mode)')

    # --- state ---
    def _change_mode(self, mode: str):
        previous = self.mux.mode
        ok, message = self.mux.set_mode(mode)
        if ok:
            self._after_mode_change(previous)
        return ok, message

    def _after_mode_change(self, previous: str):
        if previous == 'nav' and self.mux.mode != 'nav':
            self._cancel_nav()
        self._publish_state()
        self.get_logger().info(f'mode: {previous} -> {self.mux.mode}')

    def _publish_state(self):
        self.mode_pub.publish(String(data=self.mux.mode))
        self.estop_pub.publish(Bool(data=self.mux.estop))

    def _cancel_nav(self):
        if self.cancel_client.service_is_ready():
            # Empty goal_info = cancel all goals.
            self.cancel_client.call_async(CancelGoal.Request())

    # --- inputs ---
    def _on_cmd(self, source: str, msg: Twist):
        previous = self.mux.mode
        if self.mux.feed(source, msg.linear.x, msg.angular.z, time.monotonic()):
            self.get_logger().warn(f'teleop took over (from {previous} mode)')
            self._after_mode_change(previous)

    def _on_scan(self, msg: LaserScan):
        self.scan = msg
        self.scan_time = time.monotonic()

    def _on_goal(self, msg: PoseStamped):
        if self.mux.estop:
            self.get_logger().warn('emergency stop active: nav goal cancelled')
            self._cancel_nav()
            return
        if self.mux.mode != 'nav':
            self._change_mode('nav')

    # --- services ---
    def _srv_set_mode(self, request, response):
        response.success, response.message = self._change_mode(request.mode.strip().lower())
        return response

    def _srv_estop(self, request, response):
        previous = self.mux.mode
        self.mux.set_estop(request.data)
        if request.data:
            self.cmd_pub.publish(Twist())
            self.get_logger().error('EMERGENCY STOP')
        else:
            self.get_logger().info('emergency stop released (mode: idle)')
        self._after_mode_change(previous)
        response.success = True
        response.message = 'active' if request.data else 'released'
        return response

    # --- loop ---
    def _tick(self):
        linear, angular = self.mux.output(time.monotonic())
        if self.safety and linear != 0.0:
            distance = self._obstacle_distance(forward=linear > 0.0)
            if distance is not None:
                linear = math.copysign(
                    limit_linear(abs(linear), distance, self.stop_distance, self.slow_distance),
                    linear)
                self.obstacle_pub.publish(Float32(data=float(min(distance, 99.0))))
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def _obstacle_distance(self, forward: bool):
        if self.scan is None or time.monotonic() - self.scan_time > self.scan_timeout:
            return None   # no lidar -> brake disabled (e.g. mock)
        s = self.scan
        return corridor_gap(s.ranges, s.angle_min, s.angle_increment,
                            self.lidar, self.body, forward,
                            self.side_margin, s.range_min, s.range_max)


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
