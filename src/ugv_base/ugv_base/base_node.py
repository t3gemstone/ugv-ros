"""ugv_base_node: /cmd_vel -> closed-loop speed control per wheel -> motors.

Loop (control_rate Hz):
  1. encoder counts -> wheel travel and speed (low-pass filtered)
  2. odometry -> /wheel/odom (+ optional odom->base_link TF), /joint_states
  3. /cmd_vel -> wheel targets -> acceleration limit -> PI + feedforward -> PWM

If no /cmd_vel arrives within cmd_timeout the target is zeroed (watchdog).
By default the TF is published by robot_localization (EKF); when fusion is off
the launch file sets publish_tf:=true.
With encoder.mode: none the motors are driven open loop and odometry is
estimated from the commanded speed (with high covariance) — only as an
emergency fallback without encoders.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

from ugv_base.config import DEFAULTS
from ugv_base.control import LowPassFilter, SlewLimiter, WheelSpeedController
from ugv_base.drivetrain import build_drivetrain
from ugv_base.encoder import multiplier
from ugv_base.kinematics import (
    WheelOdometry, counts_to_distance, limit_wheels, twist_to_wheels, wheels_to_twist)


class BaseNode(Node):

    def __init__(self):
        super().__init__('ugv_base_node')
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        cfg = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.cfg = cfg

        self.drivetrain = build_drivetrain(cfg)
        self.closed_loop = cfg['encoder.mode'] != 'none' or cfg['backend'] == 'mock'
        self.cpr = getattr(self.drivetrain, 'counts_per_rev',
                           cfg['encoder.ppr'] * multiplier(cfg['encoder.mode']))
        if self.closed_loop and self.cpr <= 0:
            raise ValueError('encoder.ppr must be set (measure it with hw_check encoders)')

        self.radius = cfg['wheel_radius']
        self.separation = cfg['wheel_separation']
        self.max_speed = cfg['max_wheel_speed']
        self.odometry = WheelOdometry(self.separation)
        self.ramps = [SlewLimiter(cfg['max_accel']) for _ in range(2)]
        self.filters = [LowPassFilter(cfg['speed_filter_alpha']) for _ in range(2)]
        self.pids = [WheelSpeedController(
            cfg['pid.kp'], cfg['pid.ki'], cfg['pid.kff'], self.max_speed,
            cfg['pid.min_duty'], cfg['pid.max_duty'], cfg['pid.i_limit']) for _ in range(2)]

        self.cmd = Twist()
        self.cmd_time = None
        self.last_counts = self.drivetrain.counts()
        self.wheel_angle = [0.0, 0.0]
        self.last_time = self.get_clock().now()

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, '~/wheels', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if cfg['publish_tf'] else None
        self.create_timer(1.0 / cfg['control_rate'], self._loop)
        # Every /joint_states becomes a TF; publishing at the control rate floods /tf for nothing.
        self.state_every = max(1, round(cfg['control_rate'] / cfg['state_rate']))
        self.loop_count = 0

        self.get_logger().info(
            f"started: backend={cfg['backend']} encoder={cfg['encoder.mode']} "
            f"cpr={self.cpr:.0f} r={self.radius} L={self.separation} "
            f"vmax={self.max_speed} m/s closed_loop={self.closed_loop}")
        if not self.closed_loop:
            self.get_logger().warn('no encoders: open-loop driving, estimated odometry')
        for note in getattr(self.drivetrain, 'notes', []):
            self.get_logger().warn(note)

    def _on_cmd(self, msg: Twist):
        self.cmd = msg
        self.cmd_time = self.get_clock().now()

    def _targets(self, now):
        fresh = (self.cmd_time is not None and
                 (now - self.cmd_time).nanoseconds * 1e-9 <= self.cfg['cmd_timeout'])
        if not fresh:
            return 0.0, 0.0
        left, right = twist_to_wheels(self.cmd.linear.x, self.cmd.angular.z, self.separation)
        return limit_wheels(left, right, self.max_speed)

    def _loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return
        self.drivetrain.step(dt)

        targets = self._targets(now)
        ramped = [self.ramps[i].step(targets[i], dt) for i in range(2)]

        counts = self.drivetrain.counts()
        if self.closed_loop and counts is not None:
            deltas = [counts[i] - self.last_counts[i] for i in range(2)]
            self.last_counts = counts
            dist = [counts_to_distance(d, self.cpr, self.radius) for d in deltas]
            speeds = [self.filters[i].step(dist[i] / dt) for i in range(2)]
            duties = [self.pids[i].update(ramped[i], speeds[i], dt) for i in range(2)]
        else:
            dist = [ramped[i] * dt for i in range(2)]
            speeds = list(ramped)
            duties = [max(-100.0, min(100.0, self.cfg['pid.kff'] * v / self.max_speed * 100.0))
                      for v in ramped]

        self.drivetrain.set_duty(duties[0], duties[1])
        self.loop_count += 1
        state = self.loop_count % self.state_every == 0
        self._publish(now, dist, speeds, state)
        if state:
            self.debug_pub.publish(Float32MultiArray(data=[
                ramped[0], speeds[0], duties[0], ramped[1], speeds[1], duties[1]]))

    def _publish(self, now, dist, speeds, state):
        pose = self.odometry.update(dist[0], dist[1])
        v, w = wheels_to_twist(speeds[0], speeds[1], self.separation)
        stamp = now.to_msg()
        qz, qw = math.sin(pose.theta / 2.0), math.cos(pose.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.cfg['odom_frame']
        odom.child_frame_id = self.cfg['base_frame']
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        scale = 1.0 if self.closed_loop else 25.0
        odom.pose.covariance = _diag([0.01, 0.01, 1e6, 1e6, 1e6, 0.05], scale)
        odom.twist.covariance = _diag([0.002, 0.002, 1e6, 1e6, 1e6, 0.02], scale)
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = odom.child_frame_id
            tf.transform.translation.x = pose.x
            tf.transform.translation.y = pose.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

        for i in range(2):
            self.wheel_angle[i] += dist[i] / self.radius
        if not state:
            return
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['left_wheel_joint', 'right_wheel_joint']
        joints.position = list(self.wheel_angle)
        joints.velocity = [s / self.radius for s in speeds]
        self.joint_pub.publish(joints)

    def destroy_node(self):
        self.drivetrain.set_duty(0.0, 0.0)
        self.drivetrain.close()
        super().destroy_node()


def _diag(values, scale):
    cov = [0.0] * 36
    for i, value in enumerate(values):
        cov[i * 7] = value * scale if value < 1e5 else value
    return cov


def main(args=None):
    rclpy.init(args=args)
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
