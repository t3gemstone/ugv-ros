"""Differential-drive kinematics and wheel odometry (pure math, no ROS)."""

from dataclasses import dataclass
import math


def twist_to_wheels(linear: float, angular: float, separation: float):
    """(v, w) -> (left, right) wheel linear speed [m/s]."""
    half = separation / 2.0
    return linear - angular * half, linear + angular * half


def wheels_to_twist(left: float, right: float, separation: float):
    """(left, right) wheel speed [m/s] -> (v [m/s], w [rad/s])."""
    if separation <= 0.0:
        raise ValueError('wheel_separation must be positive')
    return (left + right) / 2.0, (right - left) / separation


def limit_wheels(left: float, right: float, max_speed: float):
    """If either wheel exceeds the limit, scales both down by the same ratio so
    the turning radius is preserved (clipping alone would force the robot straight)."""
    peak = max(abs(left), abs(right))
    if peak <= max_speed or peak == 0.0:
        return left, right
    scale = max_speed / peak
    return left * scale, right * scale


def counts_to_distance(counts: int, counts_per_rev: float, wheel_radius: float) -> float:
    if counts_per_rev <= 0.0:
        raise ValueError('counts_per_rev must be positive')
    return counts / counts_per_rev * 2.0 * math.pi * wheel_radius


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


class WheelOdometry:
    """Pose tracking from wheel travel deltas (dead reckoning).

    Works with distance travelled instead of speed, so timing jitter in the
    control loop does not turn into pose error.
    """

    def __init__(self, separation: float):
        if separation <= 0.0:
            raise ValueError('wheel_separation must be positive')
        self.separation = separation
        self.pose = Pose2D()

    def update(self, d_left: float, d_right: float) -> Pose2D:
        ds = (d_left + d_right) / 2.0
        dtheta = (d_right - d_left) / self.separation
        mid = self.pose.theta + dtheta / 2.0
        self.pose.x += ds * math.cos(mid)
        self.pose.y += ds * math.sin(mid)
        self.pose.theta = normalize_angle(self.pose.theta + dtheta)
        return self.pose
