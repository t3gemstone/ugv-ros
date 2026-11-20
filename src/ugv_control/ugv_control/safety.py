"""Lidar-based collision brake: scales down the linear speed according to the
nearest obstacle in the corridor the robot is heading into (pure logic, no ROS)."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Footprint:
    """Body limits relative to base_link [m]."""

    front: float        # base_link -> front edge (+x)
    rear: float         # base_link -> rear edge (along -x, positive)
    half_width: float   # base_link -> side edge


@dataclass(frozen=True)
class LidarPose:
    """Lidar pose relative to base_link."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def corridor_gap(ranges, angle_min, angle_increment, lidar: LidarPose, body: Footprint,
                 forward: bool, side_margin: float = 0.0,
                 range_min=0.0, range_max=math.inf) -> float:
    """Distance from the nearest point in the corridor along the direction of travel
    (|y| <= half width + side_margin) to the front edge of the body (rear edge when
    reversing).

    Points beside the robot are not in the corridor; points that fall inside the
    body footprint are the robot's own parts and are ignored.
    """
    lane = body.half_width + side_margin
    best = math.inf
    for i, r in enumerate(ranges):
        if math.isnan(r) or not (range_min < r < range_max):
            continue
        angle = lidar.yaw + angle_min + i * angle_increment
        x = lidar.x + r * math.cos(angle)
        y = lidar.y + r * math.sin(angle)
        if abs(y) > lane:
            continue
        gap = x - body.front if forward else -x - body.rear
        if 0.0 <= gap < best:
            best = gap
    return best


def limit_linear(linear: float, distance: float, stop_distance: float,
                 slow_distance: float) -> float:
    """distance <= stop -> 0; linear scaling between stop..slow; unrestricted beyond."""
    if distance <= stop_distance:
        return 0.0
    if distance >= slow_distance or slow_distance <= stop_distance:
        return linear
    return linear * (distance - stop_distance) / (slow_distance - stop_distance)
