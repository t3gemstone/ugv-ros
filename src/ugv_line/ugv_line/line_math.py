"""Line sensor array and line following controller (pure logic, no ROS).

The position convention matches ROS: +y is the robot's LEFT. Sensors are ordered
left to right; N sensors are laid out symmetrically about the center at `spacing` intervals.
"""

from dataclasses import dataclass
import math


def sensor_positions(count: int, spacing: float):
    """Sensor y positions left to right [m]; e.g. 5 sensors, 1 cm -> 0.02 .. -0.02."""
    mid = (count - 1) / 2.0
    return [(mid - i) * spacing for i in range(count)]


@dataclass
class LineReading:
    sensors: list
    detected: bool
    position: float
    intersection: bool


def read_line(active, positions) -> LineReading:
    """The centroid of the active sensors is the line position."""
    hits = [p for p, on in zip(positions, active) if on]
    if not hits:
        return LineReading(list(active), False, 0.0, False)
    return LineReading(list(active), True, sum(hits) / len(hits),
                       len(hits) == len(positions) and len(positions) > 2)


class LineFollower:
    """Applies PD to the line position error and produces (v, w).

    - Forward speed drops as the error grows (so it doesn't skid in sharp turns).
    - If the line is lost, turns toward the side it was last seen with `search_angular`;
      after `lost_timeout` it stops and reports 'lost'.
    - At an intersection (all sensors active) `intersection_action`: straight | stop.
    """

    def __init__(self, speed=0.15, kp=12.0, kd=0.4, max_angular=2.5,
                 min_speed_ratio=0.3, half_width=0.03,
                 search_angular=1.5, lost_timeout=1.5, intersection_action='straight'):
        if intersection_action not in ('straight', 'stop'):
            raise ValueError("intersection_action must be straight or stop")
        self.speed = speed
        self.kp = kp
        self.kd = kd
        self.max_angular = max_angular
        self.min_speed_ratio = min_speed_ratio
        self.half_width = half_width
        self.search_angular = search_angular
        self.lost_timeout = lost_timeout
        self.intersection_action = intersection_action
        self.reset()

    def reset(self):
        self.last_error = None
        self.last_side = 0.0
        self.lost_for = 0.0
        self.state = 'idle'

    def update(self, reading: LineReading, dt: float):
        """Returns: (linear_x, angular_z)."""
        if reading.intersection:
            self.lost_for = 0.0
            self.last_error = 0.0
            if self.intersection_action == 'stop':
                self.state = 'intersection'
                return 0.0, 0.0
            self.state = 'following'
            return self.speed, 0.0

        if not reading.detected:
            self.lost_for += dt
            self.last_error = None
            if self.lost_for > self.lost_timeout or self.last_side == 0.0:
                self.state = 'lost'
                return 0.0, 0.0
            self.state = 'searching'
            return 0.0, math.copysign(self.search_angular, self.last_side)

        self.lost_for = 0.0
        error = reading.position            # line on the left (+) -> turn left (+w)
        if abs(error) > 1e-6:
            self.last_side = math.copysign(1.0, error)
        derivative = 0.0
        if self.last_error is not None and dt > 0.0:
            derivative = (error - self.last_error) / dt
        self.last_error = error

        angular = self.kp * error + self.kd * derivative
        angular = max(-self.max_angular, min(self.max_angular, angular))
        ratio = min(1.0, abs(error) / self.half_width) if self.half_width > 0 else 0.0
        linear = self.speed * (1.0 - (1.0 - self.min_speed_ratio) * ratio)
        self.state = 'following'
        return linear, angular
