"""Wheel speed control: acceleration limiter + PI with feedforward."""

import math


class SlewLimiter:
    """Limits the rate of change of the target speed to max_accel [m/s^2]."""

    def __init__(self, max_accel: float):
        self.max_accel = max_accel
        self.value = 0.0

    def step(self, target: float, dt: float) -> float:
        if self.max_accel <= 0.0:
            self.value = target
            return target
        max_step = self.max_accel * dt
        self.value += max(-max_step, min(max_step, target - self.value))
        return self.value

    def reset(self):
        self.value = 0.0


class WheelSpeedController:
    """Converts one wheel's speed [m/s] into a PWM duty percentage [-100, 100].

    duty = kff * target / max_speed * 100 + kp * error + ki * ∫error

    - kff: feedforward; with 1.0 and a correct max_speed the PI only makes
      small corrections and the gains can be kept low.
    - min_duty: the lowest duty that gets the motor moving (static
      friction); measured with hw_check.
    - The integral is not grown while saturated (anti-windup).
    - i_limit: the integral contributes at most this much to the duty [%]. When
      the speed measurement stayed low for a while (battery voltage sags, encoder
      goes silent) the integral used to lock the duty at 100% and wipe out the
      difference between the two wheels (the turn); with the limit it stays at
      feedforward + a small correction.
    """

    def __init__(self, kp: float, ki: float, kff: float, max_speed: float,
                 min_duty: float = 0.0, max_duty: float = 100.0, i_limit: float = 30.0):
        if max_speed <= 0.0:
            raise ValueError('max_wheel_speed must be positive')
        self.kp = kp
        self.ki = ki
        self.kff = kff
        self.max_speed = max_speed
        self.min_duty = min_duty
        self.max_duty = max_duty
        self.i_limit = i_limit
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, target: float, measured: float, dt: float) -> float:
        if abs(target) < 1e-4:
            self.reset()
            return 0.0
        if self.integral != 0.0 and math.copysign(1.0, self.integral) != math.copysign(1.0, target):
            # Direction changed: don't let the integral built up in the old
            # direction brake the new one.
            self.reset()

        error = target - measured
        feedforward = self.kff * target / self.max_speed * 100.0
        unclamped = feedforward + self.kp * error + self.ki * (self.integral + error * dt)
        saturated = abs(unclamped) >= self.max_duty
        if not saturated or math.copysign(1.0, error) != math.copysign(1.0, unclamped):
            self.integral += error * dt
        if self.ki > 0.0:
            bound = self.i_limit / self.ki
            self.integral = max(-bound, min(bound, self.integral))

        duty = feedforward + self.kp * error + self.ki * self.integral
        duty = max(-self.max_duty, min(self.max_duty, duty))
        if duty * target > 0.0 and abs(duty) < self.min_duty:
            duty = math.copysign(self.min_duty, duty)
        elif duty * target <= 0.0:
            # If the PI wants to brake, let the motor coast; driving the
            # H-bridge in reverse makes the wheel judder.
            duty = 0.0
        return duty


class LowPassFilter:

    def __init__(self, alpha: float):
        self.alpha = max(0.0, min(1.0, alpha))
        self.value = 0.0

    def step(self, sample: float) -> float:
        self.value += self.alpha * (sample - self.value)
        return self.value
