"""Mode-based /cmd_vel multiplexer (pure logic, no ROS).

At any moment only the active mode's source reaches the motors:
    idle   -> none (zero)
    manual -> teleop      (/cmd_vel/teleop)
    line   -> line        (/cmd_vel/line)
    nav    -> nav         (/cmd_vel/nav)
The emergency stop (estop) overrides everything. With teleop_takeover on, a
non-zero teleop command arriving while in another mode switches the robot to
manual: the operator can always step in.
"""

MODES = ('idle', 'manual', 'line', 'nav')
SOURCE_OF_MODE = {'manual': 'teleop', 'line': 'line', 'nav': 'nav'}


def is_moving(linear: float, angular: float, eps: float = 1e-3) -> bool:
    return abs(linear) > eps or abs(angular) > eps


class CommandMux:

    def __init__(self, timeout: float = 0.5, teleop_takeover: bool = True):
        self.timeout = timeout
        self.teleop_takeover = teleop_takeover
        self.mode = 'idle'
        self.estop = False
        self._inputs = {}

    def set_mode(self, mode: str):
        """Returns: (success, message)."""
        if mode not in MODES:
            return False, f"unknown mode '{mode}', valid: {', '.join(MODES)}"
        if self.estop and mode != 'idle':
            return False, 'emergency stop active, release it first'
        previous, self.mode = self.mode, mode
        self._inputs.pop(SOURCE_OF_MODE.get(mode), None)   # don't let a stale command leak through
        return True, f'{previous} -> {mode}'

    def set_estop(self, active: bool):
        self.estop = active
        if active:
            self.mode = 'idle'
            self._inputs.clear()

    def feed(self, source: str, linear: float, angular: float, now: float):
        """Takes a command from a source. Returns 'manual' if teleop takes over."""
        self._inputs[source] = (linear, angular, now)
        if (source == 'teleop' and self.teleop_takeover and not self.estop
                and self.mode != 'manual' and is_moving(linear, angular)):
            self.mode = 'manual'
            return 'manual'
        return None

    def output(self, now: float):
        if self.estop:
            return 0.0, 0.0
        source = SOURCE_OF_MODE.get(self.mode)
        cmd = self._inputs.get(source)
        if cmd is None or now - cmd[2] > self.timeout:
            return 0.0, 0.0
        return cmd[0], cmd[1]
