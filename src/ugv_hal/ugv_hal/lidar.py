"""ROS-free helpers for choosing the lidar connection (used by lidar_manager_node).

Two connections:
  usb   the lidar's own USB adapter (CP210x); the adapter drives the motor
  gpio  no adapter, T3 Gemstone O1 UART directly + PWM for the lidar motor
"""

from collections import deque
import glob
import os

SOURCES = ('usb', 'gpio')

# The persistent by-id path doesn't depend on which USB port is used; otherwise ttyUSB*.
USB_PATTERNS = (
    '/dev/serial/by-id/*CP210*',
    '/dev/serial/by-id/*',
    '/dev/ttyUSB*',
)


def find_usb_port(patterns=USB_PATTERNS) -> str:
    for pattern in patterns:
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    return ''


def load_choice(path: str):
    """Saved connection; None if the file is missing or invalid."""
    try:
        with open(path) as f:
            value = f.read().strip()
    except OSError:
        return None
    return value if value in SOURCES else None


def save_choice(path: str, source: str):
    if source not in SOURCES:
        raise ValueError(f'lidar connection must be one of {SOURCES}: {source!r}')
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    tmp = f'{path}.tmp'
    with open(tmp, 'w') as f:
        f.write(source + '\n')
    os.replace(tmp, path)


def choose_source(override: str, saved, default: str) -> str:
    """Precedence: launch argument > saved choice > yaml default."""
    for value in (override, saved, default):
        if value in SOURCES:
            return value
    raise ValueError(f'no valid lidar connection (usb | gpio): {default!r}')


def sllidar_arguments(port: str, baudrate: int, frame_id: str,
                      inverted: bool, angle_compensate: bool) -> list:
    """sllidar_node command line (ROS arguments only)."""
    params = {
        'channel_type': 'serial',
        'serial_port': port,
        'serial_baudrate': int(baudrate),
        'frame_id': frame_id,
        'inverted': 'true' if inverted else 'false',
        'angle_compensate': 'true' if angle_compensate else 'false',
    }
    args = ['--ros-args', '-r', '__node:=sllidar_node']
    for name, value in params.items():
        args += ['-p', f'{name}:={value}']
    return args


class RateMeter:
    """Event rate over the last `window` seconds [Hz]."""

    def __init__(self, window: float = 3.0):
        self.window = window
        self._stamps = deque()

    def tick(self, now: float):
        self._stamps.append(now)
        self._trim(now)

    def rate(self, now: float) -> float:
        self._trim(now)
        if len(self._stamps) < 2:
            return 0.0
        return (len(self._stamps) - 1) / max(now - self._stamps[0], 1e-6)

    def _trim(self, now: float):
        while self._stamps and now - self._stamps[0] > self.window:
            self._stamps.popleft()
