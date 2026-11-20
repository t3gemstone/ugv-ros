"""Simulated line sensor array: a downward camera image -> channel brightness (no ROS).

The camera sits where the real sensor array is and looks straight down. Its image is
as wide as the array: the left edge of the image is the robot's left (+y), and the
footprint on the ground spans +-half_width [m] across the image width.
"""


def sensor_columns(width, positions, half_width):
    """Image column under each sensor (positions: sensor y [m], left to right)."""
    centre = (width - 1) / 2.0
    scale = width / (2.0 * half_width)
    return [min(width - 1, max(0, round(centre - y * scale))) for y in positions]


def channel_levels(data, width, height, step, columns, radius=1):
    """Mean brightness 0..1 around each column in the middle rows of a mono8 image."""
    rows = range(max(0, height // 2 - 1), min(height, height // 2 + 1))
    levels = []
    for col in columns:
        cols = range(max(0, col - radius), min(width, col + radius + 1))
        values = [data[r * step + c] for r in rows for c in cols]
        levels.append(sum(values) / (255.0 * len(values)))
    return levels


def to_millivolts(levels, full_scale=3300):
    """Raw values like the Deneyap ADC: a dark surface reads high."""
    return [int(round(full_scale * (1.0 - level))) for level in levels]
