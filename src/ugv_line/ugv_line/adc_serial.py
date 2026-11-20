"""Analog line sensor data from the Deneyap Kart (firmware/deneyap_line).

The board reads each channel with its ADC in millivolts and sends one JSON frame
per line:  {"ugv_adc":1,"seq":<counter>,"mv":[D1, ..., D8]}

A per-channel threshold is derived from white and black surface readings (calibration):
the threshold is the midpoint of the two medians, and the side black falls on is
determined channel by channel.
"""

import json
import os
import statistics

CHANNELS = 8
MAX_MV = 3600
MIN_SEPARATION_MV = 100   # at least this much difference between the white/black medians


def parse_frame(line):
    """Returns: (seq, [mv] * 8). ValueError on an invalid frame."""
    if len(line) > 512:
        raise ValueError('frame too long')
    data = json.loads(line)
    if not isinstance(data, dict) or data.get('ugv_adc') != 1:
        raise ValueError('not a ugv_adc frame')
    seq, values = data.get('seq'), data.get('mv')
    if type(seq) is not int or not 0 <= seq <= 0xffffffff:
        raise ValueError('invalid seq')
    if not isinstance(values, list) or len(values) != CHANNELS:
        raise ValueError(f'{CHANNELS} channels required')
    if any(type(v) is not int or not 0 <= v <= MAX_MV for v in values):
        raise ValueError('invalid millivolt value')
    return seq, values


class FrameReader:
    """Splits bytes from the serial port into lines; returns the last valid frame."""

    def __init__(self):
        self.buffer = b''
        self.last_seq = None

    def feed(self, data):
        """Returns: the values of the newest frame completed by these bytes, or None."""
        self.buffer += data
        if len(self.buffer) > 8192:      # if no line ending ever arrives (wrong baud)
            self.buffer = b''
        latest = None
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            try:
                seq, values = parse_frame(line)
            except (ValueError, UnicodeError):
                continue
            if seq == self.last_seq:
                continue
            self.last_seq = seq
            latest = values
        return latest


def make_calibration(white_frames, black_frames):
    """Per-channel threshold from frames collected on white and black surfaces.
    ValueError if the channels don't separate (the message names the channel)."""
    if not white_frames or not black_frames:
        raise ValueError('white and black readings required')
    white = [statistics.median(c) for c in zip(*white_frames)]
    black = [statistics.median(c) for c in zip(*black_frames)]
    white_range = [(min(c), max(c)) for c in zip(*white_frames)]
    black_range = [(min(c), max(c)) for c in zip(*black_frames)]
    for i, (w, b) in enumerate(zip(white, black)):
        if abs(w - b) < MIN_SEPARATION_MV:
            raise ValueError(f'D{i + 1}: white {w:.0f} mV, black {b:.0f} mV; '
                             f'difference below {MIN_SEPARATION_MV} mV')
        overlap = (white_range[i][1] >= black_range[i][0] if b > w
                   else black_range[i][1] >= white_range[i][0])
        if overlap:
            raise ValueError(f'D{i + 1}: white and black readings overlap')
    return {'white_mv': white, 'black_mv': black}


def validate_calibration(data):
    for name in ('white_mv', 'black_mv'):
        values = data.get(name) if isinstance(data, dict) else None
        if not isinstance(values, list) or len(values) != CHANNELS:
            raise ValueError(f'{name}: {CHANNELS} values required')
        if any(type(v) not in (int, float) or not 0 <= v <= MAX_MV for v in values):
            raise ValueError(f'{name}: invalid value')
    if any(abs(w - b) < MIN_SEPARATION_MV
           for w, b in zip(data['white_mv'], data['black_mv'])):
        raise ValueError(f'each channel needs at least {MIN_SEPARATION_MV} mV difference')
    return data


def load_calibration(path):
    """None if the file is missing; ValueError/OSError if it is corrupt."""
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return validate_calibration(json.load(f))


def save_calibration(path, calibration):
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(calibration, f, indent=2)
    os.replace(tmp, path)


def dark_channels(values, calibration):
    """Channels that resemble the black surface. Without calibration, none of them."""
    if calibration is None:
        return [False] * len(values)
    return [v > (w + b) / 2 if b > w else v < (w + b) / 2
            for v, w, b in zip(values, calibration['white_mv'], calibration['black_mv'])]
