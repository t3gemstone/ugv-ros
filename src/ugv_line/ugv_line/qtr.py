"""Pololu QTR-xRC (RC type) reflectance sensor reading.

Each channel is a capacitor + phototransistor:
  1. the line is driven HIGH, the capacitor charges for `charge_us`
  2. the line is switched to input; the phototransistor discharges the capacitor
  3. the time for the line to fall LOW is measured: white (high reflectance) is short,
     black (low reflectance) is long; a channel exceeding `timeout_us` counts as timed out
Channels are measured together; one reading takes at most timeout_us.
"""

import time


def read_decay_us(lines, count: int, timeout_us: int = 2500, charge_us: int = 10,
                  clock=time.perf_counter_ns):
    """lines: request object providing drive_high() / to_input() / get_all().
    Returns: per-channel discharge time [µs], at most timeout_us."""
    lines.drive_high()
    end_charge = clock() + charge_us * 1000
    while clock() < end_charge:
        pass
    lines.to_input()
    start = clock()
    times = [timeout_us] * count
    pending = set(range(count))
    while pending:
        elapsed = (clock() - start) // 1000
        if elapsed >= timeout_us:
            break
        values = lines.get_all()
        for i in [i for i in pending if not values[i]]:
            times[i] = int(elapsed)
            pending.discard(i)
    return times


def dark_channels(times, threshold_us: int):
    """Channels whose time exceeds the threshold are seeing a dark surface."""
    return [t >= threshold_us for t in times]
