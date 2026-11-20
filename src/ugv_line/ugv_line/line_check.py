"""Reads the line sensor live without ROS: channel order and QTR threshold tuning.

  ros2 run ugv_line line_check --config hardware.yaml

Move the sensor left to right over the surface first, then over the line:
the values should change in order from left to right. For QTR, choose a
`qtr.threshold_us` that lies between the surface and line values.
"""

import argparse
import sys
import time

import yaml

from ugv_hal.gpio import resolve_pin
from ugv_line.line_sensor_node import PinGroup
from ugv_line.qtr import read_decay_us


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', required=True, help='path to hardware.yaml')
    args = parser.parse_args(argv)

    with open(args.config) as f:
        params = (yaml.safe_load(f) or {}).get('line_sensor_node', {}).get('ros__parameters', {})
    if params.get('type') == 'serial':
        sys.exit('type: serial (Deneyap) live values are read while the robot is running:\n'
                 '  ros2 topic echo /line/state --field raw     # D1..D8 mV\n'
                 'calibration: https://docs.t3gemstone.org/en/projects/ugv-ros/usage'
                 '#3-line-sensor-calibration')
    pins = [resolve_pin(s) for s in params.get('pins', []) if s.strip()]
    if not pins:
        sys.exit('line_sensor_node.pins is empty')
    qtr = params.get('type', 'qtr_rc') == 'qtr_rc'
    qtr_cfg = params.get('qtr', {})
    threshold = qtr_cfg.get('threshold_us', 1000)
    timeout = qtr_cfg.get('timeout_us', 2500)
    group = PinGroup(pins, 'gpiod', 'flex' if qtr else 'input',
                     params.get('digital', {}).get('bias', 'as-is'))

    print('left to right ' + ('discharge time [µs], * = over threshold (dark)' if qtr else 'level'))
    print('Press Ctrl+C to exit.')
    try:
        while True:
            if qtr:
                raw = read_decay_us(group, len(pins), timeout)
                cells = [f'{t:5d}{"*" if t >= threshold else " "}' for t in raw]
            else:
                cells = [f'{v:3d}' for v in group.get_all()]
            print('\r  ' + ' '.join(cells), end='', flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        group.release()


if __name__ == '__main__':
    main()
