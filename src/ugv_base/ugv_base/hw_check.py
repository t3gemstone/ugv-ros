"""Field test tool: motor direction, encoder direction/resolution and speed measurement.

Needs no ROS and reads the same hardware.yaml as ugv_base_node. Put the robot
on a box so that the wheels are off the ground.

  ros2 run ugv_base hw_check pins     --config hardware.yaml
  ros2 run ugv_base hw_check motors   --config hardware.yaml [--duty 50] [--seconds 2]
  ros2 run ugv_base hw_check encoders --config hardware.yaml
  ros2 run ugv_base hw_check sweep    --config hardware.yaml

pins     : resolves every pin and PWM in the file to a line on the board and finds clashes.
motors   : drives each motor forward in turn; asks whether the wheel turned
           forward and suggests invert from the encoder sign and measured speed.
encoders : does not drive the motors; turn the wheel exactly ONE revolution forward
           by hand and read the count -> encoder.ppr = count / multiplier (x4: 4, x2: 2).
sweep    : raises the duty from 0 and measures the min_duty at which the wheel
           starts to move and the speed at 100% (max_wheel_speed).
For the line sensor: ros2 run ugv_line line_check --config hardware.yaml
"""

import argparse
import math
import sys
import time

import yaml

from ugv_base.config import flatten, load_yaml_params
from ugv_base.drivetrain import build_drivetrain
from ugv_base.encoder import multiplier
from ugv_hal.gpio import libgpiod_api, resolve_pin
from ugv_hal.pwm import describe_pwm_chips, resolve_pwm

SIDES = ('left', 'right')
SIDE_LABEL = {'left': 'LEFT', 'right': 'RIGHT'}


def _cpr(cfg):
    return cfg['encoder.ppr'] * multiplier(cfg['encoder.mode'])


def _speed(cfg, counts, seconds):
    cpr = _cpr(cfg)
    if cpr <= 0:
        return None
    return counts / cpr * 2.0 * math.pi * cfg['wheel_radius'] / seconds


def _run(dt, side, duty, seconds):
    duties = [duty if side == 'left' else 0.0, duty if side == 'right' else 0.0]
    before = dt.counts() or (0, 0)
    dt.set_duty(*duties)
    time.sleep(seconds)
    dt.set_duty(0.0, 0.0)
    after = dt.counts() or (0, 0)
    time.sleep(0.5)
    return [after[i] - before[i] for i in range(2)]


def cmd_motors(cfg, dt, args):
    print('Stand behind the robot facing forward (towards the line sensor); '
          'left/right are your left/right.')
    for idx, side in enumerate(SIDES):
        input(f'\n[{SIDE_LABEL[side]}] Enter -> drive forward at {args.duty:.0f}% '
              f'for {args.seconds:.1f} s...')
        delta = _run(dt, side, args.duty, args.seconds)
        answer = input(f'Did the {SIDE_LABEL[side]} wheel turn? [y/n]: ').strip().lower()
        if answer.startswith('n'):
            print('  -> swap the left and right blocks (including invert / enc_invert)')
        answer = input('Did the wheel turn in the direction that moves the robot FORWARD? '
                       '[y/n]: ').strip().lower()
        physical_forward = not answer.startswith('n')
        if not physical_forward:
            print(f'  -> flip {side}.invert (currently {cfg[side + ".invert"]})')
        if dt.counts() is None:
            continue
        other = delta[1 - idx]
        print(f'  encoder count: {delta[idx]} (other wheel: {other})')
        if delta[idx] == 0:
            print('  !! no counts: check the enc_a/enc_b pins, the supply and the bias setting')
        elif abs(other) > abs(delta[idx]) * 0.5:
            print('  !! the other wheel is counting too: '
                  'the left/right encoder pins may be swapped')
        elif (delta[idx] > 0) != physical_forward:
            # The count must have the same sign as the wheel's actual direction of rotation.
            current = cfg[f'{side}.enc_invert']
            print(f'  -> flip {side}.enc_invert (currently {current})')
        speed = _speed(cfg, abs(delta[idx]), args.seconds)
        if speed and delta[idx] != 0:
            print(f'  measured speed: {speed:.3f} m/s @ {args.duty:.0f}%')


def cmd_encoders(cfg, dt, args):
    if dt.counts() is None:
        sys.exit('encoder.mode: none, no encoders to measure')
    print('Turn the wheel exactly one revolution FORWARD by hand. Ctrl+C to quit.')
    start = dt.counts()
    mult = multiplier(cfg['encoder.mode'])
    try:
        while True:
            now = dt.counts()
            delta = [now[i] - start[i] for i in range(2)]
            errors = dt.encoder_errors()
            print(f'\r  left: {delta[0]:7d} (ppr≈{delta[0] / mult:7.1f})   '
                  f'right: {delta[1]:7d} (ppr≈{delta[1] / mult:7.1f})   '
                  f'errors: {errors[0]}/{errors[1]}   ', end='', flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


def cmd_sweep(cfg, dt, args):
    if dt.counts() is None:
        sys.exit('sweep needs encoders')
    for side in SIDES:
        input(f'\n[{SIDE_LABEL[side]}] Enter -> start the duty sweep...')
        min_duty = None
        for duty in range(5, 101, 5):
            delta = _run(dt, side, float(duty), 1.0)
            idx = SIDES.index(side)
            speed = _speed(cfg, abs(delta[idx]), 1.0)
            print(f'  {duty:3d}%: {abs(delta[idx]):6d} counts  {speed:.3f} m/s')
            if min_duty is None and abs(delta[idx]) > 0:
                min_duty = duty
        print(f'  -> pid.min_duty ≈ {min_duty}, max_wheel_speed ≈ {speed:.3f} '
              '(use the lower value of the two wheels)')


def _sections(path):
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    return {node: flatten(body.get('ros__parameters', {})) for node, body in doc.items()}


def cmd_pins(path):
    """Lists all pins without opening the hardware (name resolution only)."""
    api = libgpiod_api()
    if api == 0:
        sys.exit("no libgpiod Python bindings: run this command on the robot, inside the container")
    print(f'libgpiod python API: v{api}')
    used, problems = {}, []
    for node, params in _sections(path).items():
        for key, value in params.items():
            specs = value if isinstance(value, list) else [value]
            for i, spec in enumerate(specs):
                if not isinstance(spec, str) or not spec.strip():
                    continue
                name = f'{node}.{key}' + (f'[{i}]' if isinstance(value, list) else '')
                if key.split('.')[-1] in ('pwm',):
                    try:
                        chip, channel = resolve_pwm(spec)
                        print(f'  {name:38s} {spec:10s} -> pwmchip{chip}/pwm{channel}')
                    except ValueError as e:
                        problems.append(f'{name}: {e}')
                    continue
                if not (spec.upper().startswith('GPIO') or spec.startswith('gpiochip')):
                    continue
                try:
                    pin = resolve_pin(spec)
                except ValueError as e:
                    problems.append(f'{name}: {e}')
                    continue
                print(f'  {name:38s} {spec:10s} -> {pin[0]}:{pin[1]}')
                if pin in used:
                    problems.append(f'{name} and {used[pin]} share a line: {spec}')
                used[pin] = name
    print('\nPWM chips:')
    for row in describe_pwm_chips() or ['  (none: is the PWM overlay enabled? see the docs)']:
        print(f'  {row}')
    if problems:
        print('\nPROBLEMS:')
        for problem in problems:
            print(f'  !! {problem}')
        sys.exit(1)
    print('\nAll pins resolved, no clashes.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=['pins', 'motors', 'encoders', 'sweep'])
    parser.add_argument('--config', required=True, help='path to hardware.yaml')
    parser.add_argument('--duty', type=float, default=50.0)
    parser.add_argument('--seconds', type=float, default=2.0)
    args = parser.parse_args(argv)
    if args.command == 'pins':
        return cmd_pins(args.config)

    cfg = load_yaml_params(args.config)
    if args.command == 'sweep' and _cpr(cfg) <= 0:
        sys.exit('set encoder.ppr first (hw_check encoders)')
    dt = build_drivetrain(cfg)
    try:
        {'motors': cmd_motors, 'encoders': cmd_encoders, 'sweep': cmd_sweep}[args.command](
            cfg, dt, args)
    except KeyboardInterrupt:
        print()
    finally:
        dt.set_duty(0.0, 0.0)
        dt.close()


if __name__ == '__main__':
    main()
