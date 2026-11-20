"""ugv_base_node parameters: defaults and YAML loading helpers.

The same definitions are used by both the ROS node and the ROS-free hw_check
tool, so both read the same hardware.yaml.

Pin: "GPIO22" (T3 Gemstone O1 line name) or "gpiochipN:line". PWM: "ecap0", "epwm1:1".
Empty string = unused.
NOTE (ROS 2): give float parameters a decimal value in YAML (1.0, not 1).
"""

DEFAULTS = {
    'backend': 'gpiod',               # gpiod | mock (simulation without hardware)
    'motor_driver': 'in_in_pwm',      # in_in_pwm (L298N/TB6612) | dir_pwm
    'brake_on_stop': False,
    'pwm_period_ns': 1_000_000,

    'left.in1': '', 'left.in2': '', 'left.pwm': '', 'left.invert': False,
    'right.in1': '', 'right.in2': '', 'right.pwm': '', 'right.invert': False,

    'encoder.mode': 'x4',             # x4 | x2 | none
    'encoder.bias': 'as-is',          # as-is | pull-up | pull-down | disabled
    'encoder.ppr': 0.0,               # channel A pulses per WHEEL (output shaft) revolution
    'left.enc_a': '', 'left.enc_b': '', 'left.enc_invert': False,
    'right.enc_a': '', 'right.enc_b': '', 'right.enc_invert': False,

    'wheel_radius': 0.0425,           # the launch file passes it from robot.yaml
    'wheel_separation': 0.17,
    'max_wheel_speed': 0.5,           # wheel speed at 100% duty [m/s]
    'max_accel': 0.6,                 # [m/s^2]
    'control_rate': 50.0,
    'state_rate': 10.0,          # /joint_states and ~/wheels (visualization/tuning only)
    'cmd_timeout': 0.5,
    'speed_filter_alpha': 0.5,

    'pid.kp': 60.0,                   # [duty % / (m/s)]
    'pid.ki': 300.0,
    'pid.kff': 1.0,
    'pid.min_duty': 0.0,
    'pid.max_duty': 100.0,
    'pid.i_limit': 30.0,              # upper limit of the integral's duty contribution [%]

    'odom_frame': 'odom',
    'base_frame': 'base_link',
    'publish_tf': False,
}


def flatten(tree: dict, prefix: str = '') -> dict:
    flat = {}
    for key, value in tree.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            flat.update(flatten(value, name + '.'))
        else:
            flat[name] = value
    return flat


def load_yaml_params(path: str, node_name: str = 'ugv_base_node') -> dict:
    """Returns this node's settings from a ROS parameter YAML as a flat dict."""
    import yaml
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    section = doc.get(node_name, doc.get(f'/{node_name}', {}))
    cfg = dict(DEFAULTS)
    cfg.update(flatten(section.get('ros__parameters', {})))
    return cfg
