"""Runs the UGV in Gazebo Harmonic.

  ros2 launch ugv_simulation sim.launch.py               # office world, Gazebo GUI, mapping
                                                         # (the robot starts on the line track)
  ros2 launch ugv_simulation sim.launch.py nav:=true     # + Nav2 goal navigation
  ros2 launch ugv_simulation sim.launch.py gui:=false    # headless (Gazebo server only)

Gazebo stands in for the hardware: the differential drive takes /cmd_vel and publishes
/wheel/odom and /joint_states, the IMU publishes /imu/data_raw and the lidar /scan, and
line_sensor_sim turns the image of a downward camera at the line sensor array into
/line/state. The rest of the stack (mode_manager_node, EKF, slam_toolbox, Nav2,
line_follower_node, web UI at http://localhost:8000) is ugv_bringup's robot.launch.py
with backend:=sim, on simulated time. The office world has a black line track; the robot
starts on it, so Line follow works right away.

Arguments:
  world      <sdf>          world file (default: worlds/office.sdf)
  gui        true | false   Gazebo GUI
  x, y, yaw  <m, m, rad>    start pose in the world (default: on the line track)
  line       true | false   simulated line sensor + line following
  fusion, slam, slam_mode, map, nav, gcs   passed to robot.launch.py
"""

import os

import xacro
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = {
    'world': '',
    'gui': 'true',
    'x': '-1.6',
    'y': '0.0',
    'yaw': '0.0',
    'line': 'true',
    'fusion': 'true',
    'slam': 'true',
    'slam_mode': 'mapping',
    'map': '',
    'nav': 'false',
    'gcs': 'true',
}
ROBOT_ARGS = ('line', 'fusion', 'slam', 'slam_mode', 'map', 'nav', 'gcs')


def _share(package, context):
    return FindPackageShare(package).perform(context)


def _include(package, launch_file, context, **arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_share(package, context), 'launch', launch_file)),
        launch_arguments=arguments.items())


def _setup(context):
    arg = context.launch_configurations
    flag = lambda name: arg[name].lower() in ('true', '1', 'yes')  # noqa: E731
    share = _share('ugv_simulation', context)
    world = os.path.expanduser(arg['world']) or os.path.join(share, 'worlds', 'office.sdf')
    robot = xacro.process_file(os.path.join(share, 'urdf', 'ugv_sim.urdf.xacro')).toxml()
    with open(os.path.join(_share('ugv_description', context), 'config', 'robot.yaml')) as f:
        wheel_radius = float(yaml.safe_load(f)['wheel_radius'])

    # Without a display the lidar still needs a renderer: --headless-rendering.
    gz_args = f'-r {world}' if flag('gui') else f'-r -s --headless-rendering {world}'
    actions = [
        _include('ros_gz_sim', 'gz_sim.launch.py', context,
                 gz_args=gz_args, on_exit_shutdown='true'),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'ugv', '-string', robot,
                        '-x', arg['x'], '-y', arg['y'], '-z', str(wheel_radius + 0.01),
                        '-Y', arg['yaw']]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             output='screen',
             parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml'),
                          'use_sim_time': True}]),
    ]
    if flag('line'):
        actions.append(Node(package='ugv_simulation', executable='line_sensor_sim',
                            output='screen', parameters=[{'use_sim_time': True}]))
    if not flag('fusion'):
        # Without the EKF the odom -> base_link TF comes from the differential drive,
        # as it comes from ugv_base_node on the robot.
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='gz_tf_bridge',
            output='screen', arguments=['/ugv/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            remappings=[('/ugv/tf', '/tf')], parameters=[{'use_sim_time': True}]))
    actions.append(_include('ugv_bringup', 'robot.launch.py', context, backend='sim',
                            **{name: arg[name] for name in ROBOT_ARGS}))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in ARGS.items()]
        + [OpaqueFunction(function=_setup)])
