"""Starts the UGV robot with a single command.

  ros2 launch ugv_bringup robot.launch.py                    # full robot, mapping
  ros2 launch ugv_bringup robot.launch.py nav:=true          # + Nav2 goal navigation
  ros2 launch ugv_bringup robot.launch.py slam_mode:=localization map:=~/ugv_maps/salon nav:=true
  ros2 launch ugv_bringup robot.launch.py backend:=mock      # no hardware (laptop)
  ros2 launch ugv_simulation sim.launch.py                   # Gazebo (runs backend:=sim)

Arguments:
  backend      gpiod | mock | sim
                                mock: motor/encoder simulation, IMU and lidar off
                                sim: started by ugv_simulation's sim.launch.py; Gazebo drives
                                the wheels and publishes IMU and lidar, ugv_simulation's
                                line_sensor_sim stands in for the line sensor, every node
                                runs on simulated time
  hardware     <yaml>           pin/hardware file (default config/hardware.yaml)
  imu          true | false     ICM-20948
  fusion       true | false     EKF (wheel + IMU) publishes the odom TF;
                                if false, ugv_base_node publishes it
  lidar        true | false     RPLIDAR A1 (sllidar_ros2, via lidar_manager_node)
  lidar_source <empty> | usb | gpio  if empty, the connection selected and saved in the UI,
                                or failing that the one in hardware.yaml (usb)
  lidar_port   <empty>          set the serial port manually (e.g. /dev/ttyUSB1)
  line         true | false     line sensor + line following (if hardware.yaml has
                                type: serial, the Deneyap UART is closed to the GPIO lidar)
  slam         true | false     slam_toolbox + map saving service
  slam_mode    mapping | localization
  map          <path>           pose graph path without extension, for localization
  nav          true | false     Nav2
  gcs          true | false     rosbridge + web UI (http://<robot>:8000)
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

ARGS = {
    'backend': 'gpiod',
    'hardware': '',
    'imu': 'true',
    'fusion': 'true',
    'lidar': 'true',
    'lidar_source': '',
    'lidar_port': '',
    'line': 'true',
    'slam': 'true',
    'slam_mode': 'mapping',
    'map': '',
    'nav': 'false',
    'gcs': 'true',
}


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
    backend = arg['backend']
    mock = backend == 'mock'
    sim = backend == 'sim'
    bringup = _share('ugv_bringup', context)
    hardware = (os.path.expanduser(arg['hardware'])
                or os.path.join(bringup, 'config', 'hardware.yaml'))
    control = os.path.join(bringup, 'config', 'control.yaml')
    with open(os.path.join(_share('ugv_description', context), 'config', 'robot.yaml')) as f:
        geometry = yaml.safe_load(f)

    with open(hardware) as f:
        line_params = (yaml.safe_load(f) or {}).get('line_sensor_node', {}).get(
            'ros__parameters', {})
    # If the Deneyap line sensor uses the UART, the GPIO lidar must not open the same port.
    line_port = (line_params.get('serial', {}).get('port', '/dev/ttyS3')
                 if flag('line') and line_params.get('type') == 'serial' else '')

    fusion = flag('fusion')
    odom_topic = '/odometry/filtered' if fusion else '/wheel/odom'
    # In simulation Gazebo stands in for ugv_base_node, ugv_imu_node and the lidar driver.
    actions = [SetParameter(name='use_sim_time', value=True)] if sim else []
    actions.append(_include('ugv_description', 'description.launch.py', context))

    if not sim:
        actions.append(Node(
            package='ugv_base', executable='ugv_base_node', output='screen',
            parameters=[hardware, {
                'backend': backend,
                'wheel_radius': float(geometry['wheel_radius']),
                'wheel_separation': float(geometry['wheel_separation']),
                'publish_tf': not fusion,
            }]))

    if flag('imu') and not mock and not sim:
        actions.append(Node(package='ugv_imu', executable='ugv_imu_node', output='screen',
                            parameters=[hardware]))
    if fusion:
        actions.append(Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            output='screen', parameters=[os.path.join(bringup, 'config', 'ekf.yaml')]))

    if flag('lidar') and not mock and not sim:
        actions.append(Node(
            package='ugv_hal', executable='lidar_manager_node', output='screen',
            parameters=[hardware, {
                'backend': backend,
                'source_override': arg['lidar_source'],
                'port_override': arg['lidar_port'],
                'busy_port': line_port,
            }]))

    if flag('line'):
        if not sim:
            actions.append(Node(package='ugv_line', executable='line_sensor_node',
                                output='screen', parameters=[hardware, {'backend': backend}]))
        actions.append(Node(package='ugv_line', executable='line_follower_node', output='screen',
                            parameters=[control]))

    chassis = geometry['chassis']
    actions.append(Node(
        package='ugv_control', executable='mode_manager_node', output='screen',
        parameters=[control, {
            'footprint.front': float(chassis['x'] + chassis['length'] / 2),
            'footprint.rear': float(chassis['length'] / 2 - chassis['x']),
            'footprint.half_width': float(chassis['width'] / 2),
            'lidar.x': float(geometry['lidar']['x']),
            'lidar.y': float(geometry['lidar']['y']),
            'lidar.yaw': float(geometry['lidar']['yaw']),
        }]))

    if flag('slam'):
        actions.append(_include('ugv_navigation', 'slam.launch.py', context,
                                slam_mode=arg['slam_mode'], map=arg['map'],
                                use_sim_time=str(sim).lower()))
        actions.append(Node(package='ugv_navigation', executable='map_manager_node',
                            output='screen',
                            parameters=[control, {'slam_mode': arg['slam_mode']}]))
    if flag('nav'):
        actions.append(_include('ugv_navigation', 'nav.launch.py', context, odom_topic=odom_topic))
    if flag('gcs'):
        actions.append(_include('ugv_gcs', 'gcs.launch.py', context))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in ARGS.items()]
        + [OpaqueFunction(function=_setup)])
