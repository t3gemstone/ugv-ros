"""slam_toolbox: mapping, or localization on a saved map.

  slam_mode:=mapping                       new map (default)
  slam_mode:=localization map:=<path>      localize on a saved pose graph
                                           (<path> without extension, e.g. ~/ugv_maps/salon)

In Jazzy slam_toolbox is a lifecycle node; configure/activate are done here.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def _setup(context):
    mode = context.launch_configurations['slam_mode']
    map_path = os.path.expanduser(context.launch_configurations['map'])
    if mode not in ('mapping', 'localization'):
        raise RuntimeError(f"slam_mode must be mapping or localization, got: {mode}")

    use_sim_time = context.launch_configurations['use_sim_time'].lower() in ('true', '1', 'yes')
    overrides = {'mode': mode, 'use_sim_time': use_sim_time}
    if mode == 'localization':
        if not map_path:
            raise RuntimeError('localization needs map:=<pose graph path without extension>')
        if not os.path.exists(map_path + '.posegraph'):
            raise RuntimeError(f'{map_path}.posegraph not found')
        overrides.update({'map_file_name': map_path, 'map_start_at_dock': True})

    share = FindPackageShare('ugv_navigation').perform(context)
    params = os.path.join(share, 'config', 'slam.yaml')
    node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node' if mode == 'mapping'
        else 'localization_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[params, overrides],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE))]))
    return [node, configure, activate]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('slam_mode', default_value='mapping'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
