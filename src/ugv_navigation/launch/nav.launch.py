"""Nav2 servers (the map comes from slam_toolbox; no AMCL / map_server).

Velocity chain:
  controller_server / behavior_server -> cmd_vel/nav_raw
  velocity_smoother                   -> cmd_vel/nav  -> mode_manager_node -> cmd_vel
Goal: /goal_pose (GCS or RViz) or the navigate_to_pose action.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NODES = ['controller_server', 'smoother_server', 'planner_server',
         'behavior_server', 'velocity_smoother', 'bt_navigator']


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    odom_topic = LaunchConfiguration('odom_topic')
    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    raw = tf + [('cmd_vel', 'cmd_vel/nav_raw')]

    def server(package, executable, remappings=tf, extra=None):
        return Node(package=package, executable=executable, name=executable,
                    output='screen', parameters=[params, extra or {}],
                    remappings=remappings)

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [FindPackageShare('ugv_navigation'), 'config', 'nav2.yaml'])),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        server('nav2_controller', 'controller_server', raw),
        server('nav2_smoother', 'smoother_server'),
        server('nav2_planner', 'planner_server'),
        server('nav2_behaviors', 'behavior_server', raw),
        server('nav2_velocity_smoother', 'velocity_smoother',
               tf + [('cmd_vel', 'cmd_vel/nav_raw'), ('cmd_vel_smoothed', 'cmd_vel/nav')],
               {'odom_topic': odom_topic}),
        server('nav2_bt_navigator', 'bt_navigator', tf, {'odom_topic': odom_topic}),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True, 'node_names': NODES}],
        ),
    ])
