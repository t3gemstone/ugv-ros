"""map_manager_node: saves and resets the SLAM map.

/ugv/save_map  (ugv_interfaces/SaveMap): saves the map on the robot.
/ugv/reset_map (std_srvs/Trigger): resets the EKF pose (odom), and in mapping
               mode also resets the slam_toolbox map; used after the robot has
               been carried somewhere else by hand.

A single call writes two groups of files (under map_dir, `name` without extension):
  name.pgm / name.yaml        -> image map (Nav2 map_server, sharing)
  name.posegraph / name.data  -> slam_toolbox pose graph; later opened with
                                 `slam_mode:=localization map:=<path>`
"""

from datetime import datetime
import os
import re
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_localization.srv import SetPose
from slam_toolbox.srv import Reset, SaveMap, SerializePoseGraph
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ugv_interfaces.srv import SaveMap as UgvSaveMap

_NAME = re.compile(r'^[A-Za-z0-9_\-]{1,64}$')


class MapManagerNode(Node):

    def __init__(self):
        super().__init__('map_manager_node')
        self.declare_parameter('map_dir', '~/ugv_maps')
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('slam_mode', 'mapping')
        self.declare_parameter('odom_frame', 'odom')
        self.map_dir = os.path.expanduser(self.get_parameter('map_dir').value)
        self.timeout = self.get_parameter('timeout').value
        os.makedirs(self.map_dir, exist_ok=True)

        group = ReentrantCallbackGroup()
        self.save_client = self.create_client(
            SaveMap, 'slam_toolbox/save_map', callback_group=group)
        self.serialize_client = self.create_client(
            SerializePoseGraph, 'slam_toolbox/serialize_map', callback_group=group)
        self.reset_client = self.create_client(
            Reset, 'slam_toolbox/reset', callback_group=group)
        self.set_pose_client = self.create_client(SetPose, 'set_pose', callback_group=group)
        self.create_service(UgvSaveMap, 'ugv/save_map', self._on_save, callback_group=group)
        self.create_service(Trigger, 'ugv/reset_map', self._on_reset, callback_group=group)
        self.get_logger().info(f'maps: {self.map_dir}')

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        # Reentrant group + multi-threaded executor: another thread handles
        # the response, so waiting here does not deadlock.
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self.timeout):
            return None
        return future.result()

    def _on_save(self, request, response):
        name = request.name.strip() or datetime.now().strftime('map_%Y%m%d_%H%M%S')
        if not _NAME.match(name):
            response.success = False
            response.message = 'name may only contain letters, digits, _ and -'
            return response
        path = os.path.join(self.map_dir, name)

        image = self._call(self.save_client, SaveMap.Request(name=String(data=path)))
        graph = self._call(self.serialize_client, SerializePoseGraph.Request(filename=path))
        problems = []
        if image is None or image.result != 0:
            problems.append('image map')
        if graph is None or graph.result != 0:
            problems.append('pose graph')

        response.path = path
        response.success = not problems
        response.message = (f'saved: {path}' if not problems else
                            'could not save: ' + ', '.join(problems) +
                            ' (is slam_toolbox running?)')
        log = self.get_logger().info if response.success else self.get_logger().error
        log(response.message)
        return response

    def _on_reset(self, request, response):
        problems = []
        # Pose first: the new map should start from where the robot is now.
        pose = SetPose.Request()
        pose.pose.header.frame_id = self.get_parameter('odom_frame').value
        pose.pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.pose.orientation.w = 1.0
        if self._call(self.set_pose_client, pose) is None:
            problems.append('pose (EKF set_pose)')

        if self.get_parameter('slam_mode').value == 'mapping':
            result = self._call(self.reset_client, Reset.Request())
            if result is None or result.result != Reset.Response.RESULT_SUCCESS:
                problems.append('map (slam_toolbox reset)')
            what = 'map and pose'
        else:
            what = 'pose (the saved map is kept in localization mode)'

        response.success = not problems
        response.message = (f'{what} reset' if not problems
                            else 'could not reset: ' + ', '.join(problems))
        log = self.get_logger().info if response.success else self.get_logger().error
        log(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
