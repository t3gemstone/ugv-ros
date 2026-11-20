"""line_sensor_sim: the simulated line sensor array for Gazebo.

Stands in for ugv_line's line_sensor_node in simulation. It reads the image of the
downward camera on line_sensor_link (/line_sensor/image, mono8) and publishes
/line/state like the real node: channels left to right, raw values in millivolts
(dark = high, as from the Deneyap ADC), the line position and intersections from
ugv_line.line_math. line_follower_node and the web UI therefore work unchanged.

A channel sees the line when its brightness is below `threshold` (0..1). Unlike the real
array, the camera has no light of its own: the black tape reads about 0.05 in any light,
the floor 0.6 in sunlight and down to about 0.25 in the robot's shadow, so the threshold
sits between the two. `half_width` must match the camera footprint in
urdf/ugv_sim.urdf.xacro.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ugv_interfaces.msg import LineState
from ugv_line.line_math import read_line, sensor_positions

from ugv_simulation.line_camera import channel_levels, sensor_columns, to_millivolts


class LineSensorSim(Node):

    def __init__(self):
        super().__init__('line_sensor_sim')
        count = self.declare_parameter('count', 8).value
        spacing = self.declare_parameter('spacing', 0.009525).value
        self.half_width = self.declare_parameter('half_width', 0.04).value
        self.threshold = self.declare_parameter('threshold', 0.15).value
        self.positions = sensor_positions(count, spacing)
        self.pub = self.create_publisher(LineState, 'line/state', 10)
        self.create_subscription(Image, 'line_sensor/image', self._on_image,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f'{count} channels, {spacing * 1000:.3f} mm apart, threshold {self.threshold}')

    def _on_image(self, msg: Image):
        if msg.encoding != 'mono8':
            self.get_logger().error(f'expected a mono8 image, got {msg.encoding}',
                                    throttle_duration_sec=5.0)
            return
        columns = sensor_columns(msg.width, self.positions, self.half_width)
        levels = channel_levels(msg.data, msg.width, msg.height, msg.step, columns)
        reading = read_line([level < self.threshold for level in levels], self.positions)
        state = LineState()
        state.header = msg.header
        state.sensors = reading.sensors
        state.raw = to_millivolts(levels)
        state.detected = reading.detected
        state.position = float(reading.position)
        state.intersection = reading.intersection
        self.pub.publish(state)


def main():
    rclpy.init()
    node = LineSensorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
