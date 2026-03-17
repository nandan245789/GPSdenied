"""Obstacle detector stub node — detects obstacles from depth data."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.declare_parameter('min_distance', 0.3)
        self.declare_parameter('max_distance', 5.0)
        self.depth_sub = self.create_subscription(
            Image, '/perception/depth_filtered', self.depth_callback, 10
        )
        self.get_logger().info('ObstacleDetector initialized')

    def depth_callback(self, msg):
        """TODO: Segment near-field obstacles from depth image."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
