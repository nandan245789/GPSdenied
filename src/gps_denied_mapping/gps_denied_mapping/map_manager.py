"""Map manager — monitors map health and staleness."""

import rclpy
from rclpy.node import Node


class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')
        self.declare_parameter('map_stale_timeout', 5.0)
        self.get_logger().info('MapManager initialized')


def main(args=None):
    rclpy.init(args=args)
    node = MapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
