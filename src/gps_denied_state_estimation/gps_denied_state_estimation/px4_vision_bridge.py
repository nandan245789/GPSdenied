"""PX4 vision bridge — forwards VIO odom to PX4 EKF2 as external vision."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class PX4VisionBridge(Node):
    """Forward VIO odometry to PX4 as vehicle_visual_odometry.

    TODO: Subscribe to /state_estimation/odom, convert frame conventions
    (ENU→NED), publish to /fmu/in/vehicle_visual_odometry.
    """

    def __init__(self):
        super().__init__('px4_vision_bridge')
        self.odom_sub = self.create_subscription(
            Odometry, '/state_estimation/odom', self.odom_callback, 10
        )
        self.get_logger().info('PX4VisionBridge initialized')

    def odom_callback(self, msg: Odometry):
        """Convert and forward to PX4. TODO: ENU→NED transform."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PX4VisionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
