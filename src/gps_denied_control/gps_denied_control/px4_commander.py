"""PX4 commander — handles mode transitions, arming, and emergency commands."""

import rclpy
from rclpy.node import Node


class PX4Commander(Node):
    """Manage PX4 mode transitions and emergency commands.

    Handles: arm/disarm, offboard mode engagement, emergency land,
    and RC override detection.
    """

    def __init__(self):
        super().__init__('px4_commander')
        self.declare_parameter('arm_timeout', 5.0)
        # TODO: Subscribe to /safety/failsafe_cmd, /fmu/out/vehicle_status
        # TODO: Publish to /fmu/in/vehicle_command
        self.get_logger().info('PX4Commander initialized')


def main(args=None):
    rclpy.init(args=args)
    node = PX4Commander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
