"""PX4 Vision Bridge — forwards VIO odometry to PX4 EKF2.

Converts from ROS2 ENU frame convention to PX4 NED frame convention
and publishes as vehicle_visual_odometry for EKF2 fusion.

Frame conventions:
  ROS2 (ENU): X=East, Y=North, Z=Up
  PX4  (NED): X=North, Y=East, Z=Down
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from builtin_interfaces.msg import Time as PX4Time
import numpy as np


class PX4VisionBridge(Node):
    """Convert VIO odometry (ENU) to PX4 vehicle_visual_odometry (NED)."""

    def __init__(self):
        super().__init__('px4_vision_bridge')

        self.declare_parameter('publish_rate', 30.0)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribe to VIO output
        self.odom_sub = self.create_subscription(
            Odometry, '/state_estimation/odom',
            self.odom_callback, reliable_qos
        )

        # Publish to PX4
        self.px4_pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', px4_qos
        )

        self.msg_count = 0
        self.get_logger().info('PX4VisionBridge initialized — ENU → NED conversion active')

    def odom_callback(self, msg: Odometry):
        """Convert ENU odometry to NED and publish to PX4."""
        px4_msg = VehicleOdometry()

        # Timestamp (microseconds)
        px4_msg.timestamp = (
            msg.header.stamp.sec * 1_000_000 +
            msg.header.stamp.nanosec // 1000
        )
        px4_msg.timestamp_sample = px4_msg.timestamp

        # Position: ENU → NED
        # NED.x = ENU.y (North)
        # NED.y = ENU.x (East)
        # NED.z = -ENU.z (Down)
        px4_msg.position = [
            float(msg.pose.pose.position.y),   # North
            float(msg.pose.pose.position.x),   # East
            float(-msg.pose.pose.position.z),  # Down
        ]

        # Velocity: ENU → NED
        px4_msg.velocity = [
            float(msg.twist.twist.linear.y),   # North
            float(msg.twist.twist.linear.x),   # East
            float(-msg.twist.twist.linear.z),  # Down
        ]

        # Quaternion: ENU → NED
        # q_ned = q_enu * R_enu_to_ned
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # ENU to NED quaternion rotation
        # Swap x↔y, negate z
        px4_msg.q = [
            float(qw),    # w first for PX4
            float(qy),    # North component
            float(qx),    # East component
            float(-qz),   # Down component
        ]

        # Pose frame and velocity frame
        px4_msg.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        px4_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_FRD

        # Position/velocity variance from covariance
        px4_msg.position_variance = [
            float(msg.pose.covariance[0]),   # x variance
            float(msg.pose.covariance[7]),   # y variance
            float(msg.pose.covariance[14]),  # z variance
        ]
        px4_msg.velocity_variance = [0.1, 0.1, 0.1]  # Default

        self.px4_pub.publish(px4_msg)

        self.msg_count += 1
        if self.msg_count % 90 == 0:  # Every 3 seconds at 30 Hz
            self.get_logger().info(
                f'PX4 bridge: {self.msg_count} msgs sent | '
                f'pos_ned=[{px4_msg.position[0]:.2f}, '
                f'{px4_msg.position[1]:.2f}, {px4_msg.position[2]:.2f}]'
            )


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


if __name__ == '__main__':
    main()
