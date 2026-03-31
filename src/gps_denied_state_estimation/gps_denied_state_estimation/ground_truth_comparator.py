"""Ground truth comparator — measures VIO drift vs Gazebo ground truth.

Subscribes to:
  /state_estimation/odom — VIO estimated pose
  /model/x500/pose — Gazebo ground truth pose (via ros_gz bridge)

Publishes:
  /telemetry/gt_error — real-time position error (Float32)
  /telemetry/gt_stats — periodic RMSE statistics (String)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
import numpy as np
import math


class GroundTruthComparator(Node):
    """Compare VIO output against simulation ground truth."""

    def __init__(self):
        super().__init__('ground_truth_comparator')

        self.declare_parameter('gt_topic', '/model/x500/pose')
        self.declare_parameter('report_interval', 5.0)
        gt_topic = self.get_parameter('gt_topic').value

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # Subscribers
        self.vio_sub = self.create_subscription(
            Odometry, '/state_estimation/odom', self.vio_callback, reliable_qos
        )
        self.gt_sub = self.create_subscription(
            PoseStamped, gt_topic, self.gt_callback, sensor_qos
        )

        # Publishers
        self.error_pub = self.create_publisher(Float32, '/telemetry/gt_error', 10)
        self.stats_pub = self.create_publisher(String, '/telemetry/gt_stats', 10)

        # State
        self.vio_pos = None
        self.gt_pos = None
        self.errors = []
        self.total_distance = 0.0
        self.prev_gt_pos = None

        # Periodic report
        report_interval = self.get_parameter('report_interval').value
        self.report_timer = self.create_timer(report_interval, self.publish_report)

        self.get_logger().info(f'GroundTruthComparator initialized — GT topic: {gt_topic}')

    def vio_callback(self, msg: Odometry):
        self.vio_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        self._compute_error()

    def gt_callback(self, msg: PoseStamped):
        self.gt_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        # Track total distance traveled
        if self.prev_gt_pos is not None:
            self.total_distance += np.linalg.norm(self.gt_pos - self.prev_gt_pos)
        self.prev_gt_pos = self.gt_pos.copy()

    def _compute_error(self):
        if self.vio_pos is None or self.gt_pos is None:
            return
        error = np.linalg.norm(self.vio_pos - self.gt_pos)
        self.errors.append(error)

        err_msg = Float32()
        err_msg.data = float(error)
        self.error_pub.publish(err_msg)

    def publish_report(self):
        if len(self.errors) < 10:
            return
        errors = np.array(self.errors)
        rmse = float(np.sqrt(np.mean(errors ** 2)))
        max_err = float(np.max(errors))
        mean_err = float(np.mean(errors))
        drift_pct = (rmse / max(self.total_distance, 0.1)) * 100

        report = (
            f'RMSE={rmse:.3f}m | Mean={mean_err:.3f}m | '
            f'Max={max_err:.3f}m | Drift={drift_pct:.2f}% | '
            f'Dist={self.total_distance:.1f}m | N={len(self.errors)}'
        )

        stats_msg = String()
        stats_msg.data = report
        self.stats_pub.publish(stats_msg)
        self.get_logger().info(f'GT comparison: {report}')


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthComparator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
