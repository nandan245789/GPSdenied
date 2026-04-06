"""Terrain-aided navigation — position correction using terrain matching.

In GPS-denied outdoor ops, VIO drifts over long distances (km).
Terrain matching corrects this by comparing the downward camera's
view of terrain against pre-loaded satellite imagery/elevation data.

How it works:
  1. Downward camera captures terrain texture
  2. ORB features are extracted from the live image
  3. Features are matched against pre-loaded terrain database
  4. Matched position provides absolute correction to VIO
  5. Correction is fused as a position measurement

This is CRITICAL for border ops where VIO may drift >10m over 4km.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
import numpy as np
import time
import math


class TerrainMatcher(Node):
    """Correct VIO drift using terrain feature matching."""

    def __init__(self):
        super().__init__('terrain_matcher')

        self.declare_parameter('match_interval', 5.0)
        self.declare_parameter('min_altitude', 30.0)
        self.declare_parameter('max_correction', 20.0)
        self.declare_parameter('correction_gain', 0.3)
        self.declare_parameter('min_match_confidence', 0.5)

        self.match_interval = self.get_parameter('match_interval').value
        self.min_alt = self.get_parameter('min_altitude').value
        self.max_correction = self.get_parameter('max_correction').value
        self.correction_gain = self.get_parameter('correction_gain').value
        self.min_confidence = self.get_parameter('min_match_confidence').value

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/state_estimation/odom',
            self._on_odom, reliable_qos
        )

        # Publishers
        self.correction_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/terrain/position_correction', reliable_qos
        )
        self.confidence_pub = self.create_publisher(
            Float32, '/terrain/match_confidence', 10
        )
        self.status_pub = self.create_publisher(
            String, '/terrain/status', 10
        )

        # State
        self.drone_pos = np.zeros(3)
        self.cumulative_drift = np.zeros(2)
        self.total_distance = 0.0
        self.prev_pos = np.zeros(3)
        self.last_match_time = time.time()
        self.corrections_applied = 0
        self.avg_correction_m = 0.0

        # Simulated terrain database
        # In real system: loaded from GeoTIFF with georeferenced features
        self.terrain_features = self._load_terrain_db()

        # Terrain matching timer
        self.create_timer(self.match_interval, self.do_terrain_match)
        self.create_timer(5.0, self._publish_status)

        self.get_logger().info(
            f'TerrainMatcher initialized — '
            f'interval={self.match_interval}s, '
            f'{len(self.terrain_features)} reference points'
        )

    def _load_terrain_db(self):
        """Load terrain reference features.

        In production: reads GeoTIFF/satellite imagery and extracts
        feature descriptors at known GPS coordinates.

        For simulation: returns pre-defined reference points along
        the border patrol route.
        """
        # Reference terrain features along the patrol route
        # Each has: position (NED), descriptor (simulated), name
        return [
            {'pos': np.array([0, 0]), 'name': 'launch_site', 'type': 'structure'},
            {'pos': np.array([500, -50]), 'name': 'road_crossing_1', 'type': 'road'},
            {'pos': np.array([1000, 200]), 'name': 'clearing_1', 'type': 'clearing'},
            {'pos': np.array([1500, -100]), 'name': 'river_bend', 'type': 'river'},
            {'pos': np.array([2000, -500]), 'name': 'patrol_start_ref', 'type': 'structure'},
            {'pos': np.array([2500, 0]), 'name': 'road_crossing_2', 'type': 'road'},
            {'pos': np.array([3000, 100]), 'name': 'observation_ref', 'type': 'clearing'},
            {'pos': np.array([3500, -200]), 'name': 'forest_edge', 'type': 'treeline'},
            {'pos': np.array([4000, 500]), 'name': 'sweep_east_ref', 'type': 'road'},
            {'pos': np.array([4500, 0]), 'name': 'turnaround_ref', 'type': 'structure'},
        ]

    def _on_odom(self, msg):
        new_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        self.total_distance += np.linalg.norm(new_pos - self.prev_pos)
        self.prev_pos = new_pos.copy()
        self.drone_pos = new_pos

    def do_terrain_match(self):
        """Attempt terrain-based position correction.

        Algorithm (production):
          1. Capture downward camera image
          2. Extract ORB features
          3. Match against terrain DB using BFMatcher
          4. If good match (>50% confidence):
             - Compute position correction = DB_pos - VIO_pos
             - Apply with gain (0.3 = 30% correction per match)
             - Publish as position measurement for EKF fusion

        Algorithm (simulation):
          - Find nearest terrain reference point
          - Simulate matching with distance-based confidence
          - Add realistic noise to correction
        """
        if self.drone_pos[2] < self.min_alt:
            return  # Too low for terrain matching

        # Find nearest reference point
        min_dist = float('inf')
        nearest = None
        for feat in self.terrain_features:
            d = np.linalg.norm(self.drone_pos[:2] - feat['pos'])
            if d < min_dist:
                min_dist = d
                nearest = feat

        if nearest is None:
            return

        # Simulate match confidence (higher when closer to reference)
        # In production: based on feature descriptor match quality
        confidence = max(0.0, 1.0 - (min_dist / 500.0))  # 100% at ref, 0% at 500m

        # Publish confidence
        conf_msg = Float32()
        conf_msg.data = float(confidence)
        self.confidence_pub.publish(conf_msg)

        if confidence < self.min_confidence:
            return

        # Simulate position correction
        # VIO drift accumulates at ~0.5% of distance traveled
        estimated_drift = self.total_distance * 0.005
        drift_direction = np.random.randn(2)
        drift_direction = drift_direction / (np.linalg.norm(drift_direction) + 1e-6)
        simulated_drift = drift_direction * estimated_drift * 0.1

        # Correction = negative of accumulated drift (clamped)
        correction = -simulated_drift * self.correction_gain
        correction_magnitude = np.linalg.norm(correction)

        if correction_magnitude > self.max_correction:
            correction = correction / correction_magnitude * self.max_correction

        # Publish correction as PoseWithCovariance
        corr_msg = PoseWithCovarianceStamped()
        corr_msg.header.stamp = self.get_clock().now().to_msg()
        corr_msg.header.frame_id = 'local_ned'
        corr_msg.pose.pose.position.x = float(self.drone_pos[0] + correction[0])
        corr_msg.pose.pose.position.y = float(self.drone_pos[1] + correction[1])
        corr_msg.pose.pose.position.z = float(self.drone_pos[2])
        corr_msg.pose.pose.orientation.w = 1.0

        # Covariance — lower = more confident correction
        cov_value = 5.0 / max(confidence, 0.1)  # Lower cov when confident
        corr_msg.pose.covariance[0] = cov_value   # x
        corr_msg.pose.covariance[7] = cov_value   # y
        corr_msg.pose.covariance[14] = 10.0       # z (less certain)

        self.correction_pub.publish(corr_msg)

        self.corrections_applied += 1
        self.avg_correction_m = (
            (self.avg_correction_m * (self.corrections_applied - 1) +
             correction_magnitude) / self.corrections_applied
        )

        self.get_logger().info(
            f'🗺️ Terrain match: {nearest["name"]} ({nearest["type"]}) '
            f'conf={confidence:.0%} corr={correction_magnitude:.2f}m '
            f'dist_traveled={self.total_distance:.0f}m'
        )

    def _publish_status(self):
        status = String()
        status.data = (
            f'terrain_matches={self.corrections_applied} '
            f'avg_corr={self.avg_correction_m:.2f}m '
            f'dist={self.total_distance:.0f}m '
            f'alt={self.drone_pos[2]:.0f}m'
        )
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainMatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
