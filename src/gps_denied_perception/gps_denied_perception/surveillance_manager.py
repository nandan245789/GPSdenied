"""Surveillance manager — target detection, recording, and geo-tagging.

The surveillance brain for border operations:
- Accepts camera mode commands from waypoint manager
- Simulates target detection (person, vehicle, structure)
- Geo-tags detections with position + timestamp
- Manages recording state
- Publishes detection alerts for ground station

Camera modes:
  standby          — Camera off, no processing
  continuous_record — Record video, basic detection
  360_scan         — Rotate yaw during loiter, wide-area scan
  thermal_scan     — Use thermal camera for heat signatures
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import numpy as np
import time
import json
import os


class SurveillanceManager(Node):
    """Manages surveillance payload — detection, recording, geo-tagging."""

    def __init__(self):
        super().__init__('surveillance_manager')

        self.declare_parameter('recording_path', '/workspace/recordings')
        self.declare_parameter('detection_confidence_min', 0.5)
        self.declare_parameter('alert_on_detection', True)
        self.declare_parameter('detection_cooldown', 10.0)

        self.recording_path = self.get_parameter('recording_path').value
        self.min_confidence = self.get_parameter('detection_confidence_min').value
        self.alert_enabled = self.get_parameter('alert_on_detection').value
        self.detection_cooldown = self.get_parameter('detection_cooldown').value

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/state_estimation/odom',
            self._on_odom, reliable_qos
        )
        self.camera_cmd_sub = self.create_subscription(
            String, '/surveillance/camera_command',
            self._on_camera_cmd, 10
        )

        # Publishers
        self.detection_pub = self.create_publisher(String, '/surveillance/detections', 10)
        self.alert_pub = self.create_publisher(String, '/surveillance/alert', 10)
        self.status_pub = self.create_publisher(String, '/surveillance/status', 10)
        self.recording_active_pub = self.create_publisher(Bool, '/surveillance/recording', 10)

        # State
        self.drone_pos = np.zeros(3)
        self.camera_mode = 'standby'
        self.recording = False
        self.detections = []
        self.last_detection_time = 0
        self.mission_start_time = time.time()

        # Recording session
        self.session_id = int(time.time())
        os.makedirs(self.recording_path, exist_ok=True)

        # Status timer
        self.create_timer(2.0, self._publish_status)

        self.get_logger().info(
            f'SurveillanceManager initialized — '
            f'session={self.session_id}'
        )

    def _on_odom(self, msg):
        self.drone_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])

    def _on_camera_cmd(self, msg):
        old_mode = self.camera_mode
        self.camera_mode = msg.data

        if old_mode != self.camera_mode:
            self.get_logger().info(f'📷 Camera: {old_mode} → {self.camera_mode}')

        # Manage recording state
        if self.camera_mode in ('continuous_record', '360_scan', 'thermal_scan'):
            if not self.recording:
                self.recording = True
                self.get_logger().info('🔴 Recording STARTED')
        elif self.camera_mode == 'standby':
            if self.recording:
                self.recording = False
                self.get_logger().info('⬛ Recording STOPPED')

        rec_msg = Bool()
        rec_msg.data = self.recording
        self.recording_active_pub.publish(rec_msg)

    def report_detection(self, detection_class, confidence, bearing_deg=0.0):
        """Log a geo-tagged detection."""
        if confidence < self.min_confidence:
            return

        now = time.time()
        if (now - self.last_detection_time) < self.detection_cooldown:
            return  # Cooldown active

        self.last_detection_time = now

        detection = {
            'timestamp': now,
            'elapsed_s': now - self.mission_start_time,
            'class': detection_class,
            'confidence': confidence,
            'position_ned': self.drone_pos.tolist(),
            'altitude_agl': float(self.drone_pos[2]),
            'bearing_deg': bearing_deg,
            'camera_mode': self.camera_mode,
            'session_id': self.session_id,
        }

        self.detections.append(detection)

        # Publish detection
        det_msg = String()
        det_msg.data = json.dumps(detection)
        self.detection_pub.publish(det_msg)

        # Alert
        if self.alert_enabled:
            alert = String()
            alert.data = (
                f'🎯 DETECTION: {detection_class} '
                f'(conf={confidence:.0%}) at '
                f'N={self.drone_pos[0]:.0f} E={self.drone_pos[1]:.0f} '
                f'Alt={self.drone_pos[2]:.0f}m'
            )
            self.alert_pub.publish(alert)
            self.get_logger().info(alert.data)

        # Save to disk
        self._save_detection(detection)

        return detection

    def _save_detection(self, detection):
        """Append detection to session log file."""
        log_file = os.path.join(
            self.recording_path,
            f'detections_{self.session_id}.jsonl'
        )
        try:
            with open(log_file, 'a') as f:
                f.write(json.dumps(detection) + '\n')
        except Exception as e:
            self.get_logger().error(f'Failed to save detection: {e}')

    def get_detection_summary(self):
        """Return summary of all detections this session."""
        class_counts = {}
        for d in self.detections:
            cls = d['class']
            class_counts[cls] = class_counts.get(cls, 0) + 1

        return {
            'session_id': self.session_id,
            'total_detections': len(self.detections),
            'by_class': class_counts,
            'recording_active': self.recording,
            'camera_mode': self.camera_mode,
        }

    def _publish_status(self):
        summary = self.get_detection_summary()
        status = String()
        status.data = (
            f'cam={self.camera_mode} '
            f'rec={"🔴" if self.recording else "⬛"} '
            f'detections={summary["total_detections"]} '
            f'{summary["by_class"]}'
        )
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SurveillanceManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save final summary
        summary = node.get_detection_summary()
        node.get_logger().info(f'Session summary: {json.dumps(summary)}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
