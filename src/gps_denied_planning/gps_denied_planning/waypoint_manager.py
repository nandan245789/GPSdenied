"""Waypoint manager — loads YAML missions and sequences waypoint goals.

Publishes the current target waypoint, detects arrival, and advances
through the mission. Publishes mission progress for telemetry.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import numpy as np
import yaml
import os


class WaypointManager(Node):
    """Sequence through mission waypoints, detect arrival, advance goals."""

    def __init__(self):
        super().__init__('waypoint_manager')

        self.declare_parameter('mission_file', '/workspace/missions/warehouse_inspection.yaml')
        self.declare_parameter('arrival_tolerance', 0.5)
        self.declare_parameter('hold_time', 2.0)
        self.declare_parameter('check_rate', 10.0)

        self.arrival_tol = self.get_parameter('arrival_tolerance').value
        self.hold_time = self.get_parameter('hold_time').value

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/state_estimation/odom',
            self.odom_callback, reliable_qos
        )

        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped, '/planning/current_goal', reliable_qos
        )
        self.status_pub = self.create_publisher(String, '/planning/mission_status', 10)
        self.mission_complete_pub = self.create_publisher(Bool, '/planning/mission_complete', 10)

        # State
        self.waypoints = []
        self.current_wp_idx = 0
        self.drone_pos = np.zeros(3)
        self.arrived = False
        self.arrived_time = None
        self.mission_active = False

        # Load mission
        mission_file = self.get_parameter('mission_file').value
        self._load_mission(mission_file)

        # Check timer
        check_rate = self.get_parameter('check_rate').value
        self.check_timer = self.create_timer(1.0 / check_rate, self.check_progress)

        self.get_logger().info(
            f'WaypointManager: {len(self.waypoints)} waypoints loaded'
        )

    def _load_mission(self, filepath):
        """Load waypoints from YAML mission file."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)

            mission = data.get('mission', data)
            for wp in mission.get('waypoints', []):
                self.waypoints.append({
                    'id': wp.get('id', 0),
                    'x': float(wp['x']),
                    'y': float(wp['y']),
                    'z': float(wp['z']),
                    'yaw': float(wp.get('yaw', 0.0)),
                    'tolerance': float(wp.get('tolerance', self.arrival_tol)),
                    'hold_time': float(wp.get('hold_time', self.hold_time)),
                    'label': wp.get('label', f'WP{wp.get("id", 0)}'),
                })

            if self.waypoints:
                self.mission_active = True
                self._publish_current_goal()

        except Exception as e:
            self.get_logger().error(f'Failed to load mission: {e}')

    def odom_callback(self, msg: Odometry):
        self.drone_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])

    def check_progress(self):
        """Check if drone has arrived at current waypoint."""
        if not self.mission_active or self.current_wp_idx >= len(self.waypoints):
            return

        wp = self.waypoints[self.current_wp_idx]
        target = np.array([wp['x'], wp['y'], wp['z']])
        distance = np.linalg.norm(self.drone_pos - target)

        if distance < wp['tolerance']:
            if not self.arrived:
                self.arrived = True
                self.arrived_time = self.get_clock().now()
                self.get_logger().info(
                    f"✓ Arrived at {wp['label']} (d={distance:.2f}m) — "
                    f"holding for {wp['hold_time']}s"
                )

            # Check hold time
            elapsed = (self.get_clock().now() - self.arrived_time).nanoseconds / 1e9
            if elapsed >= wp['hold_time']:
                self._advance_waypoint()
        else:
            self.arrived = False
            self.arrived_time = None

        # Publish status
        status = String()
        status.data = (
            f"WP {self.current_wp_idx + 1}/{len(self.waypoints)} "
            f"[{wp['label']}] dist={distance:.2f}m"
        )
        self.status_pub.publish(status)

    def _advance_waypoint(self):
        """Move to next waypoint or complete mission."""
        self.current_wp_idx += 1
        self.arrived = False
        self.arrived_time = None

        if self.current_wp_idx >= len(self.waypoints):
            self.mission_active = False
            self.get_logger().info('🏁 MISSION COMPLETE — all waypoints reached')
            complete_msg = Bool()
            complete_msg.data = True
            self.mission_complete_pub.publish(complete_msg)
        else:
            wp = self.waypoints[self.current_wp_idx]
            self.get_logger().info(
                f"→ Navigating to {wp['label']} "
                f"[{wp['x']:.1f}, {wp['y']:.1f}, {wp['z']:.1f}]"
            )
            self._publish_current_goal()

    def _publish_current_goal(self):
        """Publish current waypoint as PoseStamped goal."""
        if self.current_wp_idx >= len(self.waypoints):
            return

        wp = self.waypoints[self.current_wp_idx]
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'odom'
        goal.pose.position.x = wp['x']
        goal.pose.position.y = wp['y']
        goal.pose.position.z = wp['z']

        # Yaw to quaternion (simplified — only yaw rotation)
        yaw = wp['yaw']
        goal.pose.orientation.z = float(np.sin(yaw / 2))
        goal.pose.orientation.w = float(np.cos(yaw / 2))

        self.goal_pub.publish(goal)

        # Republish periodically
        self.create_timer(1.0, lambda: self.goal_pub.publish(goal))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
