"""Waypoint manager — border surveillance mission sequencer.

Handles long-range outdoor navigation for cross-border operations:
- NED frame waypoints (north/east/alt_agl)
- Surveillance actions: transit, sweep, loiter_scan, rtl
- Loiter orbit patterns for area surveillance
- Camera mode switching per waypoint
- Emergency landing site management
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Float32
import numpy as np
import yaml
import math
import time


class WaypointManager(Node):
    """Sequence through patrol waypoints with surveillance actions."""

    def __init__(self):
        super().__init__('waypoint_manager')

        self.declare_parameter('mission_file', '/workspace/missions/border_patrol.yaml')
        self.declare_parameter('arrival_tolerance', 15.0)          # 15m for outdoor
        self.declare_parameter('hold_time', 0.0)
        self.declare_parameter('check_rate', 5.0)                  # 5 Hz check rate
        self.declare_parameter('default_cruise_speed', 8.0)
        self.declare_parameter('loiter_radius', 50.0)

        self.arrival_tol = self.get_parameter('arrival_tolerance').value
        self.hold_time = self.get_parameter('hold_time').value
        self.cruise_speed = self.get_parameter('default_cruise_speed').value
        self.loiter_radius = self.get_parameter('loiter_radius').value

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
        self.speed_pub = self.create_publisher(Float32, '/planning/target_speed', 10)
        self.camera_cmd_pub = self.create_publisher(String, '/surveillance/camera_command', 10)
        self.action_pub = self.create_publisher(String, '/planning/current_action', 10)

        # State
        self.waypoints = []
        self.emergency_sites = []
        self.current_wp_idx = 0
        self.drone_pos = np.zeros(3)       # NED: [north, east, down]
        self.arrived = False
        self.arrived_time = None
        self.mission_active = False
        self.mission_name = ""
        self.total_distance = 0.0
        self.prev_pos = np.zeros(3)
        self.mission_start_time = None

        # Loiter state
        self.loiter_active = False
        self.loiter_center = np.zeros(3)
        self.loiter_angle = 0.0
        self.loiter_orbits_done = 0
        self.loiter_orbits_target = 0

        # Load mission
        mission_file = self.get_parameter('mission_file').value
        self._load_mission(mission_file)

        # Timers
        check_rate = self.get_parameter('check_rate').value
        self.check_timer = self.create_timer(1.0 / check_rate, self.check_progress)
        self.goal_republish_timer = self.create_timer(0.5, self._republish_goal)

        self.get_logger().info(
            f'WaypointManager [{self.mission_name}]: '
            f'{len(self.waypoints)} waypoints, '
            f'{len(self.emergency_sites)} emergency sites loaded'
        )

    def _load_mission(self, filepath):
        """Load border patrol mission from YAML."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)

            mission = data.get('mission', data)
            self.mission_name = mission.get('name', 'unnamed')

            for wp in mission.get('waypoints', []):
                self.waypoints.append({
                    'id': wp.get('id', 0),
                    'north': float(wp.get('north', wp.get('x', 0.0))),
                    'east': float(wp.get('east', wp.get('y', 0.0))),
                    'alt_agl': float(wp.get('alt_agl', wp.get('z', 60.0))),
                    'speed': float(wp.get('speed', self.cruise_speed)),
                    'tolerance': float(wp.get('tolerance', self.arrival_tol)),
                    'hold_time': float(wp.get('hold_time', 0.0)),
                    'action': wp.get('action', 'transit'),
                    'camera_mode': wp.get('camera_mode', 'standby'),
                    'loiter_orbits': int(wp.get('loiter_orbits', 1)),
                    'label': wp.get('label', f'WP{wp.get("id", 0)}'),
                })

            # Load emergency landing sites
            for els in mission.get('emergency_landing_sites', []):
                self.emergency_sites.append({
                    'id': els.get('id', 'unknown'),
                    'north': float(els.get('north', 0)),
                    'east': float(els.get('east', 0)),
                    'description': els.get('description', ''),
                })

            if self.waypoints:
                self.mission_active = True
                self.mission_start_time = time.time()
                self._publish_current_goal()
                self._set_camera_mode()

        except Exception as e:
            self.get_logger().error(f'Failed to load mission: {e}')

    def odom_callback(self, msg: Odometry):
        new_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        self.total_distance += np.linalg.norm(new_pos - self.prev_pos)
        self.prev_pos = new_pos.copy()
        self.drone_pos = new_pos

    def check_progress(self):
        """Check arrival, handle loiter, advance waypoints."""
        if not self.mission_active or self.current_wp_idx >= len(self.waypoints):
            return

        wp = self.waypoints[self.current_wp_idx]
        target = np.array([wp['north'], wp['east'], wp['alt_agl']])
        h_dist = np.linalg.norm(self.drone_pos[:2] - target[:2])  # Horizontal distance
        v_dist = abs(self.drone_pos[2] - target[2])

        # Loiter mode — orbit around waypoint
        if self.loiter_active:
            self._update_loiter()
            self._publish_status(wp, h_dist)
            return

        # Check arrival (horizontal + vertical)
        if h_dist < wp['tolerance'] and v_dist < wp['tolerance'] * 0.5:
            if not self.arrived:
                self.arrived = True
                self.arrived_time = time.time()

                elapsed = time.time() - self.mission_start_time if self.mission_start_time else 0
                self.get_logger().info(
                    f"✓ [{elapsed:.0f}s] Arrived at {wp['label']} "
                    f"(d={h_dist:.0f}m, alt={self.drone_pos[2]:.0f}m)"
                )

                # Start loiter if waypoint action requires it
                if wp['action'] == 'loiter_scan':
                    self._start_loiter(wp)
                    return

            # Check hold time
            if (time.time() - self.arrived_time) >= wp['hold_time']:
                self._advance_waypoint()
        else:
            self.arrived = False
            self.arrived_time = None

        # Publish speed target
        speed_msg = Float32()
        speed_msg.data = float(wp['speed'])
        self.speed_pub.publish(speed_msg)

        self._publish_status(wp, h_dist)

    def _start_loiter(self, wp):
        """Begin orbital loiter around current waypoint."""
        self.loiter_active = True
        self.loiter_center = np.array([wp['north'], wp['east'], wp['alt_agl']])
        self.loiter_angle = 0.0
        self.loiter_orbits_done = 0
        self.loiter_orbits_target = wp['loiter_orbits']
        self.get_logger().info(
            f"🔄 Loitering at {wp['label']} — "
            f"{self.loiter_orbits_target} orbits, r={self.loiter_radius:.0f}m"
        )

    def _update_loiter(self):
        """Advance loiter orbit and publish tangent goal."""
        # Calculate angle step (aim for ~5m/s around the orbit)
        orbit_circumference = 2 * math.pi * self.loiter_radius
        steps_per_orbit = orbit_circumference / 5.0  # 5m steps
        angle_step = (2 * math.pi) / steps_per_orbit * 0.2  # at 5 Hz timer

        self.loiter_angle += angle_step

        if self.loiter_angle >= 2 * math.pi:
            self.loiter_angle -= 2 * math.pi
            self.loiter_orbits_done += 1
            self.get_logger().info(
                f"  🔄 Orbit {self.loiter_orbits_done}/{self.loiter_orbits_target} complete"
            )

        if self.loiter_orbits_done >= self.loiter_orbits_target:
            self.loiter_active = False
            wp = self.waypoints[self.current_wp_idx]
            self.arrived_time = time.time() - wp['hold_time']  # Force advance
            self._advance_waypoint()
            return

        # Publish orbit setpoint
        orbit_north = self.loiter_center[0] + self.loiter_radius * math.cos(self.loiter_angle)
        orbit_east = self.loiter_center[1] + self.loiter_radius * math.sin(self.loiter_angle)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'local_ned'
        goal.pose.position.x = float(orbit_north)
        goal.pose.position.y = float(orbit_east)
        goal.pose.position.z = float(self.loiter_center[2])

        # Face center of orbit (inward-pointing yaw)
        yaw = math.atan2(
            self.loiter_center[1] - orbit_east,
            self.loiter_center[0] - orbit_north
        )
        goal.pose.orientation.z = float(math.sin(yaw / 2))
        goal.pose.orientation.w = float(math.cos(yaw / 2))

        self.goal_pub.publish(goal)

    def _advance_waypoint(self):
        """Move to next waypoint or complete mission."""
        self.current_wp_idx += 1
        self.arrived = False
        self.arrived_time = None
        self.loiter_active = False

        if self.current_wp_idx >= len(self.waypoints):
            elapsed = time.time() - self.mission_start_time if self.mission_start_time else 0
            self.mission_active = False
            self.get_logger().info(
                f'🏁 MISSION COMPLETE — {len(self.waypoints)} waypoints, '
                f'{self.total_distance:.0f}m traveled, {elapsed:.0f}s elapsed'
            )
            complete_msg = Bool()
            complete_msg.data = True
            self.mission_complete_pub.publish(complete_msg)
        else:
            wp = self.waypoints[self.current_wp_idx]
            dist_to_next = np.linalg.norm(
                self.drone_pos[:2] - np.array([wp['north'], wp['east']])
            )
            eta = dist_to_next / max(wp['speed'], 0.1)
            self.get_logger().info(
                f"→ WP{wp['id']} {wp['label']} [{wp['action']}] "
                f"dist={dist_to_next:.0f}m ETA={eta:.0f}s"
            )
            self._publish_current_goal()
            self._set_camera_mode()

    def _publish_current_goal(self):
        """Publish current waypoint as PoseStamped."""
        if self.current_wp_idx >= len(self.waypoints):
            return

        wp = self.waypoints[self.current_wp_idx]
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'local_ned'
        goal.pose.position.x = float(wp['north'])
        goal.pose.position.y = float(wp['east'])
        goal.pose.position.z = float(wp['alt_agl'])
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)

    def _republish_goal(self):
        """Republish goal at 2 Hz for subscriber reliability."""
        if not self.loiter_active:
            self._publish_current_goal()

    def _set_camera_mode(self):
        """Switch camera mode for current waypoint."""
        if self.current_wp_idx >= len(self.waypoints):
            return
        wp = self.waypoints[self.current_wp_idx]
        cam_cmd = String()
        cam_cmd.data = wp['camera_mode']
        self.camera_cmd_pub.publish(cam_cmd)

        action = String()
        action.data = wp['action']
        self.action_pub.publish(action)

    def get_nearest_emergency_site(self):
        """Find closest emergency landing site to current position."""
        if not self.emergency_sites:
            return {'north': 0, 'east': 0, 'description': 'Launch point'}

        min_dist = float('inf')
        nearest = self.emergency_sites[0]
        for site in self.emergency_sites:
            d = np.linalg.norm(
                self.drone_pos[:2] - np.array([site['north'], site['east']])
            )
            if d < min_dist:
                min_dist = d
                nearest = site
        return nearest

    def _publish_status(self, wp, distance):
        """Publish mission progress."""
        elapsed = time.time() - self.mission_start_time if self.mission_start_time else 0
        status = String()
        action = f'[{wp["action"]}]' if wp['action'] != 'transit' else ''
        status.data = (
            f"WP {self.current_wp_idx + 1}/{len(self.waypoints)} "
            f"{wp['label']} {action} "
            f"dist={distance:.0f}m alt={self.drone_pos[2]:.0f}m "
            f"total={self.total_distance:.0f}m t={elapsed:.0f}s"
        )
        self.status_pub.publish(status)


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
