"""Waypoint manager — sequences mission waypoints and detects arrival."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import yaml
import math


class WaypointManager(Node):
    """Manage mission waypoint sequences.

    Loads waypoints from YAML mission files, publishes the current goal,
    detects arrival, and advances through the sequence.
    """

    def __init__(self):
        super().__init__('waypoint_manager')

        self.declare_parameter('mission_file', '')
        self.declare_parameter('arrival_tolerance', 0.5)
        self.declare_parameter('heading_tolerance', 0.3)

        self.waypoints = []
        self.current_index = 0
        self.arrival_tolerance = self.get_parameter('arrival_tolerance').value

        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped, '/planning/current_goal', 10
        )

        # Subscribers
        state_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1)
        self.odom_sub = self.create_subscription(
            Odometry, '/state_estimation/odom', self.odom_callback, state_qos
        )

        # Load mission if provided
        mission_file = self.get_parameter('mission_file').value
        if mission_file:
            self.load_mission(mission_file)

        self.get_logger().info('WaypointManager initialized')

    def load_mission(self, filepath: str):
        """Load waypoints from a YAML mission file."""
        try:
            with open(filepath, 'r') as f:
                mission = yaml.safe_load(f)
            self.waypoints = mission.get('mission', {}).get('waypoints', [])
            self.current_index = 0
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} waypoints from {filepath}'
            )
            self.publish_current_goal()
        except Exception as e:
            self.get_logger().error(f'Failed to load mission: {e}')

    def publish_current_goal(self):
        """Publish the current waypoint as a goal pose."""
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('Mission complete — all waypoints reached')
            return
        wp = self.waypoints[self.current_index]
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'odom'
        goal.pose.position.x = float(wp['x'])
        goal.pose.position.y = float(wp['y'])
        goal.pose.position.z = float(wp['z'])
        self.goal_pub.publish(goal)

    def odom_callback(self, msg: Odometry):
        """Check if drone has arrived at current waypoint."""
        if self.current_index >= len(self.waypoints):
            return
        wp = self.waypoints[self.current_index]
        dx = msg.pose.pose.position.x - wp['x']
        dy = msg.pose.pose.position.y - wp['y']
        dz = msg.pose.pose.position.z - wp['z']
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        if dist < self.arrival_tolerance:
            self.get_logger().info(
                f'Arrived at waypoint {self.current_index + 1}/{len(self.waypoints)}'
            )
            self.current_index += 1
            self.publish_current_goal()


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
