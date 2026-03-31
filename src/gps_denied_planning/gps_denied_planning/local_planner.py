"""Local planner — RRT* path planning with obstacle avoidance.

Plans collision-free paths from current position to goal using
the occupancy grid. Replans periodically to handle dynamic changes.
Outputs smooth trajectory for the control layer.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from std_msgs.msg import Bool
import numpy as np
import random
import time


class RRTNode:
    """Single node in the RRT tree."""
    __slots__ = ['pos', 'parent', 'cost']

    def __init__(self, pos, parent=None, cost=0.0):
        self.pos = np.array(pos)
        self.parent = parent
        self.cost = cost


class LocalPlanner(Node):
    """RRT*-based local planner with occupancy grid collision checking."""

    def __init__(self):
        super().__init__('local_planner')

        self.declare_parameter('replan_rate', 2.0)
        self.declare_parameter('max_velocity', 1.5)
        self.declare_parameter('safety_margin', 0.4)
        self.declare_parameter('rrt_max_iter', 500)
        self.declare_parameter('rrt_step_size', 0.5)
        self.declare_parameter('rrt_goal_bias', 0.2)
        self.declare_parameter('rrt_rewire_radius', 1.5)
        self.declare_parameter('path_smoothing_passes', 3)
        self.declare_parameter('goal_reached_tolerance', 0.3)

        self.max_vel = self.get_parameter('max_velocity').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.max_iter = self.get_parameter('rrt_max_iter').value
        self.step_size = self.get_parameter('rrt_step_size').value
        self.goal_bias = self.get_parameter('rrt_goal_bias').value
        self.rewire_radius = self.get_parameter('rrt_rewire_radius').value
        self.smoothing_passes = self.get_parameter('path_smoothing_passes').value
        self.goal_tolerance = self.get_parameter('goal_reached_tolerance').value

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/state_estimation/odom', self.odom_callback, reliable_qos
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/planning/current_goal', self.goal_callback, reliable_qos
        )
        self.grid_sub = self.create_subscription(
            OccupancyGrid, '/mapping/occupancy_grid', self.grid_callback, 10
        )

        # Publishers
        self.path_pub = self.create_publisher(Path, '/planning/local_path', 10)
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/control/position_setpoint', reliable_qos
        )

        # State
        self.current_pos = np.zeros(3)
        self.current_goal = None
        self.occupancy_grid = None
        self.grid_info = None
        self.current_path = []
        self.path_index = 0

        # Replan timer
        replan_rate = self.get_parameter('replan_rate').value
        self.replan_timer = self.create_timer(1.0 / replan_rate, self.replan)

        # Setpoint publishing at higher rate
        self.setpoint_timer = self.create_timer(0.1, self.publish_setpoint)

        self.get_logger().info('LocalPlanner (RRT*) initialized')

    def odom_callback(self, msg: Odometry):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])

    def goal_callback(self, msg: PoseStamped):
        new_goal = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        if self.current_goal is None or np.linalg.norm(new_goal - self.current_goal) > 0.3:
            self.current_goal = new_goal
            self.current_path = []
            self.path_index = 0
            self.get_logger().info(
                f'New goal: [{new_goal[0]:.1f}, {new_goal[1]:.1f}, {new_goal[2]:.1f}]'
            )

    def grid_callback(self, msg: OccupancyGrid):
        self.occupancy_grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )
        self.grid_info = msg.info

    def replan(self):
        """Periodically replan path from current position to goal."""
        if self.current_goal is None:
            return

        dist_to_goal = np.linalg.norm(self.current_pos[:2] - self.current_goal[:2])
        if dist_to_goal < self.goal_tolerance:
            return  # Already at goal

        # Run RRT*
        start_2d = self.current_pos[:2]
        goal_2d = self.current_goal[:2]

        path_2d = self._rrt_star(start_2d, goal_2d)

        if path_2d is not None and len(path_2d) > 0:
            # Smooth the path
            path_2d = self._smooth_path(path_2d)

            # Convert to 3D (maintain goal z height)
            z = self.current_goal[2]
            self.current_path = [np.array([p[0], p[1], z]) for p in path_2d]
            self.path_index = 0

            # Publish path for visualization
            self._publish_path()
        else:
            self.get_logger().warn('RRT* failed to find path — retrying next cycle')

    def _rrt_star(self, start, goal):
        """RRT* planner on 2D occupancy grid."""
        tree = [RRTNode(start)]

        for i in range(self.max_iter):
            # Sample random point (with goal bias)
            if random.random() < self.goal_bias:
                sample = goal.copy()
            else:
                # Sample in local map area
                sample = start + np.random.uniform(
                    -self.map_radius(), self.map_radius(), 2
                )

            # Find nearest node in tree
            nearest = min(tree, key=lambda n: np.linalg.norm(n.pos - sample))
            direction = sample - nearest.pos
            dist = np.linalg.norm(direction)
            if dist < 1e-6:
                continue

            # Steer toward sample
            direction = direction / dist
            step = min(dist, self.step_size)
            new_pos = nearest.pos + direction * step

            # Collision check
            if self._is_collision(nearest.pos, new_pos):
                continue

            # RRT* — find best parent in neighborhood
            new_cost = nearest.cost + step
            near_nodes = [
                n for n in tree
                if np.linalg.norm(n.pos - new_pos) < self.rewire_radius
            ]

            best_parent = nearest
            best_cost = new_cost
            for node in near_nodes:
                cost = node.cost + np.linalg.norm(node.pos - new_pos)
                if cost < best_cost and not self._is_collision(node.pos, new_pos):
                    best_parent = node
                    best_cost = cost

            new_node = RRTNode(new_pos, best_parent, best_cost)
            tree.append(new_node)

            # Rewire tree
            for node in near_nodes:
                rewire_cost = new_node.cost + np.linalg.norm(new_node.pos - node.pos)
                if rewire_cost < node.cost and not self._is_collision(new_node.pos, node.pos):
                    node.parent = new_node
                    node.cost = rewire_cost

            # Check if goal reached
            if np.linalg.norm(new_pos - goal) < self.goal_tolerance:
                # Trace back path
                path = [goal]
                node = new_node
                while node is not None:
                    path.append(node.pos)
                    node = node.parent
                path.reverse()
                return path

        return None  # Failed to find path

    def _is_collision(self, p1, p2):
        """Check if line segment p1→p2 collides with occupied cells."""
        if self.occupancy_grid is None or self.grid_info is None:
            return False  # No map yet, assume free

        # Check points along the line
        dist = np.linalg.norm(p2 - p1)
        n_checks = max(2, int(dist / (self.grid_info.resolution * 0.5)))

        for t in np.linspace(0, 1, n_checks):
            pt = p1 + t * (p2 - p1)
            gx = int((pt[0] - self.grid_info.origin.position.x) / self.grid_info.resolution)
            gy = int((pt[1] - self.grid_info.origin.position.y) / self.grid_info.resolution)

            if 0 <= gx < self.grid_info.width and 0 <= gy < self.grid_info.height:
                if self.occupancy_grid[gy, gx] > 50:  # Occupied
                    return True

        return False

    def _smooth_path(self, path):
        """Shortcut-based path smoothing."""
        if len(path) <= 2:
            return path

        for _ in range(self.smoothing_passes):
            i = 0
            while i < len(path) - 2:
                if not self._is_collision(path[i], path[i + 2]):
                    path.pop(i + 1)
                else:
                    i += 1
        return path

    def map_radius(self):
        if self.grid_info:
            return self.grid_info.width * self.grid_info.resolution / 2
        return 8.0

    def publish_setpoint(self):
        """Publish the next setpoint along the path at 10 Hz."""
        if not self.current_path or self.path_index >= len(self.current_path):
            if self.current_goal is not None:
                # No path — fly directly to goal
                sp = PoseStamped()
                sp.header.stamp = self.get_clock().now().to_msg()
                sp.header.frame_id = 'odom'
                sp.pose.position.x = float(self.current_goal[0])
                sp.pose.position.y = float(self.current_goal[1])
                sp.pose.position.z = float(self.current_goal[2])
                sp.pose.orientation.w = 1.0
                self.setpoint_pub.publish(sp)
            return

        target = self.current_path[self.path_index]

        # Advance path index if close to current waypoint
        dist = np.linalg.norm(self.current_pos - target)
        if dist < 0.5 and self.path_index < len(self.current_path) - 1:
            self.path_index += 1
            target = self.current_path[self.path_index]

        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'odom'
        sp.pose.position.x = float(target[0])
        sp.pose.position.y = float(target[1])
        sp.pose.position.z = float(target[2])
        sp.pose.orientation.w = 1.0
        self.setpoint_pub.publish(sp)

    def _publish_path(self):
        """Publish full path as nav_msgs/Path for RViz visualization."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for pt in self.current_path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = float(pt[2])
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
