"""Local planner — RRT* path search + trajectory smoothing."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from trajectory_msgs.msg import MultiDOFJointTrajectory


class LocalPlanner(Node):
    """Compute collision-free trajectories from current pose to goal.

    Uses RRT* for path search through the occupancy map, then applies
    minimum-snap smoothing for dynamically feasible trajectories.
    """

    def __init__(self):
        super().__init__('local_planner')

        self.declare_parameter('replan_rate', 5.0)
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('max_acceleration', 1.5)
        self.declare_parameter('safety_margin', 0.4)
        self.declare_parameter('planning_horizon', 5.0)
        self.declare_parameter('max_planning_time', 0.15)

        state_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/state_estimation/odom', self.odom_callback, state_qos
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/planning/current_goal', self.goal_callback, state_qos
        )

        # Publishers
        self.path_pub = self.create_publisher(Path, '/planning/local_path', 10)
        self.traj_pub = self.create_publisher(
            MultiDOFJointTrajectory, '/planning/trajectory', 10
        )

        # Replan timer
        replan_rate = self.get_parameter('replan_rate').value
        self.replan_timer = self.create_timer(
            1.0 / replan_rate, self.replan_callback
        )

        self.current_odom = None
        self.current_goal = None

        self.get_logger().info('LocalPlanner initialized')

    def odom_callback(self, msg: Odometry):
        self.current_odom = msg

    def goal_callback(self, msg: PoseStamped):
        self.current_goal = msg
        self.get_logger().info(
            f'New goal: ({msg.pose.position.x:.1f}, '
            f'{msg.pose.position.y:.1f}, {msg.pose.position.z:.1f})'
        )

    def replan_callback(self):
        """Periodic replanning cycle.

        TODO: Implement RRT* search, collision checking against OctoMap,
        minimum-snap smoothing, and velocity profiling.
        """
        if self.current_odom is None or self.current_goal is None:
            return
        # Stub — Phase 3 implementation
        pass


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
