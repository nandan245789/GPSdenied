"""Trajectory tracker — interpolates trajectories and publishes PX4 setpoints."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory


class TrajectoryTracker(Node):
    """Track planned trajectories by publishing PX4 offboard setpoints.

    Interpolates the current trajectory at the control rate (50 Hz)
    and publishes position/velocity setpoints to PX4.
    """

    def __init__(self):
        super().__init__('trajectory_tracker')

        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('offboard_heartbeat_rate', 10.0)
        self.declare_parameter('max_tracking_error', 1.0)

        control_rate = self.get_parameter('control_rate').value

        state_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1)

        self.odom_sub = self.create_subscription(
            Odometry, '/state_estimation/odom', self.odom_callback, state_qos
        )
        self.traj_sub = self.create_subscription(
            MultiDOFJointTrajectory, '/planning/trajectory',
            self.trajectory_callback, state_qos
        )

        self.control_timer = self.create_timer(
            1.0 / control_rate, self.control_loop
        )

        self.current_odom = None
        self.current_trajectory = None

        self.get_logger().info(f'TrajectoryTracker initialized @ {control_rate} Hz')

    def odom_callback(self, msg: Odometry):
        self.current_odom = msg

    def trajectory_callback(self, msg: MultiDOFJointTrajectory):
        self.current_trajectory = msg

    def control_loop(self):
        """50 Hz control loop.

        TODO: Interpolate trajectory at current time, compute setpoint,
        publish to PX4 /fmu/in/trajectory_setpoint, maintain offboard heartbeat.
        """
        if self.current_trajectory is None or self.current_odom is None:
            return
        # Stub — Phase 3 implementation
        pass


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
