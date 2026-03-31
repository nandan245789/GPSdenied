"""Trajectory tracker — converts position setpoints to PX4 offboard commands.

Subscribes to position setpoints from the planner and publishes
PX4 trajectory setpoints at 50 Hz for smooth flight control.
Maintains the offboard heartbeat required by PX4.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    TrajectorySetpoint,
    OffboardControlMode,
    VehicleCommand,
    VehicleStatus,
)
import numpy as np


class TrajectoryTracker(Node):
    """Track position setpoints and publish PX4 offboard commands."""

    def __init__(self):
        super().__init__('trajectory_tracker')

        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('max_velocity', 1.5)
        self.declare_parameter('position_kp', 1.0)
        self.declare_parameter('max_tracking_error', 2.0)

        self.control_rate = self.get_parameter('control_rate').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.kp = self.get_parameter('position_kp').value
        self.max_error = self.get_parameter('max_tracking_error').value

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # Subscribers
        self.setpoint_sub = self.create_subscription(
            PoseStamped, '/control/position_setpoint',
            self.setpoint_callback, reliable_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/state_estimation/odom',
            self.odom_callback, reliable_qos
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, px4_qos
        )

        # Publishers
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos
        )

        # State
        self.current_pos = np.zeros(3)  # ENU
        self.target_pos = None          # ENU
        self.vehicle_status = None
        self.offboard_setpoint_count = 0

        # 50 Hz control loop
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self.control_loop
        )

        self.get_logger().info(
            f'TrajectoryTracker @ {self.control_rate} Hz, '
            f'max_vel={self.max_velocity} m/s'
        )

    def setpoint_callback(self, msg: PoseStamped):
        self.target_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

    def odom_callback(self, msg: Odometry):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def control_loop(self):
        """50 Hz control — offboard heartbeat + trajectory setpoint."""
        # Always publish offboard heartbeat
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )
        self.offboard_pub.publish(offboard_msg)

        self.offboard_setpoint_count += 1

        # After enough heartbeats, arm and switch to offboard
        if self.offboard_setpoint_count == 50:
            self._engage_offboard()
            self._arm()

        # Compute and publish trajectory setpoint
        if self.target_pos is None:
            # No target — hold current position
            self._publish_setpoint(self.current_pos)
            return

        # P-controller: velocity command toward target
        error = self.target_pos - self.current_pos
        distance = np.linalg.norm(error)

        if distance < 0.1:
            # At target — publish position hold
            self._publish_setpoint(self.target_pos)
        else:
            # Velocity limited proportional control
            direction = error / distance
            speed = min(self.kp * distance, self.max_velocity)
            velocity = direction * speed

            self._publish_setpoint(self.target_pos, velocity)

    def _publish_setpoint(self, position, velocity=None):
        """Publish PX4 trajectory setpoint in NED frame."""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # ENU → NED conversion
        msg.position = [
            float(position[1]),    # N = ENU.y
            float(position[0]),    # E = ENU.x
            float(-position[2]),   # D = -ENU.z
        ]

        if velocity is not None:
            msg.velocity = [
                float(velocity[1]),    # N
                float(velocity[0]),    # E
                float(-velocity[2]),   # D
            ]
        else:
            msg.velocity = [0.0, 0.0, 0.0]

        msg.yaw = float('nan')  # Let PX4 handle yaw
        self.trajectory_pub.publish(msg)

    def _engage_offboard(self):
        """Request PX4 to enter OFFBOARD mode."""
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0            # Custom mode
        cmd.param2 = 6.0            # OFFBOARD
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(cmd)
        self.get_logger().info('OFFBOARD mode requested')

    def _arm(self):
        """Send arm command to PX4."""
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0            # Arm
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(cmd)
        self.get_logger().info('ARM command sent')


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


if __name__ == '__main__':
    main()
