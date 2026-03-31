"""Standalone simulation demo — runs the full autonomy pipeline.

Simulates sensor data internally so no Gazebo/PX4 needed.
Demonstrates: VIO → Mapping → Planning → Control → Safety

Usage (inside Docker):
  cd /workspace && colcon build --symlink-install
  source install/setup.bash
  python3 scripts/run_demo.py
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String, Bool

import numpy as np
import time
import math
import sys
import threading


class SimulatedDrone(Node):
    """Simulates drone physics — moves toward setpoints with simple dynamics."""

    def __init__(self):
        super().__init__('simulated_drone')

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # Subscribe to control setpoint
        self.setpoint_sub = self.create_subscription(
            PoseStamped, '/control/position_setpoint',
            self.setpoint_callback, reliable_qos
        )

        # Publish simulated sensor data
        self.odom_pub = self.create_publisher(Odometry, '/state_estimation/odom', reliable_qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/state_estimation/pose', reliable_qos)
        self.confidence_pub = self.create_publisher(Float32, '/state_estimation/confidence', 10)
        self.gt_pose_pub = self.create_publisher(PoseStamped, '/model/x500/pose', 10)

        # Simulated occupancy grid (static obstacles)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/mapping/occupancy_grid', 10)

        # Drone state
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.zeros(3)
        self.setpoint = np.array([0.0, 0.0, 1.5])  # Initial hover
        self.max_speed = 1.5
        self.dt = 0.02  # 50 Hz

        # Simulated obstacles (shelf positions from warehouse world)
        self.obstacles = [
            {'center': [3.0, 0.0], 'size': [0.5, 6.0]},   # shelf_1
            {'center': [-3.0, 0.0], 'size': [0.5, 6.0]},   # shelf_2
        ]

        # Timers
        self.physics_timer = self.create_timer(self.dt, self.physics_step)
        self.grid_timer = self.create_timer(1.0, self.publish_obstacle_grid)

        self.step_count = 0
        self.get_logger().info('SimulatedDrone initialized at origin')

    def setpoint_callback(self, msg: PoseStamped):
        self.setpoint = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

    def physics_step(self):
        """Simple drone dynamics — P controller toward setpoint."""
        error = self.setpoint - self.position
        distance = np.linalg.norm(error)

        if distance > 0.05:
            direction = error / distance
            speed = min(1.0 * distance, self.max_speed)
            desired_vel = direction * speed
        else:
            desired_vel = np.zeros(3)

        # Simple first-order dynamics
        tau = 0.3  # Time constant
        self.velocity += (desired_vel - self.velocity) * (self.dt / tau)
        self.position += self.velocity * self.dt

        # Add small noise (simulates VIO uncertainty)
        noisy_pos = self.position + np.random.normal(0, 0.005, 3)

        # Publish odometry
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(noisy_pos[0])
        odom.pose.pose.position.y = float(noisy_pos[1])
        odom.pose.pose.position.z = float(noisy_pos[2])
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = float(self.velocity[0])
        odom.twist.twist.linear.y = float(self.velocity[1])
        odom.twist.twist.linear.z = float(self.velocity[2])
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 0.01
        self.odom_pub.publish(odom)

        # Publish pose
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        # Publish confidence (high when features would be visible)
        conf = Float32()
        conf.data = 0.85 + 0.1 * np.random.random()  # 0.85-0.95
        self.confidence_pub.publish(conf)

        # Ground truth (perfect position)
        gt = PoseStamped()
        gt.header.stamp = stamp
        gt.header.frame_id = 'odom'
        gt.pose.position.x = float(self.position[0])
        gt.pose.position.y = float(self.position[1])
        gt.pose.position.z = float(self.position[2])
        gt.pose.orientation.w = 1.0
        self.gt_pose_pub.publish(gt)

        self.step_count += 1
        if self.step_count % 50 == 0:  # Every 1 second
            d = np.linalg.norm(self.position - self.setpoint)
            self.get_logger().info(
                f'🛸 pos=[{self.position[0]:6.2f}, {self.position[1]:6.2f}, '
                f'{self.position[2]:6.2f}] vel={np.linalg.norm(self.velocity):.2f} '
                f'm/s → target dist={d:.2f}m'
            )

    def publish_obstacle_grid(self):
        """Publish static 2D occupancy grid with obstacles."""
        resolution = 0.1
        size = 160  # 16m x 16m
        origin_x = -8.0
        origin_y = -8.0

        grid_data = np.zeros(size * size, dtype=np.int8)  # All free

        for obs in self.obstacles:
            cx, cy = obs['center']
            sx, sy = obs['size']
            x_min = int((cx - sx / 2 - origin_x) / resolution)
            x_max = int((cx + sx / 2 - origin_x) / resolution)
            y_min = int((cy - sy / 2 - origin_y) / resolution)
            y_max = int((cy + sy / 2 - origin_y) / resolution)

            for y in range(max(0, y_min), min(size, y_max)):
                for x in range(max(0, x_min), min(size, x_max)):
                    grid_data[y * size + x] = 100  # Occupied

            # Inflation (0.3m)
            inf_cells = 3
            for y in range(max(0, y_min - inf_cells), min(size, y_max + inf_cells)):
                for x in range(max(0, x_min - inf_cells), min(size, x_max + inf_cells)):
                    if grid_data[y * size + x] == 0:
                        grid_data[y * size + x] = 50  # Inflated

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = resolution
        msg.info.width = size
        msg.info.height = size
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.data = grid_data.tolist()
        self.grid_pub.publish(msg)


class MissionMonitor(Node):
    """Monitors and prints mission progress to terminal."""

    def __init__(self):
        super().__init__('mission_monitor')

        self.create_subscription(
            String, '/planning/mission_status', self.status_cb, 10
        )
        self.create_subscription(
            Bool, '/planning/mission_complete', self.complete_cb, 10
        )
        self.create_subscription(
            String, '/safety/system_status', self.safety_cb, 10
        )
        self.create_subscription(
            String, '/telemetry/gt_stats', self.gt_cb, 10
        )

        self.mission_done = False
        self.start_time = time.time()

    def status_cb(self, msg: String):
        elapsed = time.time() - self.start_time
        self.get_logger().info(f'[{elapsed:5.1f}s] 📍 {msg.data}')

    def complete_cb(self, msg: Bool):
        if msg.data:
            elapsed = time.time() - self.start_time
            self.get_logger().info(
                f'\n{"=" * 50}\n'
                f'  🏁  MISSION COMPLETE in {elapsed:.1f}s\n'
                f'{"=" * 50}\n'
            )
            self.mission_done = True

    def safety_cb(self, msg: String):
        pass  # Quiet — logged by supervisor

    def gt_cb(self, msg: String):
        self.get_logger().info(f'📊 GT: {msg.data}')


def main():
    rclpy.init()
    print()
    print('=' * 60)
    print('  🚀  GPS-Denied Navigation — Standalone Simulation Demo')
    print('=' * 60)
    print()
    print('  Simulating: Camera + IMU → VIO → Map → RRT* → Control')
    print('  Mission: 3-waypoint warehouse inspection')
    print('  Obstacles: 2 shelf units at x=±3m')
    print()

    # Create all nodes
    sim_drone = SimulatedDrone()
    monitor = MissionMonitor()

    # Import and create real pipeline nodes
    from gps_denied_planning.waypoint_manager import WaypointManager
    from gps_denied_planning.local_planner import LocalPlanner
    from gps_denied_safety.mission_supervisor import MissionSupervisor
    from gps_denied_state_estimation.ground_truth_comparator import GroundTruthComparator

    wp_manager = WaypointManager()
    planner = LocalPlanner()
    supervisor = MissionSupervisor()
    gt_comparator = GroundTruthComparator()

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(sim_drone)
    executor.add_node(monitor)
    executor.add_node(wp_manager)
    executor.add_node(planner)
    executor.add_node(supervisor)
    executor.add_node(gt_comparator)

    print('  All nodes launched — mission starting...\n')

    try:
        start = time.time()
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
            elapsed = time.time() - start

            if monitor.mission_done:
                # Wait a few seconds then exit
                time.sleep(3)
                break

            if elapsed > 120:  # 2 minute timeout
                print('\n⚠️  Mission timeout (120s) — stopping')
                break

    except KeyboardInterrupt:
        print('\n\nMission interrupted by user')
    finally:
        executor.shutdown()
        sim_drone.destroy_node()
        monitor.destroy_node()
        wp_manager.destroy_node()
        planner.destroy_node()
        supervisor.destroy_node()
        gt_comparator.destroy_node()
        rclpy.shutdown()
        print('\nSimulation ended.\n')


if __name__ == '__main__':
    main()
