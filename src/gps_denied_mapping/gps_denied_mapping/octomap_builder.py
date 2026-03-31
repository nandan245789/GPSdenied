"""OctoMap builder — incremental 3D occupancy grid from point clouds.

Uses a numpy-based voxel grid (no C++ OctoMap dependency).
Maintains a rolling local map centered on the drone's position.
Publishes occupancy grid for the planner.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import struct
import threading


class OctomapBuilder(Node):
    """Build and maintain a local 3D voxel occupancy grid."""

    def __init__(self):
        super().__init__('octomap_builder')

        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('local_map_radius', 8.0)
        self.declare_parameter('obstacle_inflation', 0.3)
        self.declare_parameter('occupancy_threshold', 0.65)
        self.declare_parameter('free_threshold', 0.35)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('map_height_slice', 1.5)

        self.resolution = self.get_parameter('resolution').value
        self.map_radius = self.get_parameter('local_map_radius').value
        self.inflation = self.get_parameter('obstacle_inflation').value
        self.occ_thresh = self.get_parameter('occupancy_threshold').value
        self.free_thresh = self.get_parameter('free_threshold').value
        self.height_slice = self.get_parameter('map_height_slice').value

        # Voxel grid as a dict: (ix, iy, iz) → log-odds probability
        self.voxel_grid = {}
        self.lock = threading.Lock()

        # Log-odds update values
        self.l_occ = 0.85   # Log-odds for occupied
        self.l_free = -0.4  # Log-odds for free
        self.l_max = 3.5
        self.l_min = -2.0

        # Drone position
        self.drone_pos = np.zeros(3)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # Subscribers
        self.pc_sub = self.create_subscription(
            PointCloud2, '/perception/pointcloud_filtered',
            self.cloud_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/state_estimation/pose',
            self.pose_callback, reliable_qos
        )

        # Publishers
        self.grid_pub = self.create_publisher(
            OccupancyGrid, '/mapping/occupancy_grid', 10
        )

        # Periodic publish
        pub_rate = self.get_parameter('publish_rate').value
        self.pub_timer = self.create_timer(1.0 / pub_rate, self.publish_grid)

        self.update_count = 0
        self.get_logger().info(
            f'OctomapBuilder: res={self.resolution}m, '
            f'radius={self.map_radius}m, inflation={self.inflation}m'
        )

    def pose_callback(self, msg: PoseStamped):
        self.drone_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

    def cloud_callback(self, msg: PointCloud2):
        """Insert point cloud into voxel grid."""
        points = self._pointcloud2_to_numpy(msg)
        if points is None or len(points) == 0:
            return

        # Transform points to odom frame (assuming TF is handled)
        # For now, offset by drone position (simplified)
        points_world = points + self.drone_pos

        with self.lock:
            for pt in points_world:
                ix = int(np.floor(pt[0] / self.resolution))
                iy = int(np.floor(pt[1] / self.resolution))
                iz = int(np.floor(pt[2] / self.resolution))
                key = (ix, iy, iz)

                # Update log-odds
                current = self.voxel_grid.get(key, 0.0)
                updated = np.clip(current + self.l_occ, self.l_min, self.l_max)
                self.voxel_grid[key] = updated

            # Apply inflation — mark neighbors as semi-occupied
            inflation_cells = max(1, int(self.inflation / self.resolution))
            inflated_keys = []
            for pt in points_world[::10]:  # Subsample for performance
                ix = int(np.floor(pt[0] / self.resolution))
                iy = int(np.floor(pt[1] / self.resolution))
                iz = int(np.floor(pt[2] / self.resolution))
                for dx in range(-inflation_cells, inflation_cells + 1):
                    for dy in range(-inflation_cells, inflation_cells + 1):
                        nkey = (ix + dx, iy + dy, iz)
                        if nkey not in self.voxel_grid:
                            inflated_keys.append(nkey)

            for key in inflated_keys:
                current = self.voxel_grid.get(key, 0.0)
                self.voxel_grid[key] = np.clip(
                    current + self.l_occ * 0.3, self.l_min, self.l_max
                )

            # Prune voxels outside local map radius
            self._prune_distant_voxels()

        self.update_count += 1
        if self.update_count % 30 == 0:
            self.get_logger().info(
                f'Map: {len(self.voxel_grid)} voxels | '
                f'drone=[{self.drone_pos[0]:.1f}, {self.drone_pos[1]:.1f}]'
            )

    def _prune_distant_voxels(self):
        """Remove voxels outside the rolling local map window."""
        max_cells = int(self.map_radius / self.resolution)
        drone_ix = int(np.floor(self.drone_pos[0] / self.resolution))
        drone_iy = int(np.floor(self.drone_pos[1] / self.resolution))

        to_remove = []
        for key in self.voxel_grid:
            if abs(key[0] - drone_ix) > max_cells or abs(key[1] - drone_iy) > max_cells:
                to_remove.append(key)
        for key in to_remove:
            del self.voxel_grid[key]

    def publish_grid(self):
        """Publish 2D occupancy grid (height slice) for planner."""
        with self.lock:
            if len(self.voxel_grid) == 0:
                return

            # Extract 2D slice at drone height
            iz_slice = int(np.floor(self.height_slice / self.resolution))
            iz_range = range(iz_slice - 2, iz_slice + 3)  # ±2 cells

            # Find bounds
            grid_size = int(2 * self.map_radius / self.resolution)
            origin_x = self.drone_pos[0] - self.map_radius
            origin_y = self.drone_pos[1] - self.map_radius
            origin_ix = int(np.floor(origin_x / self.resolution))
            origin_iy = int(np.floor(origin_y / self.resolution))

            # Build 2D grid
            grid_data = np.full(grid_size * grid_size, -1, dtype=np.int8)  # Unknown

            for key, logodds in self.voxel_grid.items():
                if key[2] not in iz_range:
                    continue
                gx = key[0] - origin_ix
                gy = key[1] - origin_iy
                if 0 <= gx < grid_size and 0 <= gy < grid_size:
                    prob = 1.0 / (1.0 + np.exp(-logodds))  # Log-odds → probability
                    idx = gy * grid_size + gx
                    if prob > self.occ_thresh:
                        grid_data[idx] = 100  # Occupied
                    elif prob < self.free_thresh:
                        grid_data[idx] = 0    # Free
                    else:
                        grid_data[idx] = 50   # Unknown

        # Publish
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = float(self.resolution)
        msg.info.width = grid_size
        msg.info.height = grid_size
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = self.height_slice
        msg.data = grid_data.tolist()
        self.grid_pub.publish(msg)

    def _pointcloud2_to_numpy(self, msg):
        """Parse PointCloud2 to Nx3 numpy array."""
        if msg.width == 0:
            return None
        data = np.frombuffer(msg.data, dtype=np.float32)
        if len(data) < msg.width * 3:
            return None
        return data.reshape(-1, 3)[:msg.width]

    def is_occupied(self, x, y, z):
        """Query if a world position is occupied (for planner)."""
        ix = int(np.floor(x / self.resolution))
        iy = int(np.floor(y / self.resolution))
        iz = int(np.floor(z / self.resolution))
        logodds = self.voxel_grid.get((ix, iy, iz), 0.0)
        prob = 1.0 / (1.0 + np.exp(-logodds))
        return prob > self.occ_thresh


def main(args=None):
    rclpy.init(args=args)
    node = OctomapBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
