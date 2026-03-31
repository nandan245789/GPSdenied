"""Depth processor — converts depth images to filtered point clouds.

Subscribes to depth image from D435i, filters by range, downsamples
via voxel grid, and publishes as PointCloud2 for the mapper.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import struct


class DepthProcessor(Node):
    """Process depth images into filtered point clouds."""

    def __init__(self):
        super().__init__('depth_processor')

        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('downsample_factor', 2)

        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.downsample = self.get_parameter('downsample_factor').value

        # Camera intrinsics (D435i depth defaults)
        self.fx = 386.0
        self.fy = 386.0
        self.cx = 320.0
        self.cy = 240.0

        self.bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        self.depth_sub = self.create_subscription(
            Image, '/sensors/camera/depth/image_raw',
            self.depth_callback, sensor_qos
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/perception/pointcloud_filtered', 10
        )

        self.frame_count = 0
        self.get_logger().info('DepthProcessor initialized')

    def depth_callback(self, msg: Image):
        """Convert depth image to filtered point cloud."""
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')
            return

        depth = depth.astype(np.float32)
        if depth.max() > 100.0:  # If in millimeters, convert to meters
            depth = depth / 1000.0

        # Downsample for performance
        depth = depth[::self.downsample, ::self.downsample]
        h, w = depth.shape

        # Build pixel coordinate grid
        u = np.arange(0, w) * self.downsample
        v = np.arange(0, h) * self.downsample
        u, v = np.meshgrid(u, v)

        # Back-project to 3D
        z = depth
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        # Stack and filter by range
        points = np.stack([x, y, z], axis=-1).reshape(-1, 3)
        distances = np.linalg.norm(points, axis=1)
        mask = (distances > self.min_range) & (distances < self.max_range)
        mask &= np.isfinite(distances)
        points = points[mask]

        if len(points) == 0:
            return

        # Voxel grid downsampling
        if self.voxel_size > 0:
            points = self._voxel_downsample(points, self.voxel_size)

        # Publish as PointCloud2
        cloud_msg = self._create_pointcloud2(points, msg.header)
        self.cloud_pub.publish(cloud_msg)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Depth: {len(points)} points published')

    def _voxel_downsample(self, points, voxel_size):
        """Simple voxel grid downsampling."""
        quantized = np.floor(points / voxel_size).astype(np.int32)
        _, unique_idx = np.unique(
            quantized, axis=0, return_index=True
        )
        return points[unique_idx]

    def _create_pointcloud2(self, points, header):
        """Create PointCloud2 message from Nx3 numpy array."""
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = 'camera_depth_optical_frame'
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12  # 3 * float32
        msg.row_step = msg.point_step * msg.width
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = points.astype(np.float32).tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
