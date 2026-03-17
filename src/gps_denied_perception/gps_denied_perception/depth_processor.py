"""Depth processor node — converts depth images to filtered point clouds."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2


class DepthProcessor(Node):
    """Process depth images into filtered 3D point clouds.

    Subscribes to depth images, applies filtering (NaN removal,
    voxel downsampling, statistical outlier removal), and publishes
    clean point clouds for mapping.
    """

    def __init__(self):
        super().__init__('depth_processor')

        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('input_topic', '/sensors/camera/depth/image_raw')
        self.declare_parameter('output_topic', '/perception/pointcloud_filtered')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=1
        )

        self.depth_sub = self.create_subscription(
            Image, input_topic, self.depth_callback, sensor_qos
        )
        self.pc_pub = self.create_publisher(PointCloud2, output_topic, 10)

        self.get_logger().info('DepthProcessor initialized')

    def depth_callback(self, msg: Image):
        """Process depth frame to point cloud.

        TODO: Implement depth→cloud projection, filtering, publishing.
        """
        pass


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
