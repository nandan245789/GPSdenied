"""OctoMap builder — incremental 3D occupancy mapping from point clouds."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped


class OctomapBuilder(Node):
    """Build a rolling local OctoMap from filtered point clouds + VIO pose.

    TODO: Integrate octomap library, insert clouds, maintain rolling window,
    publish octomap + 2.5D occupancy grid + obstacle markers.
    """

    def __init__(self):
        super().__init__('octomap_builder')
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('local_map_radius', 8.0)
        self.declare_parameter('insertion_max_range', 5.0)
        self.declare_parameter('obstacle_inflation', 0.3)

        self.pc_sub = self.create_subscription(
            PointCloud2, '/perception/pointcloud_filtered', self.cloud_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/state_estimation/pose', self.pose_callback, 10
        )
        self.get_logger().info('OctomapBuilder initialized')

    def cloud_callback(self, msg): pass
    def pose_callback(self, msg): pass


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
