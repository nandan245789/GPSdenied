"""VIO node — Visual-Inertial Odometry wrapper for VINS-Fusion."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster


class VIONode(Node):
    """Visual-Inertial Odometry — wraps VINS-Fusion for pose estimation.

    Subscribes to stereo images + IMU, runs VIO pipeline, publishes
    6-DoF pose with covariance, TF broadcast, and confidence score.
    """

    def __init__(self):
        super().__init__('vio_node')

        self.declare_parameter('vio_algorithm', 'vins_fusion')
        self.declare_parameter('feature_threshold', 30)
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('max_covariance_position', 1.0)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        # Subscribers
        self.img_sub = self.create_subscription(
            Image, '/sensors/camera/infra1/image_raw',
            self.image_callback, sensor_qos
        )
        self.imu_sub = self.create_subscription(
            Imu, '/sensors/camera/imu', self.imu_callback, sensor_qos
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/state_estimation/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/state_estimation/pose', 10)
        self.confidence_pub = self.create_publisher(Float32, '/state_estimation/confidence', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'VIO node initialized — algorithm: '
            f'{self.get_parameter("vio_algorithm").value}'
        )

    def image_callback(self, msg: Image):
        """Process incoming stereo frame for VIO.
        TODO: Feed to VINS-Fusion, get pose, publish odom + TF.
        """
        pass

    def imu_callback(self, msg: Imu):
        """Process IMU for pre-integration.
        TODO: Feed to VINS-Fusion IMU pre-integrator.
        """
        pass


def main(args=None):
    rclpy.init(args=args)
    node = VIONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
