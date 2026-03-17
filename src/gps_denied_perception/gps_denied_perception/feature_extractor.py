"""Feature extractor node — ORB features from stereo images."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class FeatureExtractor(Node):
    """Extract ORB features from incoming camera images.

    Subscribes to RGB/stereo images, extracts keypoints and descriptors,
    and publishes tracked features for state estimation.
    """

    def __init__(self):
        super().__init__('feature_extractor')

        # Parameters
        self.declare_parameter('max_features', 500)
        self.declare_parameter('scale_factor', 1.2)
        self.declare_parameter('n_levels', 8)
        self.declare_parameter('input_topic', '/sensors/camera/color/image_raw')
        self.declare_parameter('output_topic', '/perception/features')

        input_topic = self.get_parameter('input_topic').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            Image, input_topic, self.image_callback, sensor_qos
        )

        self.get_logger().info(
            f'FeatureExtractor initialized — subscribing to {input_topic}'
        )

    def image_callback(self, msg: Image):
        """Process incoming image frame.

        TODO: Implement ORB extraction, feature tracking, and publishing.
        """
        # Stub — Phase 1 implementation
        pass


def main(args=None):
    rclpy.init(args=args)
    node = FeatureExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
