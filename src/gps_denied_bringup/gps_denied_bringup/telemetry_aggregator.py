"""Telemetry aggregator — collects metrics from all subsystems."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TelemetryAggregator(Node):
    def __init__(self):
        super().__init__('telemetry_aggregator')
        self.metrics_pub = self.create_publisher(String, '/telemetry/metrics', 10)
        self.timer = self.create_timer(1.0, self.aggregate_metrics)
        self.get_logger().info('TelemetryAggregator initialized')

    def aggregate_metrics(self):
        """TODO: Collect metrics from all subsystem status topics."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
