"""Safety monitor — subsystem heartbeat checker and health aggregator."""

import rclpy
from rclpy.node import Node


class SafetyMonitor(Node):
    """Monitor subsystem heartbeats, battery, and geofence.

    TODO: Subscribe to all subsystem diagnostics, aggregate health,
    publish to /safety/system_status, enforce geofence boundaries.
    """

    def __init__(self):
        super().__init__('safety_monitor')
        self.declare_parameter('geofence_x_min', -10.0)
        self.declare_parameter('geofence_x_max', 10.0)
        self.declare_parameter('geofence_y_min', -10.0)
        self.declare_parameter('geofence_y_max', 10.0)
        self.declare_parameter('geofence_z_max', 5.0)
        self.get_logger().info('SafetyMonitor initialized')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
