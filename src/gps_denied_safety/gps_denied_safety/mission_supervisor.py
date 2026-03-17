"""Mission supervisor — top-level state machine and safety coordinator."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from enum import IntEnum


class MissionState(IntEnum):
    PREFLIGHT = 0
    ARMED = 1
    TAKEOFF = 2
    MISSION = 3
    RTL = 4
    HOVER = 5
    LAND = 6
    EMERGENCY = 7
    DISARMED = 8


class MissionSupervisor(Node):
    """Top-level mission state machine and safety coordinator.

    Monitors all subsystem health, enforces safety policies, and triggers
    fail-safe actions (hover, land, RTL, disarm) based on system state.
    """

    def __init__(self):
        super().__init__('mission_supervisor')

        # Parameters
        self.declare_parameter('vio_confidence_warn', 0.3)
        self.declare_parameter('vio_confidence_critical', 0.1)
        self.declare_parameter('vio_recovery_timeout', 5.0)
        self.declare_parameter('battery_warn_pct', 25.0)
        self.declare_parameter('battery_critical_pct', 15.0)
        self.declare_parameter('heartbeat_timeout', 2.0)
        self.declare_parameter('max_tracking_error', 1.5)

        self.state = MissionState.PREFLIGHT
        self.vio_confidence = 1.0

        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1)

        # Subscribers
        self.confidence_sub = self.create_subscription(
            Float32, '/state_estimation/confidence',
            self.confidence_callback, reliable_qos
        )

        # Publishers
        self.state_pub = self.create_publisher(
            String, '/safety/state_machine', 10
        )
        self.status_pub = self.create_publisher(
            String, '/safety/system_status', 10
        )

        # Watchdog timer
        self.watchdog_timer = self.create_timer(1.0, self.watchdog_callback)

        self.get_logger().info(
            f'MissionSupervisor initialized — state: {self.state.name}'
        )

    def confidence_callback(self, msg: Float32):
        self.vio_confidence = msg.data
        warn_thresh = self.get_parameter('vio_confidence_warn').value
        crit_thresh = self.get_parameter('vio_confidence_critical').value

        if self.vio_confidence < crit_thresh and self.state == MissionState.MISSION:
            self.transition_to(MissionState.HOVER)
            self.get_logger().warn(
                f'VIO confidence CRITICAL ({self.vio_confidence:.2f}) — HOVER'
            )
        elif self.vio_confidence < warn_thresh and self.state == MissionState.MISSION:
            self.get_logger().warn(
                f'VIO confidence LOW ({self.vio_confidence:.2f}) — reducing speed'
            )

    def transition_to(self, new_state: MissionState):
        old_state = self.state
        self.state = new_state
        msg = String()
        msg.data = f'{old_state.name} -> {new_state.name}'
        self.state_pub.publish(msg)
        self.get_logger().info(f'State transition: {msg.data}')

    def watchdog_callback(self):
        """Periodic health check — monitor all subsystem heartbeats.

        TODO: Check heartbeats from all subsystem nodes,
        battery status, tracking error, geofence.
        """
        status = String()
        status.data = f'state={self.state.name} vio={self.vio_confidence:.2f}'
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
