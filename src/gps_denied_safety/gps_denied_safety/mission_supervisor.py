"""Mission supervisor — hierarchical fail-safe state machine.

This is the safety brain of the drone. It monitors all subsystems
and triggers escalating safety responses:

  MISSION → DEGRADED → HOVER → LAND → EMERGENCY

State Machine:
  ┌────────┐   confidence<0.3   ┌──────────┐
  │MISSION │ ──────────────────▶│ DEGRADED │
  │(flying)│   (reduce speed)   │(slow fly)│
  └────┬───┘                    └────┬─────┘
       │                             │ confidence<0.1
       │ mission complete            ▼
       │                        ┌─────────┐
       ▼                        │  HOVER  │ ◀── node crash
  ┌────────┐   5s timeout       │ (hold)  │
  │COMPLETE│   ──────────▶ ┌────┴─────────┘
  └────────┘               │    │
                           │    │ 5s no recovery
                           │    ▼
                      ┌────┴─────┐
                      │   LAND   │ ◀── battery<15%
                      │(descend) │
                      └────┬─────┘
                           │ contact / timeout
                           ▼
                      ┌──────────┐
                      │ DISARMED │
                      └──────────┘
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from enum import IntEnum
import numpy as np
import time


class MissionState(IntEnum):
    PREFLIGHT = 0
    TAKEOFF = 1
    MISSION = 2
    DEGRADED = 3
    HOVER = 4
    RTL = 5
    LAND = 6
    EMERGENCY = 7
    COMPLETE = 8
    DISARMED = 9


class MissionSupervisor(Node):
    """Top-level safety state machine with hierarchical fail-safe escalation."""

    def __init__(self):
        super().__init__('mission_supervisor')

        # --- Parameters ---
        self.declare_parameter('vio_confidence_warn', 0.3)
        self.declare_parameter('vio_confidence_critical', 0.1)
        self.declare_parameter('vio_recovery_timeout', 5.0)
        self.declare_parameter('battery_warn_pct', 25.0)
        self.declare_parameter('battery_critical_pct', 15.0)
        self.declare_parameter('heartbeat_timeout', 2.0)
        self.declare_parameter('max_tracking_error', 1.5)
        self.declare_parameter('geofence_x_min', -10.0)
        self.declare_parameter('geofence_x_max', 10.0)
        self.declare_parameter('geofence_y_min', -10.0)
        self.declare_parameter('geofence_y_max', 10.0)
        self.declare_parameter('geofence_z_max', 5.0)
        self.declare_parameter('land_descent_rate', 0.3)
        self.declare_parameter('watchdog_rate', 5.0)

        self.warn_thresh = self.get_parameter('vio_confidence_warn').value
        self.crit_thresh = self.get_parameter('vio_confidence_critical').value
        self.recovery_timeout = self.get_parameter('vio_recovery_timeout').value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self.max_tracking_err = self.get_parameter('max_tracking_error').value
        self.descent_rate = self.get_parameter('land_descent_rate').value

        # Geofence
        self.geofence = {
            'x_min': self.get_parameter('geofence_x_min').value,
            'x_max': self.get_parameter('geofence_x_max').value,
            'y_min': self.get_parameter('geofence_y_min').value,
            'y_max': self.get_parameter('geofence_y_max').value,
            'z_max': self.get_parameter('geofence_z_max').value,
        }

        # --- State ---
        self.state = MissionState.MISSION  # Start in mission mode
        self.vio_confidence = 1.0
        self.battery_pct = 100.0
        self.drone_pos = np.zeros(3)
        self.hover_position = None
        self.degraded_since = None
        self.hover_since = None
        self.last_transition_time = time.time()
        self.min_state_hold = 1.0  # Minimum 1s in any state before transitioning

        # Heartbeat tracking (only nodes that actually publish)
        self.heartbeats = {
            'vio': time.time(),
            'control': time.time(),
        }

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # --- Subscribers ---
        self.confidence_sub = self.create_subscription(
            Float32, '/state_estimation/confidence',
            self.confidence_callback, reliable_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/state_estimation/odom',
            self.odom_callback, reliable_qos
        )
        self.mission_complete_sub = self.create_subscription(
            Bool, '/planning/mission_complete',
            self.mission_complete_callback, 10
        )

        # --- Publishers ---
        self.state_pub = self.create_publisher(String, '/safety/state_machine', 10)
        self.status_pub = self.create_publisher(String, '/safety/system_status', 10)
        self.override_pub = self.create_publisher(
            PoseStamped, '/control/position_setpoint', reliable_qos
        )
        self.speed_limit_pub = self.create_publisher(Float32, '/safety/speed_limit', 10)

        # --- Watchdog timer ---
        watchdog_rate = self.get_parameter('watchdog_rate').value
        self.watchdog_timer = self.create_timer(1.0 / watchdog_rate, self.watchdog)

        # Status publish timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f'MissionSupervisor initialized — state: {self.state.name}'
        )

    # ─── Callbacks ───

    def confidence_callback(self, msg: Float32):
        self.vio_confidence = msg.data
        self.heartbeats['vio'] = time.time()

    def odom_callback(self, msg: Odometry):
        self.drone_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        self.heartbeats['control'] = time.time()

    def mission_complete_callback(self, msg: Bool):
        if msg.data and self.state == MissionState.MISSION:
            self.transition_to(MissionState.COMPLETE)

    # ─── State machine logic ───

    def _can_transition(self):
        """Prevent rapid state flapping — enforce minimum hold time."""
        return (time.time() - self.last_transition_time) >= self.min_state_hold

    def watchdog(self):
        """5 Hz watchdog — check all safety conditions."""
        now = time.time()

        if not self._can_transition():
            # Still in hold period — don't change state, but enforce hover/land
            if self.state == MissionState.HOVER:
                self._enforce_hover()
            elif self.state == MissionState.LAND:
                self._enforce_landing()
            return

        # ── MISSION state: check for degradation ──
        if self.state == MissionState.MISSION:
            if self.vio_confidence < self.crit_thresh:
                self.transition_to(MissionState.HOVER)
                self.get_logger().error(
                    f'⚠️ VIO CRITICAL ({self.vio_confidence:.2f}) → HOVER'
                )
            elif self.vio_confidence < self.warn_thresh:
                self.transition_to(MissionState.DEGRADED)
                self.get_logger().warn(
                    f'⚠️ VIO LOW ({self.vio_confidence:.2f}) → DEGRADED'
                )
            elif not self._in_geofence(self.drone_pos):
                self.get_logger().error(
                    f'🚧 GEOFENCE BREACH → HOVER'
                )
                self.transition_to(MissionState.HOVER)

        # ── DEGRADED: check recovery or escalate (elif = no cascade) ──
        elif self.state == MissionState.DEGRADED:
            # Publish speed limit
            speed_msg = Float32()
            speed_msg.data = 0.5
            self.speed_limit_pub.publish(speed_msg)

            if self.vio_confidence >= self.warn_thresh:
                self.transition_to(MissionState.MISSION)
                self.get_logger().info('✅ VIO recovered → MISSION')
            elif self.vio_confidence < self.crit_thresh:
                self.transition_to(MissionState.HOVER)
                self.get_logger().error('⚠️ VIO CRITICAL in DEGRADED → HOVER')
            elif not self._in_geofence(self.drone_pos):
                self.transition_to(MissionState.HOVER)

        # ── HOVER: enforce position hold, check recovery or escalate ──
        elif self.state == MissionState.HOVER:
            self._enforce_hover()

            if self.vio_confidence >= self.warn_thresh:
                self.transition_to(MissionState.MISSION)
                self.get_logger().info('✅ VIO recovered from hover → MISSION')
            elif self.hover_since and (now - self.hover_since) > self.recovery_timeout:
                self.transition_to(MissionState.LAND)
                self.get_logger().error(
                    f'❌ No recovery in {self.recovery_timeout}s → LAND'
                )

        # ── LAND: descend until ground ──
        elif self.state == MissionState.LAND:
            self._enforce_landing()
            if self.drone_pos[2] < 0.1:
                self.transition_to(MissionState.DISARMED)
                self.get_logger().info('🛬 Landed and disarmed')

        # ── Heartbeat check (applies to MISSION + DEGRADED only) ──
        if self.state in (MissionState.MISSION, MissionState.DEGRADED):
            for name, last_beat in self.heartbeats.items():
                if (now - last_beat) > self.heartbeat_timeout:
                    self.get_logger().error(
                        f'💀 {name} heartbeat lost → HOVER'
                    )
                    self.transition_to(MissionState.HOVER)
                    break  # Only one transition per tick

    def transition_to(self, new_state: MissionState):
        """Transition to a new state with logging."""
        old_state = self.state
        self.state = new_state
        self.last_transition_time = time.time()

        # Record timing
        if new_state == MissionState.DEGRADED:
            self.degraded_since = time.time()
        if new_state == MissionState.HOVER:
            self.hover_since = time.time()
            self.hover_position = self.drone_pos.copy()
        if new_state == MissionState.LAND:
            self.hover_position = None

        # Publish transition
        msg = String()
        msg.data = f'{old_state.name} → {new_state.name}'
        self.state_pub.publish(msg)
        self.get_logger().info(f'State: {msg.data}')

    def _enforce_hover(self):
        """Override planner — hold current position."""
        if self.hover_position is None:
            self.hover_position = self.drone_pos.copy()

        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'odom'
        sp.pose.position.x = float(self.hover_position[0])
        sp.pose.position.y = float(self.hover_position[1])
        sp.pose.position.z = float(self.hover_position[2])
        sp.pose.orientation.w = 1.0
        self.override_pub.publish(sp)

    def _enforce_landing(self):
        """Override planner — descend at fixed rate."""
        current_z = max(0.0, self.drone_pos[2] - self.descent_rate * 0.2)

        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'odom'
        sp.pose.position.x = float(self.drone_pos[0])
        sp.pose.position.y = float(self.drone_pos[1])
        sp.pose.position.z = float(current_z)
        sp.pose.orientation.w = 1.0
        self.override_pub.publish(sp)

    def _in_geofence(self, pos):
        """Check if position is within geofence boundaries."""
        return (
            self.geofence['x_min'] <= pos[0] <= self.geofence['x_max'] and
            self.geofence['y_min'] <= pos[1] <= self.geofence['y_max'] and
            pos[2] <= self.geofence['z_max']
        )

    def publish_status(self):
        """Publish system status summary at 1 Hz."""
        status = String()
        status.data = (
            f'state={self.state.name} '
            f'vio={self.vio_confidence:.2f} '
            f'pos=[{self.drone_pos[0]:.1f},{self.drone_pos[1]:.1f},{self.drone_pos[2]:.1f}] '
            f'fence={"OK" if self._in_geofence(self.drone_pos) else "BREACH"}'
        )
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


if __name__ == '__main__':
    main()
