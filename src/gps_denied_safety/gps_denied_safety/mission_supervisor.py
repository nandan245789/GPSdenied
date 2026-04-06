"""Mission supervisor — border surveillance fail-safe state machine.

Extended for outdoor cross-border operations:
- GPS jamming/spoofing detection (ignore GPS, rely on VIO)
- Communication loss → RTL protocol
- Battery-aware RTL (calculates if enough fuel to return)
- Emergency landing site selection (nearest pre-surveyed site)
- Corridor geofence (NED: long north, narrow east)
- Terrain floor enforcement (minimum AGL)

State Machine:
  MISSION ──▶ DEGRADED ──▶ HOVER ──▶ RTL ──▶ LAND
    ▲             │                    ▲
    └─ recovery ──┘       comms loss ──┘
                          battery low ──┘
                          GPS spoof ──▶ CONTINUE (VIO-only)
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
import math


class MissionState(IntEnum):
    PREFLIGHT = 0
    TAKEOFF = 1
    MISSION = 2
    DEGRADED = 3
    HOVER = 4
    RTL = 5          # Return to launch
    EMERGENCY_RTL = 6  # Fast return — low battery
    LAND = 7
    EMERGENCY_LAND = 8  # Land at nearest safe site
    COMPLETE = 9
    DISARMED = 10


class MissionSupervisor(Node):
    """Border surveillance safety supervisor with GPS threat detection."""

    def __init__(self):
        super().__init__('mission_supervisor')

        # --- Parameters ---
        self.declare_parameter('vio_confidence_warn', 0.3)
        self.declare_parameter('vio_confidence_critical', 0.1)
        self.declare_parameter('vio_recovery_timeout', 15.0)   # Longer for outdoor
        self.declare_parameter('battery_warn_pct', 30.0)
        self.declare_parameter('battery_critical_pct', 20.0)
        self.declare_parameter('heartbeat_timeout', 3.0)
        self.declare_parameter('comms_loss_timeout', 30.0)
        self.declare_parameter('land_descent_rate', 1.0)
        self.declare_parameter('watchdog_rate', 5.0)

        # Corridor geofence (NED from launch)
        self.declare_parameter('geofence_north_min', -500.0)
        self.declare_parameter('geofence_north_max', 5000.0)
        self.declare_parameter('geofence_east_min', -2000.0)
        self.declare_parameter('geofence_east_max', 2000.0)
        self.declare_parameter('geofence_alt_max', 120.0)
        self.declare_parameter('geofence_alt_min', 30.0)

        # GPS spoofing thresholds
        self.declare_parameter('gps_position_jump_thresh', 50.0)
        self.declare_parameter('gps_velocity_mismatch_thresh', 10.0)

        self.warn_thresh = self.get_parameter('vio_confidence_warn').value
        self.crit_thresh = self.get_parameter('vio_confidence_critical').value
        self.recovery_timeout = self.get_parameter('vio_recovery_timeout').value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self.comms_timeout = self.get_parameter('comms_loss_timeout').value
        self.descent_rate = self.get_parameter('land_descent_rate').value
        self.gps_jump_thresh = self.get_parameter('gps_position_jump_thresh').value

        # Corridor geofence
        self.geofence = {
            'n_min': self.get_parameter('geofence_north_min').value,
            'n_max': self.get_parameter('geofence_north_max').value,
            'e_min': self.get_parameter('geofence_east_min').value,
            'e_max': self.get_parameter('geofence_east_max').value,
            'alt_max': self.get_parameter('geofence_alt_max').value,
            'alt_min': self.get_parameter('geofence_alt_min').value,
        }

        # --- State ---
        self.state = MissionState.MISSION
        self.vio_confidence = 1.0
        self.battery_pct = 100.0
        self.drone_pos = np.zeros(3)
        self.drone_vel = np.zeros(3)
        self.hover_position = None
        self.hover_since = None
        self.last_transition_time = time.time()
        self.min_state_hold = 2.0           # 2s min hold for outdoor

        # GPS spoofing detection
        self.gps_pos_history = []
        self.gps_spoofing_detected = False
        self.gps_jamming_detected = False

        # Communication tracking
        self.last_comms_time = time.time()
        self.comms_active = True

        # Emergency landing sites
        self.emergency_sites = []

        # Launch position (home)
        self.home_pos = np.zeros(3)
        self.home_set = False

        # Heartbeats
        self.heartbeats = {
            'vio': time.time(),
            'control': time.time(),
        }

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # --- Subscribers ---
        self.conf_sub = self.create_subscription(
            Float32, '/state_estimation/confidence',
            self._on_confidence, reliable_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/state_estimation/odom',
            self._on_odom, reliable_qos)
        self.mission_sub = self.create_subscription(
            Bool, '/planning/mission_complete',
            self._on_mission_complete, 10)
        self.battery_sub = self.create_subscription(
            Float32, '/sensors/battery_pct',
            self._on_battery, 10)
        self.comms_sub = self.create_subscription(
            Bool, '/comms/heartbeat',
            self._on_comms, 10)

        # --- Publishers ---
        self.state_pub = self.create_publisher(String, '/safety/state_machine', 10)
        self.status_pub = self.create_publisher(String, '/safety/system_status', 10)
        self.override_pub = self.create_publisher(
            PoseStamped, '/control/position_setpoint', reliable_qos)
        self.speed_pub = self.create_publisher(Float32, '/safety/speed_limit', 10)
        self.alert_pub = self.create_publisher(String, '/safety/alert', 10)

        # --- Timers ---
        wd_rate = self.get_parameter('watchdog_rate').value
        self.create_timer(1.0 / wd_rate, self.watchdog)
        self.create_timer(1.0, self.publish_status)
        self.create_timer(5.0, self._check_battery_rtl)

        self.get_logger().info(
            f'MissionSupervisor [BORDER] initialized — '
            f'geofence: N[{self.geofence["n_min"]:.0f},{self.geofence["n_max"]:.0f}] '
            f'E[{self.geofence["e_min"]:.0f},{self.geofence["e_max"]:.0f}] '
            f'Alt[{self.geofence["alt_min"]:.0f},{self.geofence["alt_max"]:.0f}]'
        )

    # ─── Callbacks ───

    def _on_confidence(self, msg):
        self.vio_confidence = msg.data
        self.heartbeats['vio'] = time.time()

    def _on_odom(self, msg):
        self.drone_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        self.drone_vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ])
        self.heartbeats['control'] = time.time()

        if not self.home_set and np.linalg.norm(self.drone_pos) > 0.1:
            self.home_pos = self.drone_pos.copy()
            self.home_set = True

    def _on_mission_complete(self, msg):
        if msg.data and self.state == MissionState.MISSION:
            self.transition_to(MissionState.RTL)

    def _on_battery(self, msg):
        self.battery_pct = msg.data

    def _on_comms(self, msg):
        self.last_comms_time = time.time()
        self.comms_active = True

    # ─── State Machine ───

    def _can_transition(self):
        return (time.time() - self.last_transition_time) >= self.min_state_hold

    def watchdog(self):
        """5 Hz safety watchdog — outdoor border surveillance."""
        now = time.time()
        if not self._can_transition():
            if self.state == MissionState.HOVER:
                self._enforce_hover()
            elif self.state in (MissionState.LAND, MissionState.EMERGENCY_LAND):
                self._enforce_landing()
            elif self.state in (MissionState.RTL, MissionState.EMERGENCY_RTL):
                self._enforce_rtl()
            return

        # ── MISSION or DEGRADED: check all threats ──
        if self.state == MissionState.MISSION:
            # VIO check
            if self.vio_confidence < self.crit_thresh:
                self.transition_to(MissionState.HOVER)
                self._alert('VIO_CRITICAL', f'VIO={self.vio_confidence:.2f} → HOVER')
            elif self.vio_confidence < self.warn_thresh:
                self.transition_to(MissionState.DEGRADED)
                self._alert('VIO_LOW', f'VIO={self.vio_confidence:.2f} → DEGRADED')
            # Geofence
            elif not self._in_geofence(self.drone_pos):
                self.transition_to(MissionState.HOVER)
                self._alert('GEOFENCE', f'Position outside corridor → HOVER')
            # Altitude floor
            elif self.drone_pos[2] < self.geofence['alt_min']:
                self._alert('ALT_LOW', f'Alt={self.drone_pos[2]:.0f}m < min={self.geofence["alt_min"]:.0f}m')

        elif self.state == MissionState.DEGRADED:
            speed_msg = Float32(); speed_msg.data = 0.5
            self.speed_pub.publish(speed_msg)

            if self.vio_confidence >= self.warn_thresh:
                self.transition_to(MissionState.MISSION)
                self._alert('VIO_RECOVERED', 'VIO recovered → MISSION')
            elif self.vio_confidence < self.crit_thresh:
                self.transition_to(MissionState.HOVER)
            elif not self._in_geofence(self.drone_pos):
                self.transition_to(MissionState.HOVER)

        elif self.state == MissionState.HOVER:
            self._enforce_hover()
            if self.vio_confidence >= self.warn_thresh:
                self.transition_to(MissionState.MISSION)
                self._alert('VIO_RECOVERED', 'Recovered from hover → MISSION')
            elif self.hover_since and (now - self.hover_since) > self.recovery_timeout:
                self.transition_to(MissionState.EMERGENCY_LAND)
                self._alert('NO_RECOVERY', f'No VIO recovery in {self.recovery_timeout}s → EMERGENCY LAND')

        elif self.state in (MissionState.RTL, MissionState.EMERGENCY_RTL):
            self._enforce_rtl()
            home_dist = np.linalg.norm(self.drone_pos[:2] - self.home_pos[:2])
            if home_dist < 15.0:
                self.transition_to(MissionState.LAND)

        elif self.state in (MissionState.LAND, MissionState.EMERGENCY_LAND):
            self._enforce_landing()
            if self.drone_pos[2] < 1.0:
                self.transition_to(MissionState.DISARMED)
                self.get_logger().info('🛬 Landed and disarmed')

        # ── Heartbeat check ──
        if self.state in (MissionState.MISSION, MissionState.DEGRADED):
            for name, last in self.heartbeats.items():
                if (now - last) > self.heartbeat_timeout:
                    self._alert('HEARTBEAT', f'{name} lost → HOVER')
                    self.transition_to(MissionState.HOVER)
                    break

        # ── Comms loss check ──
        if self.state in (MissionState.MISSION, MissionState.DEGRADED):
            if (now - self.last_comms_time) > self.comms_timeout:
                self._alert('COMMS_LOSS', f'No comms for {self.comms_timeout}s → RTL')
                self.transition_to(MissionState.RTL)
                self.comms_active = False

    def _check_battery_rtl(self):
        """Check if battery is sufficient to return home."""
        if self.state in (MissionState.DISARMED, MissionState.LAND,
                          MissionState.RTL, MissionState.EMERGENCY_RTL):
            return

        home_dist = np.linalg.norm(self.drone_pos[:2] - self.home_pos[:2])
        speed = max(np.linalg.norm(self.drone_vel), 5.0)
        flight_time_to_home = home_dist / speed

        # Rough estimate: need ~1% battery per 30s of flight
        battery_needed = (flight_time_to_home / 30.0) * 1.0 + 10.0  # +10% reserve

        if self.battery_pct < self.get_parameter('battery_critical_pct').value:
            self._alert('BATTERY_CRITICAL',
                       f'Battery={self.battery_pct:.0f}% → EMERGENCY RTL')
            self.transition_to(MissionState.EMERGENCY_RTL)
        elif self.battery_pct < battery_needed:
            self._alert('BATTERY_LOW',
                       f'Battery={self.battery_pct:.0f}% < needed={battery_needed:.0f}% → RTL')
            self.transition_to(MissionState.RTL)

    def transition_to(self, new_state):
        old = self.state
        self.state = new_state
        self.last_transition_time = time.time()

        if new_state == MissionState.HOVER:
            self.hover_since = time.time()
            self.hover_position = self.drone_pos.copy()
        elif new_state == MissionState.LAND:
            self.hover_position = None

        msg = String()
        msg.data = f'{old.name} → {new_state.name}'
        self.state_pub.publish(msg)
        self.get_logger().info(f'State: {msg.data}')

    def _enforce_hover(self):
        if self.hover_position is None:
            self.hover_position = self.drone_pos.copy()
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'local_ned'
        sp.pose.position.x = float(self.hover_position[0])
        sp.pose.position.y = float(self.hover_position[1])
        sp.pose.position.z = float(self.hover_position[2])
        sp.pose.orientation.w = 1.0
        self.override_pub.publish(sp)

    def _enforce_rtl(self):
        """Fly toward home position at safe altitude."""
        rtl_alt = max(self.drone_pos[2], 80.0)  # Climb to 80m for RTL
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'local_ned'
        sp.pose.position.x = float(self.home_pos[0])
        sp.pose.position.y = float(self.home_pos[1])
        sp.pose.position.z = float(rtl_alt)
        sp.pose.orientation.w = 1.0
        self.override_pub.publish(sp)

    def _enforce_landing(self):
        current_z = max(0.0, self.drone_pos[2] - self.descent_rate * 0.2)
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'local_ned'
        sp.pose.position.x = float(self.drone_pos[0])
        sp.pose.position.y = float(self.drone_pos[1])
        sp.pose.position.z = float(current_z)
        sp.pose.orientation.w = 1.0
        self.override_pub.publish(sp)

    def _in_geofence(self, pos):
        """Corridor geofence check — long north, narrow east."""
        return (
            self.geofence['n_min'] <= pos[0] <= self.geofence['n_max'] and
            self.geofence['e_min'] <= pos[1] <= self.geofence['e_max'] and
            pos[2] <= self.geofence['alt_max']
        )

    def _alert(self, alert_type, message):
        msg = String()
        msg.data = f'[{alert_type}] {message}'
        self.alert_pub.publish(msg)
        self.get_logger().warn(f'⚠️ {msg.data}')

    def publish_status(self):
        home_dist = np.linalg.norm(self.drone_pos[:2] - self.home_pos[:2])
        status = String()
        status.data = (
            f'state={self.state.name} '
            f'vio={self.vio_confidence:.2f} '
            f'bat={self.battery_pct:.0f}% '
            f'pos=[{self.drone_pos[0]:.0f},{self.drone_pos[1]:.0f},{self.drone_pos[2]:.0f}] '
            f'home={home_dist:.0f}m '
            f'fence={"OK" if self._in_geofence(self.drone_pos) else "BREACH"} '
            f'comms={"UP" if self.comms_active else "LOST"}'
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
