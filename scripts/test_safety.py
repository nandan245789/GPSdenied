"""Safety test — demonstrates all fail-safe scenarios.

Runs the drone through a mission, then injects failures to test
the safety state machine:

Test 1: VIO confidence drop → DEGRADED (speed reduced)
Test 2: VIO confidence critical → HOVER (hold position)
Test 3: VIO recovery → back to MISSION
Test 4: Sustained VIO loss → auto LAND
Test 5: Geofence breach → HOVER

Usage (inside Docker):
  cd /workspace && colcon build --symlink-install
  source install/setup.bash
  python3 scripts/test_safety.py
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String, Bool
import numpy as np
import time


class SafetyTestHarness(Node):
    """Inject failures and monitor safety state machine response."""

    def __init__(self):
        super().__init__('safety_test_harness')

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # Publishers — simulate sensor data
        self.odom_pub = self.create_publisher(Odometry, '/state_estimation/odom', reliable_qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/state_estimation/pose', reliable_qos)
        self.confidence_pub = self.create_publisher(Float32, '/state_estimation/confidence', 10)

        # Subscribe to safety outputs
        self.state_sub = self.create_subscription(
            String, '/safety/state_machine', self.state_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, '/safety/system_status', self.status_callback, 10
        )
        self.speed_sub = self.create_subscription(
            Float32, '/safety/speed_limit', self.speed_callback, 10
        )

        # State tracking
        self.current_state = 'MISSION'
        self.position = np.array([0.0, 0.0, 1.5])
        self.confidence = 0.9
        self.test_results = []
        self.speed_limit = 1.0

        # Publish simulated data at 50 Hz
        self.sim_timer = self.create_timer(0.02, self.publish_sim_data)

    def state_callback(self, msg: String):
        parts = msg.data.split(' → ')
        if len(parts) == 2:
            self.current_state = parts[1]

    def status_callback(self, msg: String):
        pass

    def speed_callback(self, msg: Float32):
        self.speed_limit = msg.data

    def publish_sim_data(self):
        """Publish fake sensor data."""
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = float(self.position[0])
        odom.pose.pose.position.y = float(self.position[1])
        odom.pose.pose.position.z = float(self.position[2])
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        conf = Float32()
        conf.data = float(self.confidence)
        self.confidence_pub.publish(conf)


def run_test(name, description, test_fn, harness, executor, expected_state):
    """Run a single safety test."""
    print(f'\n{"─" * 60}')
    print(f'  TEST: {name}')
    print(f'  {description}')
    print(f'{"─" * 60}')

    test_fn(harness)

    # Spin for a few seconds to let state machine react
    start = time.time()
    while (time.time() - start) < 3.0:
        executor.spin_once(timeout_sec=0.05)

    passed = harness.current_state == expected_state
    result = '✅ PASS' if passed else f'❌ FAIL (got {harness.current_state})'
    print(f'  Expected state: {expected_state}')
    print(f'  Actual state:   {harness.current_state}')
    print(f'  Result: {result}')
    harness.test_results.append((name, passed))
    return passed


def main():
    rclpy.init()

    print()
    print('=' * 60)
    print('  🛡️  Safety System Test Suite')
    print('  Testing fail-safe state machine responses')
    print('=' * 60)

    harness = SafetyTestHarness()

    from gps_denied_safety.mission_supervisor import MissionSupervisor
    supervisor = MissionSupervisor()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(harness)
    executor.add_node(supervisor)

    # Warm up
    print('\n  Warming up nodes...')
    start = time.time()
    while (time.time() - start) < 2.0:
        executor.spin_once(timeout_sec=0.05)
    print(f'  State: {harness.current_state}')

    # ── Test 1: VIO confidence drop → DEGRADED ──
    def test_degraded(h):
        h.confidence = 0.2  # Below warn threshold (0.3)
        print(f'  → Setting VIO confidence to {h.confidence}')

    run_test(
        'VIO Degradation',
        'Dropping VIO confidence below warning threshold (0.3)',
        test_degraded, harness, executor, 'DEGRADED'
    )

    # ── Test 2: VIO confidence critical → HOVER ──
    def test_hover(h):
        h.confidence = 0.05  # Below critical threshold (0.1)
        print(f'  → Setting VIO confidence to {h.confidence} (CRITICAL)')

    run_test(
        'VIO Critical → Hover',
        'Dropping VIO confidence below critical threshold (0.1)',
        test_hover, harness, executor, 'HOVER'
    )

    # ── Test 3: VIO recovery → back to MISSION ──
    def test_recovery(h):
        h.confidence = 0.85  # Recover above warn threshold
        print(f'  → Restoring VIO confidence to {h.confidence}')

    run_test(
        'VIO Recovery',
        'Restoring VIO confidence above threshold',
        test_recovery, harness, executor, 'MISSION'
    )

    # ── Test 4: Sustained VIO loss → LAND ──
    def test_land(h):
        h.confidence = 0.05  # Critical
        print(f'  → Setting VIO confidence to {h.confidence} (sustained)')
        print(f'  → Waiting {6.0}s for landing escalation...')

    harness.confidence = 0.05
    start = time.time()
    while (time.time() - start) < 8.0:  # Wait for recovery_timeout (5s) + margin
        executor.spin_once(timeout_sec=0.05)

    print(f'\n{"─" * 60}')
    print(f'  TEST: Sustained VIO Loss → Land')
    print(f'  VIO critical for >5 seconds, should trigger landing')
    print(f'{"─" * 60}')
    landed = harness.current_state in ('LAND', 'DISARMED')
    result = '✅ PASS' if landed else f'❌ FAIL (got {harness.current_state})'
    print(f'  Expected: LAND or DISARMED')
    print(f'  Actual:   {harness.current_state}')
    print(f'  Result: {result}')
    harness.test_results.append(('Sustained VIO Loss → Land', landed))

    # ── Reset for geofence test ──
    harness.confidence = 0.9
    harness.position = np.array([0.0, 0.0, 1.5])
    supervisor.state = supervisor.state.__class__(2)  # Reset to MISSION
    harness.current_state = 'MISSION'

    start = time.time()
    while (time.time() - start) < 2.0:
        executor.spin_once(timeout_sec=0.05)

    # ── Test 5: Geofence breach → HOVER ──
    def test_geofence(h):
        h.position = np.array([15.0, 0.0, 1.5])  # Outside ±10m geofence
        print(f'  → Moving drone to [{h.position[0]}, {h.position[1]}, {h.position[2]}]')
        print(f'  → Geofence is ±10m — this is outside!')

    run_test(
        'Geofence Breach',
        'Moving drone outside ±10m geofence boundary',
        test_geofence, harness, executor, 'HOVER'
    )

    # ── Results Summary ──
    print()
    print('=' * 60)
    print('  📊 Test Results Summary')
    print('=' * 60)
    all_passed = True
    for name, passed in harness.test_results:
        icon = '✅' if passed else '❌'
        print(f'  {icon}  {name}')
        if not passed:
            all_passed = False

    passed_count = sum(1 for _, p in harness.test_results if p)
    total = len(harness.test_results)
    print(f'\n  {passed_count}/{total} tests passed')

    if all_passed:
        print('\n  🎉 ALL SAFETY TESTS PASSED')
    else:
        print('\n  ⚠️  Some tests failed — review state machine logic')

    print('=' * 60)
    print()

    executor.shutdown()
    harness.destroy_node()
    supervisor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
