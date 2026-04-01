"""Phase 5 — Integration test suite (v3 — robust single-process).

Runs all scenarios sequentially in one rclpy context.
Fixes: continuous goal republishing, proper node cleanup.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String, Bool
import numpy as np
import time
import json
import os


def run_scenario(name, waypoints, obstacles, noise, timeout):
    """Run a single mission scenario. Returns dict of results."""
    r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                   history=HistoryPolicy.KEEP_LAST, depth=1)

    # ── Shared state ──
    state = {
        'pos': np.array([0.0, 0.0, 0.0]),
        'vel': np.zeros(3),
        'sp': np.array([0.0, 0.0, 1.5]),
        'wp_idx': 0,
        'arrived': False,
        'arrived_time': None,
        'mission_done': False,
        'collisions': 0,
        'total_dist': 0.0,
        'prev_pos': np.array([0.0, 0.0, 0.0]),
        'gt_errors': [],
        'gt_pos': None,
        'vio_pos': None,
    }

    # ── Drone physics node ──
    class Drone(Node):
        def __init__(self):
            super().__init__('test_drone')
            self.sp_sub = self.create_subscription(
                PoseStamped, '/control/position_setpoint', self._on_sp, r)
            self.odom_pub = self.create_publisher(Odometry, '/state_estimation/odom', r)
            self.pose_pub = self.create_publisher(PoseStamped, '/state_estimation/pose', r)
            self.conf_pub = self.create_publisher(Float32, '/state_estimation/confidence', 10)
            self.gt_pub = self.create_publisher(PoseStamped, '/model/x500/pose', 10)
            self.grid_pub = self.create_publisher(OccupancyGrid, '/mapping/occupancy_grid', 10)
            self.goal_pub = self.create_publisher(PoseStamped, '/planning/current_goal', r)
            self.status_pub = self.create_publisher(String, '/planning/mission_status', 10)
            self.complete_pub = self.create_publisher(Bool, '/planning/mission_complete', 10)
            self.create_timer(0.02, self._physics)
            self.create_timer(0.5, self._pub_goal)    # Republish goal every 0.5s
            self.create_timer(0.2, self._check_wp)    # Check arrival at 5 Hz
            self.create_timer(2.0, self._pub_grid)    # Grid every 2s
            self.create_timer(0.02, self._gt_check)   # GT comparison

        def _on_sp(self, m):
            state['sp'] = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])

        def _physics(self):
            e = state['sp'] - state['pos']
            d = np.linalg.norm(e)
            dv = (e / d) * min(d, 1.5) if d > 0.05 else np.zeros(3)
            state['vel'] += (dv - state['vel']) * (0.02 / 0.3)
            state['pos'] += state['vel'] * 0.02
            state['total_dist'] += np.linalg.norm(state['pos'] - state['prev_pos'])
            state['prev_pos'] = state['pos'].copy()

            # Collisions
            for o in obstacles:
                if (abs(state['pos'][0] - o['center'][0]) < o['size'][0]/2 + 0.2 and
                    abs(state['pos'][1] - o['center'][1]) < o['size'][1]/2 + 0.2):
                    state['collisions'] += 1

            # Publish odometry (noisy)
            n = state['pos'] + np.random.normal(0, noise, 3)
            s = self.get_clock().now().to_msg()

            om = Odometry()
            om.header.stamp = s; om.header.frame_id = 'odom'
            om.pose.pose.position.x = float(n[0])
            om.pose.pose.position.y = float(n[1])
            om.pose.pose.position.z = float(n[2])
            om.pose.pose.orientation.w = 1.0
            om.twist.twist.linear.x = float(state['vel'][0])
            om.twist.twist.linear.y = float(state['vel'][1])
            om.twist.twist.linear.z = float(state['vel'][2])
            om.pose.covariance[0] = om.pose.covariance[7] = om.pose.covariance[14] = 0.01
            self.odom_pub.publish(om)

            state['vio_pos'] = n.copy()

            p = PoseStamped(); p.header = om.header; p.pose = om.pose.pose
            self.pose_pub.publish(p)

            c = Float32(); c.data = 0.85 + 0.1 * np.random.random()
            self.conf_pub.publish(c)

            # Ground truth
            gt = PoseStamped(); gt.header.stamp = s; gt.header.frame_id = 'odom'
            gt.pose.position.x = float(state['pos'][0])
            gt.pose.position.y = float(state['pos'][1])
            gt.pose.position.z = float(state['pos'][2])
            gt.pose.orientation.w = 1.0
            self.gt_pub.publish(gt)
            state['gt_pos'] = state['pos'].copy()

        def _gt_check(self):
            if state['vio_pos'] is not None and state['gt_pos'] is not None:
                err = np.linalg.norm(state['vio_pos'] - state['gt_pos'])
                state['gt_errors'].append(err)

        def _pub_goal(self):
            if state['mission_done'] or state['wp_idx'] >= len(waypoints):
                return
            wp = waypoints[state['wp_idx']]
            g = PoseStamped()
            g.header.stamp = self.get_clock().now().to_msg()
            g.header.frame_id = 'odom'
            g.pose.position.x = float(wp['x'])
            g.pose.position.y = float(wp['y'])
            g.pose.position.z = float(wp['z'])
            g.pose.orientation.w = 1.0
            self.goal_pub.publish(g)

        def _check_wp(self):
            if state['mission_done'] or state['wp_idx'] >= len(waypoints):
                return
            wp = waypoints[state['wp_idx']]
            target = np.array([wp['x'], wp['y'], wp['z']])
            dist = np.linalg.norm(state['pos'] - target)

            if dist < 0.5:
                if not state['arrived']:
                    state['arrived'] = True
                    state['arrived_time'] = time.time()
                elif (time.time() - state['arrived_time']) >= wp.get('hold_time', 1.0):
                    state['wp_idx'] += 1
                    state['arrived'] = False
                    if state['wp_idx'] >= len(waypoints):
                        state['mission_done'] = True
                        b = Bool(); b.data = True
                        self.complete_pub.publish(b)
            else:
                state['arrived'] = False

            s = String()
            s.data = f"WP {state['wp_idx']+1}/{len(waypoints)} dist={dist:.2f}m"
            self.status_pub.publish(s)

        def _pub_grid(self):
            res, sz, orig = 0.1, 160, -8.0
            d = np.zeros(sz * sz, dtype=np.int8)
            for o in obstacles:
                cx, cy = o['center']
                sx, sy = o['size']
                for y in range(max(0, int((cy-sy/2-orig)/res)), min(sz, int((cy+sy/2-orig)/res))):
                    for x in range(max(0, int((cx-sx/2-orig)/res)), min(sz, int((cx+sx/2-orig)/res))):
                        d[y*sz+x] = 100
                # Inflation
                for y in range(max(0, int((cy-sy/2-orig)/res)-3), min(sz, int((cy+sy/2-orig)/res)+3)):
                    for x in range(max(0, int((cx-sx/2-orig)/res)-3), min(sz, int((cx+sx/2-orig)/res)+3)):
                        if d[y*sz+x] == 0: d[y*sz+x] = 50
            m = OccupancyGrid()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'odom'
            m.info.resolution = res; m.info.width = sz; m.info.height = sz
            m.info.origin.position.x = orig; m.info.origin.position.y = orig
            m.data = d.tolist()
            self.grid_pub.publish(m)

    # Create nodes
    drone = Drone()

    from gps_denied_planning.local_planner import LocalPlanner
    planner = LocalPlanner()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(drone)
    executor.add_node(planner)

    start = time.time()
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
            if state['mission_done'] or (time.time() - start) > timeout:
                break
    except Exception:
        pass

    elapsed = time.time() - start
    errors = np.array(state['gt_errors']) if state['gt_errors'] else np.array([0.0])
    rmse = float(np.sqrt(np.mean(errors**2)))
    drift = (rmse / max(state['total_dist'], 0.1)) * 100

    executor.shutdown()
    drone.destroy_node()
    planner.destroy_node()

    return {
        'name': name,
        'success': state['mission_done'],
        'time': elapsed,
        'wp': f"{state['wp_idx']}/{len(waypoints)}",
        'wp_reached': state['wp_idx'],
        'wp_total': len(waypoints),
        'rmse': rmse,
        'drift': drift,
        'collisions': state['collisions'],
        'distance': state['total_dist'],
    }


# ── Scenarios ──
OBS = [{'center': [3, 0], 'size': [0.5, 6]}, {'center': [-3, 0], 'size': [0.5, 6]}]

SCENARIOS = [
    ('3-WP Basic',
     [{'x':2,'y':0,'z':1.5,'hold_time':1},
      {'x':5,'y':3,'z':1.5,'hold_time':1},
      {'x':0,'y':0,'z':1.5,'hold_time':0}], OBS, 0.005, 60),

    ('5-WP Extended',
     [{'x':2,'y':2,'z':1.5,'hold_time':1},
      {'x':5,'y':0,'z':1.5,'hold_time':1},
      {'x':5,'y':-3,'z':1.5,'hold_time':1},
      {'x':-2,'y':-2,'z':1.5,'hold_time':1},
      {'x':0,'y':0,'z':1.5,'hold_time':0}], OBS, 0.005, 90),

    ('High Noise (10x)',
     [{'x':2,'y':0,'z':1.5,'hold_time':1},
      {'x':0,'y':0,'z':1.5,'hold_time':0}], [], 0.05, 45),

    ('Dense Obstacles',
     [{'x':1.5,'y':2,'z':1.5,'hold_time':1},
      {'x':1.5,'y':-2,'z':1.5,'hold_time':1},
      {'x':0,'y':0,'z':1.5,'hold_time':0}],
     [{'center':[1.5,0],'size':[0.4,2]}, {'center':[-1.5,1],'size':[0.4,3]}], 0.005, 60),

    ('Repeat 1',
     [{'x':2,'y':0,'z':1.5,'hold_time':1},
      {'x':5,'y':3,'z':1.5,'hold_time':1},
      {'x':0,'y':0,'z':1.5,'hold_time':0}], OBS, 0.005, 60),

    ('Repeat 2',
     [{'x':2,'y':0,'z':1.5,'hold_time':1},
      {'x':5,'y':3,'z':1.5,'hold_time':1},
      {'x':0,'y':0,'z':1.5,'hold_time':0}], OBS, 0.005, 60),

    ('Repeat 3',
     [{'x':2,'y':0,'z':1.5,'hold_time':1},
      {'x':5,'y':3,'z':1.5,'hold_time':1},
      {'x':0,'y':0,'z':1.5,'hold_time':0}], OBS, 0.005, 60),
]


def main():
    rclpy.init()

    print()
    print('=' * 65)
    print('  🧪  Phase 5 — Integration Test Suite')
    print('=' * 65)
    print()

    results = []
    for i, (name, wps, obs, noise, timeout) in enumerate(SCENARIOS):
        print(f'  [{i+1}/{len(SCENARIOS)}] {name}...', end='', flush=True)
        try:
            r = run_scenario(name, wps, obs, noise, timeout)
            icon = '✅' if r['success'] else '❌'
            print(f' {icon}  {r["time"]:.1f}s  {r["wp"]}  '
                  f'RMSE={r["rmse"]:.4f}m  drift={r["drift"]:.2f}%  '
                  f'col={r["collisions"]}  dist={r["distance"]:.1f}m')
            results.append(r)
        except Exception as e:
            print(f' ❌  Error: {e}')
            results.append({'name': name, 'success': False, 'time': 0,
                           'wp': '0/?', 'rmse': 0, 'drift': 0,
                           'collisions': 0, 'distance': 0})

    # ── Report ──
    print()
    print('=' * 65)
    print('  📊  Integration Test Report')
    print('=' * 65)
    print()

    n_pass = sum(1 for r in results if r['success'])
    rate = n_pass / len(results) * 100
    times = [r['time'] for r in results if r['success']]
    rmses = [r['rmse'] for r in results if r['success'] and r['rmse'] > 0]
    drifts = [r['drift'] for r in results if r['success'] and r['drift'] > 0]

    print(f'  Success rate:     {n_pass}/{len(results)} ({rate:.0f}%)')
    if times: print(f'  Avg mission time: {np.mean(times):.1f}s ± {np.std(times):.1f}s')
    if rmses: print(f'  Avg RMSE:         {np.mean(rmses):.4f}m ± {np.std(rmses):.4f}m')
    if drifts: print(f'  Avg drift:        {np.mean(drifts):.2f}% ± {np.std(drifts):.2f}%')
    print(f'  Total collisions: {sum(r["collisions"] for r in results)}')
    print(f'  Total distance:   {sum(r["distance"] for r in results):.1f}m')

    repeat = [r['time'] for r in results if 'Repeat' in r['name'] and r['success']]
    if len(repeat) > 1:
        print(f'  Repeat CoV:       {np.std(repeat)/np.mean(repeat)*100:.1f}%')

    print()
    if rate >= 90:   print('  🎉  VERDICT: READY FOR PHASE 6 (HARDWARE)')
    elif rate >= 70: print('  ⚠️   VERDICT: NEEDS TUNING')
    else:            print('  ❌  VERDICT: NOT READY')
    print('=' * 65)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
