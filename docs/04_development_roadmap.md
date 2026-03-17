# Simulation-First Development Roadmap & MVP Build Plan

> **Version**: 1.0 | **Date**: 2026-03-09

## MVP Objective

A drone in simulation navigates a GPS-denied warehouse using camera + IMU, estimates pose via VIO, avoids static obstacles, reaches waypoints in sequence, and enters fail-safe hover if localization confidence drops.

---

## Engineering Build Order

### Phase 0 — Environment & Toolchain (Week 1)

| Step | Task | Depends On | Output | Test Criteria |
|------|------|-----------|--------|---------------|
| 0.1 | Install ROS2 Humble + Gazebo Harmonic | — | Working `ros2 topic list` | `ros2 doctor` passes |
| 0.2 | Install PX4 SITL + px4_ros_com bridge | 0.1 | PX4 running in Gazebo with ROS2 topics | `/fmu/out/vehicle_status` appears |
| 0.3 | Disable GPS in PX4 SITL params | 0.2 | PX4 reports NO_GPS but flies in MANUAL | EKF2 logs show no GPS fusion |
| 0.4 | Build Gazebo warehouse world | 0.1 | Shelving aisles, walls, open areas | Spawns without errors |
| 0.5 | Attach simulated D435i to drone model | 0.4 | RGB + depth + IMU topics publishing | `ros2 topic hz` confirms rates |
| 0.6 | Create colcon workspace with all packages | 0.1 | `colcon build` succeeds for all 9 pkgs | Zero build errors |

**Milestone M0**: PX4 drone spawns in warehouse, camera publishes images, no GPS.

**Debugging Checklist**:
- [ ] `ros2 topic list` shows all PX4 topics
- [ ] Gazebo loads world without SDF errors
- [ ] Camera frame rate matches config (30 Hz)
- [ ] PX4 EKF2 fusing baro + IMU only (no GPS)

---

### Phase 1 — State Estimation in Sim (Weeks 2–3)

| Step | Task | Depends On | Output | Test Criteria |
|------|------|-----------|--------|---------------|
| 1.1 | Integrate VINS-Fusion (stereo+IMU) in ROS2 | 0.5, 0.6 | `/state_estimation/odom` publishing | Topic active at 30 Hz |
| 1.2 | Feed VIO pose to PX4 EKF2 as ext. vision | 1.1 | PX4 fuses vision data, shows local position | EKF2 vision_position_innovation < 0.1m |
| 1.3 | Validate against Gazebo ground truth | 1.2 | Drift metrics: position error over time | < 1% drift over 100m travel |
| 1.4 | Implement confidence monitor | 1.1 | `/state_estimation/confidence` publishing | Score drops when features removed |
| 1.5 | Test in manual flight (RC in SITL) | 1.2 | Stable flight with VIO-only position hold | Hover drift < 0.3m over 30s |

**Milestone M1**: Drone holds position using VIO-only; drift < 1% over 100m.

**Expected Outputs**:
- VIO odometry topic at 30 Hz
- PX4 enters POSITION mode without GPS
- Ground truth comparison plots (position error vs. time)

**Debugging Checklist**:
- [ ] VINS-Fusion initializes (check `gravity aligned` log)
- [ ] Camera-IMU extrinsics match URDF/SDF
- [ ] IMU timestamps synchronized with images (< 1ms offset)
- [ ] PX4 EKF2 accepts vision (`estimator_status.control_mode_flags`)
- [ ] Check VIO output frame convention matches PX4 (NED vs ENU)

---

### Phase 2 — Mapping & Obstacle Detection (Week 4)

| Step | Task | Depends On | Output | Test Criteria |
|------|------|-----------|--------|---------------|
| 2.1 | Implement depth → point cloud pipeline | 0.5 | `/perception/pointcloud_filtered` | Cloud density 5k–50k pts |
| 2.2 | Integrate OctoMap with VIO pose | 1.1, 2.1 | `/mapping/octomap` updating live | Map grows as drone moves |
| 2.3 | Add obstacle inflation | 2.2 | Inflated map with safety margins | Inflation radius = 0.3m verified |
| 2.4 | Publish 2.5D occupancy grid | 2.2 | `/mapping/occupancy_grid` | Viewable in RViz2 |
| 2.5 | Edge case: textureless walls | 2.2 | Map handles sparse depth | No artifacts in OctoMap |

**Milestone M2**: Live 3D map in RViz2 reflecting obstacles; updates at 5 Hz.

**Debugging Checklist**:
- [ ] Point cloud is in correct frame (`base_link` or `camera_depth_optical_frame`)
- [ ] TF from `camera_depth_optical_frame` to `odom` is valid
- [ ] OctoMap resolution matches config (0.1m)
- [ ] Memory usage stable (rolling window discards old voxels)
- [ ] RViz2 shows map aligned with Gazebo world

---

### Phase 3 — Path Planning & Waypoint Nav (Weeks 5–6)

| Step | Task | Depends On | Output | Test Criteria |
|------|------|-----------|--------|---------------|
| 3.1 | Implement waypoint manager (YAML missions) | 0.6 | Loads mission, publishes goals | 3 WPs loaded, first goal published |
| 3.2 | Implement RRT* local planner | 2.2 | `/planning/local_path` collision-free | Path avoids all occupied voxels |
| 3.3 | Min-snap trajectory smoothing | 3.2 | `/planning/trajectory` smooth + feasible | Max accel < 1.5 m/s² |
| 3.4 | Implement trajectory tracker | 3.3 | PX4 setpoints at 50 Hz | Tracking error < 0.3m RMS |
| 3.5 | Offboard mode engagement protocol | 3.4 | Auto arm → offboard → fly | Clean mode transition |
| 3.6 | Arrival detection + WP advance | 3.1, 3.5 | Drone reaches WP1, advances to WP2 | Arrival within 0.5m tolerance |
| 3.7 | Obstacle avoidance test | 3.2 | Drone replans around shelf | Zero collisions in 10 runs |

**Milestone M3**: Drone autonomously flies 3-waypoint mission avoiding obstacles.

**Debugging Checklist**:
- [ ] Planner receives valid OctoMap (check topic connection)
- [ ] Planned path visualized in RViz2 overlaid on map
- [ ] Offboard heartbeat never drops (check 50 Hz loop)
- [ ] PX4 stays in OFFBOARD mode (check `vehicle_status`)
- [ ] Waypoint manager advances correctly (log sequence)

---

### Phase 4 — Safety & Fail-Safe (Week 7)

| Step | Task | Depends On | Output | Test Criteria |
|------|------|-----------|--------|---------------|
| 4.1 | Implement mission supervisor state machine | M3 | `/safety/system_status` reporting | All states reachable |
| 4.2 | Subsystem heartbeat monitoring | 4.1 | Detects node crash within 2s | Kill a node → supervisor detects |
| 4.3 | VIO confidence → hover trigger | 1.4, 4.1 | Hover when confidence < 0.1 | Cover camera → drone hovers |
| 4.4 | Battery RTL trigger | 4.1 | RTL at 25% battery | Sim battery drain → RTL |
| 4.5 | Geofence enforcement | 4.1 | Reject plans outside bounds | Waypoint outside fence → rejected |
| 4.6 | Emergency land on critical failure | 4.1 | Controlled descent | Kill VIO node → drone lands safely |

**Milestone M4**: Fail-safe hover on VIO loss, auto-land after 5s, battery RTL.

**Debugging Checklist**:
- [ ] State machine transitions logged (all edges tested)
- [ ] Supervisor never crashes (exception guards verified)
- [ ] Failsafe command latency < 100ms
- [ ] PX4 failsafe (RC Land) activates if supervisor dies
- [ ] Geofence bounds match world dimensions

---

### Phase 5 — Integration Testing & Metrics (Week 8)

| Step | Task | Depends On | Output | Test Criteria |
|------|------|-----------|--------|---------------|
| 5.1 | Full mission test (10 runs) | M4 | Success rate > 90% | Count successes |
| 5.2 | Rosbag recording + analysis | M4 | Bags for each run | Post-flight plots generated |
| 5.3 | Ground truth comparison suite | 1.3 | Automated RMSE reports | Position RMSE < 0.3m |
| 5.4 | Stress tests (low light, fog) | M4 | Degradation behavior verified | Cascade triggers correctly |
| 5.5 | Performance profiling (Jetson) | M4 | CPU/GPU/RAM histograms | Under budget per doc 03 |
| 5.6 | CI pipeline running | 0.6 | Build + test on every PR | GitHub Actions green |

**Milestone M5**: 95% mission success rate, documented performance profile, CI green.

---

### Phase 6 — Hardware-in-the-Loop (Weeks 9–10)

| Step | Task | Depends On | Output | Test Criteria |
|------|------|-----------|--------|---------------|
| 6.1 | Pixhawk 6C + Jetson Orin Nano wiring | M5 | Physical stack assembled | Serial comms verified |
| 6.2 | RealSense D435i calibration | 6.1 | Calibration YAML | Reprojection error < 0.5px |
| 6.3 | Bench test: VIO on real sensor | 6.2 | VIO odom on bench | Stable pose while stationary |
| 6.4 | Tethered hover test | 6.3 | Position hold on VIO | Hover drift < 0.5m over 30s |
| 6.5 | Free flight: single waypoint | 6.4 | Navigate to WP | Arrival < 0.5m |
| 6.6 | Free flight: full mission | 6.5 | 3-WP mission | Matches sim performance ± 2x |

**Milestone M6**: Real hardware completes 3-waypoint mission in structured indoor env.

---

## Go / No-Go Criteria Per Phase

| Phase | Go Criteria | No-Go Action |
|-------|-------------|--------------|
| M0→M1 | Camera + IMU publishing in sim | Debug Gazebo sensor plugins |
| M1→M2 | VIO drift < 1%, PX4 accepts vision | Re-calibrate, check frame conventions |
| M2→M3 | Map reflects real obstacles in RViz | Debug TF, check point cloud alignment |
| M3→M4 | 3-WP mission 80%+ success | Profile planner, tune controller gains |
| M4→M5 | All fail-safes trigger correctly | Review state machine edges |
| M5→M6 | 95% sim success, CI green | Do not proceed to hardware |

---

## Metrics Dashboard (Per Run)

| Metric | Source | Target |
|--------|--------|--------|
| Position RMSE vs ground truth | Ground truth comparator | < 0.3m |
| Max position error | Same | < 1.0m |
| VIO drift (% of distance) | Odometry analysis | < 1% |
| Tracking error RMS | Control node | < 0.3m |
| Replan count | Planner logs | < 20/mission |
| VIO tracking losses | Confidence monitor | 0 |
| Collisions | Gazebo contact sensor | 0 |
| Mission time | Bag timestamp | Within 2x of optimal |
| CPU peak utilization | System monitor | < 90% |
| Loop time (main pipeline) | Timing node | < 50ms P99 |
