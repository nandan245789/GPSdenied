# GPS-Denied Border Surveillance — Codex Handoff Document

> **COMPLETE CONTEXT** for an AI coding agent to continue development of this project.
> Read this ENTIRE document before writing any code.

---

## 1. WHAT THIS PROJECT IS

An autonomous drone navigation stack for **GPS-denied border surveillance** using visual-inertial odometry (VIO) and terrain-aided navigation. The drone operates in environments where GPS is jammed or spoofed (cross-border/military scenarios), using onboard cameras and IMU for all navigation.

**Application:** Outdoor suburban / cross-border surveillance patrol (4.5km route at 60m altitude above ground level).

**Hardware target:** 450mm quadrotor with:
- Pixhawk 6C autopilot (PX4 v1.15 firmware)
- Jetson Orin Nano compute module (8GB unified RAM)
- Intel RealSense D435i (stereo depth + RGB + IMU)
- Downward-facing camera for terrain matching

**Dev machine:** MacBook Air M2, 16GB RAM, 512GB SSD, macOS.

---

## 2. REPOSITORY

```
URL:    https://github.com/nandan245789/GPSdenied.git
Branch: main (only branch)
HEAD:   f2bf223 — Phase 5d: Georeferenced terrain database + real ORB terrain matcher
```

### Full Commit History (chronological)
```
d1890cb  Initial scaffold: GPS-denied vision navigation stack
fe4cf28  Phase 0: Docker simulation environment setup
4d28113  Phase 0 complete: Docker sim environment builds and verifies
0b25a47  Phase 1: Implement VIO, PX4 bridge, ground truth comparator
6b96227  Phase 2+3: Mapping, Planning, and Control implemented
94cc2ac  Working autonomous mission demo — 3-waypoint flight in 45.7s
6588f0c  Phase 4: Safety fail-safe state machine — 5/5 tests pass
f79b772  Phase 5: Integration tests — 7/7 pass (100%), READY FOR HARDWARE
1e38585  PIVOT: Indoor warehouse → Outdoor border surveillance
1898873  docs: Add border surveillance change report
933d7da  Phase 5b: Border patrol demo simulation — 4.5km mission verified
699ba40  Phase 5c: Terrain feature diversity analysis — 1,175 DJI frames scored 71/100 GOOD
f2bf223  Phase 5d: Georeferenced terrain database + real ORB terrain matcher  ← CURRENT
```

---

## 3. DIRECTORY STRUCTURE

```
GPSdenied/
├── config/
│   └── system_params.yaml          # ALL tunable parameters (perception, planning, safety, surveillance)
├── data/
│   ├── terrain_db.pkl              # 72.9 MB — georeferenced ORB terrain database (502 frames, 1,004,000 descriptors)
│   └── terrain_db_meta.json        # Lightweight metadata (no descriptors)
├── docker/
│   ├── Dockerfile                  # ROS2 Humble + Gazebo Harmonic + PX4 v1.15 SITL
│   ├── docker-compose.yml          # Services: dev (shell), sim (full SITL)
│   └── entrypoint.sh               # Sources ROS2 + PX4 + workspace
├── frames_nadir/                   # 502 top-down DJI aerial frames (5472×3648, GPS-tagged)
│   ├── topdownSET1/ (157 JPGs)
│   ├── topdownSET2/ (170 JPGs)
│   └── topdownSET3/ (175 JPGs)
├── frames_oblique/                 # 673 oblique (45°) DJI aerial frames (same resolution)
│   ├── 3d 60M/   (156 JPGs)
│   ├── 3d 60M-1/ (180 JPGs)
│   ├── 3d 60M-2/ (155 JPGs)
│   └── 3d 60M-3/ (182 JPGs)
├── missions/
│   ├── border_patrol.yaml          # 6-waypoint 4.5km patrol (NED coords, 60m AGL)
│   └── warehouse_inspection.yaml   # Legacy indoor mission (not used)
├── scripts/
│   ├── analyze_terrain_features.py # Analyzes all 1,175 frames for ORB feature density, terrain type
│   ├── build_terrain_db.py         # Builds terrain_db.pkl from nadir frames
│   ├── run_border_demo.py          # Full 4.5km mission simulation (no Gazebo needed)
│   ├── run_demo.py                 # Legacy indoor demo
│   ├── run_integration_tests.py    # 7 integration tests (all passing)
│   ├── test_safety.py              # 5 safety tests (all passing)
│   ├── launch_sim.sh               # Starts PX4 SITL + Gazebo + DDS agent (inside Docker)
│   └── verify_sim.sh               # Verifies Docker sim environment
├── src/                            # 9 ROS2 packages (Python)
│   ├── gps_denied_bringup/         # Launch files, telemetry aggregator
│   ├── gps_denied_control/         # PX4 commander, trajectory tracker
│   ├── gps_denied_interfaces/      # Custom ROS2 message definitions
│   ├── gps_denied_mapping/         # OctoMap builder, map manager
│   ├── gps_denied_perception/      # Feature extraction, depth processing, obstacle detection, surveillance
│   ├── gps_denied_planning/        # Waypoint manager, local planner (RRT*)
│   ├── gps_denied_safety/          # Mission supervisor (state machine), safety monitor
│   ├── gps_denied_sim/             # Simulation utilities
│   └── gps_denied_state_estimation/ # VIO node, PX4 vision bridge, terrain matcher, ground truth
├── BORDER_CHANGES.md               # Documents the indoor→outdoor pivot
├── PROJECT_REPORT.md               # Formal project report
└── terrain_analysis_results.json   # Output from analyze_terrain_features.py
```

---

## 4. TECH STACK

| Component | Technology | Version |
|-----------|-----------|---------|
| **Middleware** | ROS2 | Humble (Ubuntu 22.04) |
| **Autopilot** | PX4 | v1.15.2 |
| **Simulator** | Gazebo | Harmonic |
| **PX4↔ROS2 bridge** | Micro-XRCE-DDS | Latest |
| **Language** | Python | 3.10+ |
| **Computer Vision** | OpenCV | 4.x (opencv-python-headless) |
| **Object Detection** | YOLOv8n | ultralytics (not yet fine-tuned) |
| **Containerization** | Docker | Docker Desktop (Apple Silicon) |
| **Drone model (SITL)** | x500 | PX4 gz_x500 |

---

## 5. MODELS & ALGORITHMS — WHAT RUNS WHERE

### ACTIVE NOW (Classical CV — all CPU, no training needed)

| Algorithm | File | Purpose | Library |
|-----------|------|---------|---------|
| **ORB** (Oriented FAST + Rotated BRIEF) | `vio_node.py`, `terrain_matcher.py` | Feature extraction for VIO + terrain matching | OpenCV `cv2.ORB_create(nfeatures=2000)` |
| **KLT** (Kanade-Lucas-Tomasi) | `vio_node.py` | Optical flow for frame-to-frame tracking | OpenCV `cv2.calcOpticalFlowPyrLK()` |
| **BFMatcher** (Brute Force Hamming) | `terrain_matcher.py` | Match live ORB descriptors against terrain DB | OpenCV `cv2.BFMatcher(cv2.NORM_HAMMING)` |
| **RRT*** | `local_planner.py` | Path planning with obstacle avoidance | Custom implementation |
| **Minimum Snap** | `trajectory_tracker.py` | Trajectory smoothing | Custom implementation |

### CONFIGURED BUT NOT YET INTEGRATED (Neural Network — GPU)

| Model | File | Purpose | Size | VRAM |
|-------|------|---------|------|------|
| **YOLOv8n** | `surveillance_manager.py` | Detect person/vehicle/structure in camera feed | 3.2M params, 6 MB weights | Training: 4-8 GB, Inference: ~1 GB |

> **CRITICAL:** The entire navigation loop (VIO → terrain matching → planning → control → safety) is classical CV. YOLOv8n is ONLY for the surveillance payload — it detects targets but has ZERO impact on navigation. The drone flies safely without any neural network.

---

## 6. KEY FILES — WHAT EACH ONE DOES

### State Estimation (`src/gps_denied_state_estimation/`)

| File | Lines | What It Does |
|------|-------|--------------|
| `vio_node.py` | ~200 | Visual-Inertial Odometry: subscribes to camera+IMU, publishes odometry using ORB+KLT tracking. Publishes to `/state_estimation/odom` and `/state_estimation/confidence`. |
| `terrain_matcher.py` | ~380 | **Terrain-aided position correction.** Loads `terrain_db.pkl` (1M ORB descriptors from real DJI frames). In REAL mode: matches live camera against DB using BFMatcher + Lowe's ratio test. In SIMULATION mode: uses spatial proximity. Publishes corrections to `/terrain/position_correction`. Has spatial index (10m grid cells) for fast lookup. |
| `px4_vision_bridge.py` | ~150 | Bridges VIO output to PX4's vision estimate input (MAVLink VISION_POSITION_ESTIMATE). |
| `ground_truth_comparator.py` | ~100 | Compares VIO estimate vs Gazebo ground truth, computes RMSE. |

### Perception (`src/gps_denied_perception/`)

| File | Lines | What It Does |
|------|-------|--------------|
| `feature_extractor.py` | ~180 | ORB feature extraction from forward camera, publishes feature count for health monitoring. |
| `depth_processor.py` | ~150 | Processes RealSense depth images into point clouds. |
| `obstacle_detector.py` | ~170 | Detects nearby obstacles from depth data, publishes obstacle positions. |
| `surveillance_manager.py` | ~216 | **Surveillance payload controller.** Manages camera modes (standby/continuous_record/360_scan/thermal_scan). Geo-tags detections with NED position. Saves detection log to JSONL. Currently SIMULATES detections — needs YOLOv8n integration. |

### Planning (`src/gps_denied_planning/`)

| File | Lines | What It Does |
|------|-------|--------------|
| `waypoint_manager.py` | ~195 | Sequences through mission waypoints (from border_patrol.yaml). Handles transit, surveillance_sweep, loiter_scan, RTL actions. Publishes camera commands per waypoint. Supports loiter orbits. |
| `local_planner.py` | ~200 | RRT* obstacle-avoidance planner. Takes current position + goal → outputs collision-free path. Publishes setpoints to `/control/position_setpoint`. |

### Control (`src/gps_denied_control/`)

| File | Lines | What It Does |
|------|-------|--------------|
| `px4_commander.py` | ~200 | PX4 offboard mode commander. Arms drone, switches to offboard, sends position setpoints at 10Hz. Handles failsafe (disarm on timeout). |
| `trajectory_tracker.py` | ~180 | Minimum-snap trajectory tracking. Smooths discrete waypoints into continuous trajectories. |

### Safety (`src/gps_denied_safety/`)

| File | Lines | What It Does |
|------|-------|--------------|
| `mission_supervisor.py` | ~332 | **THE SAFETY BRAIN.** State machine: NOMINAL → CAUTION → CRITICAL → EMERGENCY → RTL → LAND. Monitors: VIO confidence, battery %, comms heartbeat, GPS spoofing (position jumps + velocity mismatch), corridor geofence (5km×4km NED box). Auto-triggers RTL on comms loss (30s), low battery (20%), or geofence violation. |
| `safety_monitor.py` | ~150 | Low-level safety checks: motor status, IMU health, temperature. |

### Bringup (`src/gps_denied_bringup/`)

| File | What It Does |
|------|--------------|
| `launch/full_system.launch.py` | Launches all nodes for real flight |
| `launch/autonomous_mission.launch.py` | Launches autonomous mission with waypoints |
| `launch/test_state_estimation.launch.py` | Test launch for VIO only |
| `telemetry_aggregator.py` | Aggregates all telemetry into single status message |

---

## 7. ROS2 TOPIC MAP

```
/state_estimation/odom              [Odometry]       — VIO output (position + velocity)
/state_estimation/pose              [PoseStamped]    — VIO pose only
/state_estimation/confidence        [Float32]        — VIO tracking confidence (0-1)

/terrain/position_correction        [PoseWithCovarianceStamped] — Terrain matching correction
/terrain/match_confidence           [Float32]        — Terrain match quality
/terrain/status                     [String]         — Terrain matcher status

/planning/current_waypoint          [PoseStamped]    — Current target waypoint
/planning/current_action            [String]         — Current action (transit/sweep/loiter/rtl)
/control/position_setpoint          [PoseStamped]    — Commanded position to autopilot

/safety/state_machine               [String]         — Current safety state
/safety/alert                       [String]         — Safety alerts and warnings

/surveillance/camera_command        [String]         — Camera mode commands
/surveillance/detections            [String]         — JSON-encoded detection data
/surveillance/alert                 [String]         — Detection alerts
/surveillance/recording             [Bool]           — Recording state

/sensors/battery_pct                [Float32]        — Battery percentage
/comms/heartbeat                    [Bool]           — Ground station heartbeat

/mapping/occupancy_grid             [OccupancyGrid]  — Local obstacle map
/model/x500/pose                    [PoseStamped]    — Ground truth (from Gazebo)
```

---

## 8. TERRAIN DATABASE — THE CORE DATA ASSET

```
File:         data/terrain_db.pkl (72.9 MB, Python pickle)
Frames:       502 nadir DJI images at 60m AGL, 5472×3648 resolution
Descriptors:  1,004,000 ORB descriptors (2000 per frame)
Coverage:     561m (N-S) × 506m (E-W) = 28.4 hectares
Origin:       15.832380°N, 74.402084°E, 831.9m ASL (centroid)
GSD:          3.2 cm/pixel
Spatial Index: 403 cells, 10m × 10m grid
Accuracy:     ±16cm estimated matching accuracy

Terrain types: barren_soil (376), mixed_suburban (102), mixed (19), urban (1), dense_vegetation (4)

DB structure (pickle dict):
  db['version']           → '1.0'
  db['origin']            → {'lat', 'lon', 'alt', 'description'}
  db['camera']            → {'sensor_mm', 'focal_mm', 'resolution', 'gsd_cm', 'footprint_m'}
  db['coverage']          → {'n_frames', 'north_range', 'east_range', 'area_m2', 'terrain_distribution'}
  db['entries']           → list of 502 dicts, each with:
      entry['frame_id']
      entry['filename']
      entry['set_name']
      entry['gps']             → {'lat', 'lon', 'alt'}
      entry['ned']             → {'north', 'east', 'down'}  (meters from origin)
      entry['footprint']       → {'width_m', 'height_m', 'gsd_cm'}
      entry['features']        → {'n_keypoints', 'descriptors' (uint8 [N×32]), 'positions' (float32 [N×2])}
      entry['quality']         → {'texture_score', 'terrain_type'}
  db['spatial_index']     → dict {(cell_n, cell_e): [entry_indices]}  (10m grid)
  db['flann']             → {'descriptors' (uint8 [1004000×32]), 'desc_to_frame' (int32 [1004000])}
```

---

## 9. CONFIGURATION — system_params.yaml

All flight parameters live in `config/system_params.yaml`. Key values:

```yaml
# Perception
perception.max_features: 800
perception.max_depth_range: 30.0

# State Estimation
state_estimation.terrain_matching_enabled: true
state_estimation.terrain_match_interval: 5.0
state_estimation.gps_spoof_detection: true
state_estimation.gps_position_jump_thresh: 50.0

# Planning
planning.cruise_velocity: 8.0        # m/s
planning.surveillance_velocity: 5.0  # m/s
planning.goal_tolerance: 15.0        # meters
planning.loiter_radius: 50.0         # meters

# Safety
safety.vio_confidence_critical: 0.1
safety.battery_critical_pct: 20.0
safety.comms_loss_timeout: 30.0
safety.geofence_north_max: 5000.0
safety.geofence_east_max: 2000.0

# Surveillance
surveillance.detection_model: "yolov8n"
surveillance.detection_classes: ["person", "vehicle", "structure"]
```

---

## 10. WHAT HAS BEEN COMPLETED

| Phase | Description | Status |
|-------|-------------|--------|
| 0 | Docker environment (ROS2 + Gazebo + PX4) | ✅ |
| 1 | VIO node + PX4 bridge + ground truth comparator | ✅ |
| 2-3 | OctoMap mapping + RRT* planning + PX4 control | ✅ |
| 4 | Safety state machine (5/5 tests pass) | ✅ |
| 5 | Integration tests (7/7 pass) | ✅ |
| 5a | Indoor→outdoor pivot (all params retuned) | ✅ |
| 5b | Border patrol simulation (4.5km verified) | ✅ |
| 5c | Dataset analysis (1,175 frames, viability=71/100 GOOD) | ✅ |
| 5d | Terrain DB built (1M descriptors) + terrain_matcher rewritten | ✅ |

---

## 11. WHAT NEEDS TO BE DONE NEXT

### Phase 6a: VIO Accuracy Test (oblique frames)
- Feed the 673 oblique frames sequentially as simulated VIO input
- Measure position drift over the ~530m flight track vs GPS ground truth from EXIF
- Establish baseline drift rate (meters per km)
- **Input:** `frames_oblique/` (673 frames with GPS EXIF)
- **Output:** Drift rate measurement, accuracy metrics

### Phase 6b: Cross-Validate Terrain Matching
- Take random oblique frames → extract ORB → match against nadir terrain DB
- Verify that terrain_matcher produces correct NED position corrections
- Measure matching accuracy: how close is the matched position to GPS ground truth?
- **Input:** `frames_oblique/` + `data/terrain_db.pkl`
- **Output:** Matching accuracy metrics (meters of error)

### Phase 6c: Gazebo SITL Integration
- Build the Docker image: `cd docker && docker compose build sim`
- Start PX4 SITL + Gazebo: `docker compose up sim`
- Verify PX4 topics appear in ROS2
- Test single-node offboard control (arm → takeoff → hover → land)
- **NOTE:** Gazebo Harmonic is ALREADY in the Dockerfile but has never been run yet
- **DOCKER PATH on host Mac:** `/Applications/Docker.app/Contents/Resources/bin/docker`

### Phase 6d: Full-Stack SITL Mission
- Launch all 9 ROS2 packages inside Docker alongside PX4 SITL
- Run the full border_patrol.yaml mission in Gazebo
- Verify waypoint sequencing, loiter orbits, RTL, geofence, battery management
- Connect Gazebo camera topic to terrain_matcher to test real-time matching

### Phase 6e: YOLOv8n Fine-Tuning
- **Prerequisite:** Annotate ~300-500 DJI frames with bounding boxes (person, vehicle, structure, path)
- Use Roboflow or CVAT for annotation (browser-based, no GPU needed)
- Train: `yolo train model=yolov8n.pt data=border.yaml epochs=100 imgsz=640 batch=16 device=mps`
- Can train locally on M2 Mac (~2 hours) or on RunPod RTX 4090 (~45 min, ~$0.35)
- Export for Jetson: `yolo export model=best.pt format=engine`
- Integrate into `surveillance_manager.py` to replace simulated detections

### Phase 6f: Integrate YOLOv8n into Surveillance Manager
- Modify `surveillance_manager.py` to:
  - Subscribe to `/camera/forward/image_raw`
  - Run YOLOv8n inference on each frame
  - Replace the current simulated detection with real inference
  - Publish detections with bounding boxes, class, confidence, geo-tag

### Phase 7: Hardware Integration
- Flash PX4 on Pixhawk 6C
- Set up Jetson Orin Nano (JetPack + ROS2 Humble)
- Calibrate RealSense D435i
- Deploy ROS2 nodes to Jetson
- Bench test (all nodes running, motors off)
- Tethered hover flight test

### Phase 8: Field Testing
- Short-range VIO accuracy test (100m, GPS for ground truth)
- Terrain matching validation (fly over surveyed area)
- Full 4.5km patrol mission (GPS-denied)

---

## 12. HOW TO BUILD AND RUN

### Local (no Docker, no Gazebo)
```bash
# Install OpenCV
pip3 install opencv-python-headless numpy

# Run terrain analysis
python3 scripts/analyze_terrain_features.py

# Build terrain database
python3 scripts/build_terrain_db.py

# Run border patrol simulation (pure Python, no ROS2 needed for logic test)
# This requires ROS2 — see Docker method below
python3 scripts/run_border_demo.py
```

### Docker (full ROS2 + Gazebo + PX4)
```bash
# On macOS, Docker binary is at:
export PATH="/Applications/Docker.app/Contents/Resources/bin:$PATH"

# Build
cd docker
docker compose build dev

# Interactive shell
docker compose run dev bash

# Inside container:
colcon build --symlink-install
source install/setup.bash

# Run simulation
bash scripts/launch_sim.sh
# In another terminal:
ros2 launch gps_denied_bringup autonomous_mission.launch.py
```

### Run Tests
```bash
# Inside Docker or ROS2 environment:
python3 scripts/run_integration_tests.py    # 7 tests
python3 scripts/test_safety.py              # 5 tests
python3 scripts/run_border_demo.py          # Full mission sim
```

---

## 13. DESIGN DECISIONS & CONSTRAINTS

1. **Classical CV over Deep Learning for navigation:** ORB+KLT was chosen over SuperPoint/SuperGlue because it runs on CPU with zero VRAM, critical for the power-constrained Jetson Orin Nano where GPU is reserved for YOLO inference.

2. **Terrain matching over GPS:** The entire navigation concept assumes GPS is jammed/spoofed. VIO provides relative position, terrain matching provides absolute position corrections by comparing live downward camera against the pre-surveyed terrain database.

3. **Dual-mode terrain_matcher:** `terrain_matcher.py` auto-detects whether `terrain_db.pkl` and `cv2` are available. If yes → REAL mode (ORB matching). If no → SIMULATION mode (spatial proximity). This lets the same code run in pure simulation and on hardware.

4. **NED coordinate frame:** All positions are in North-East-Down (NED) meters relative to the launch point. GPS coordinates are only used during terrain DB construction and for ground truth comparison.

5. **Surveillance is decoupled from navigation:** The drone navigates purely on VIO + terrain matching. YOLOv8n is an independent payload. If YOLO crashes, the drone continues flying safely.

6. **Safety-first design:** The mission supervisor has a one-way escalation: NOMINAL → CAUTION → CRITICAL → EMERGENCY → RTL → LAND. It cannot downgrade from EMERGENCY. Comms loss always triggers RTL.

7. **The DJI frames are not tracked in git.** They are 13.2 GB total (5.8 GB nadir + 7.4 GB oblique). Only `terrain_db.pkl` (72.9 MB, derived from nadir frames) and `terrain_analysis_results.json` are in the repo.

---

## 14. KNOWN ISSUES & GOTCHAS

1. **GitHub large file warning:** `terrain_db.pkl` is 72.9 MB. GitHub warns about files >50 MB. Consider Git LFS if the DB grows.

2. **Docker on Apple Silicon:** The Docker image targets `ros:humble-ros-base-jammy` which is amd64. On M2 Mac, Docker Desktop runs it via Rosetta 2 emulation — expect 2-3× slower than native.

3. **Gazebo rendering:** Gazebo Harmonic requires GPU for 3D rendering. On Mac inside Docker, it must run headless (`DISPLAY` not set). Use `rviz2` or topic echo for debugging.

4. **Frame spacing in dataset:** Adjacent DJI frames are spaced 5-10m apart with only ~1% ORB overlap between sequential frames. This is fine for terrain DB matching (live frame vs stored DB frame) but NOT sufficient for frame-to-frame VIO. VIO testing needs the oblique sequence where overlap is higher.

5. **Seasonal terrain variation:** 62% of the terrain is laterite soil with vegetation that changes dramatically in monsoon season (June-Sept). Terrain DB may need seasonal variants.

6. **surveillance_manager.py currently SIMULATES detections.** It does not actually run YOLOv8n. The function `report_detection()` is called manually from the simulation script. Real YOLO integration is Phase 6e/6f.

7. **The `warehouse_inspection.yaml` mission is legacy** from when the project was indoor. It is not used. `border_patrol.yaml` is the active mission.

---

## 15. IMPORTANT FILE PATHS

```
Terrain DB:          data/terrain_db.pkl
Terrain metadata:    data/terrain_db_meta.json
System config:       config/system_params.yaml
Mission definition:  missions/border_patrol.yaml
Docker setup:        docker/Dockerfile, docker/docker-compose.yml
Sim launch:          scripts/launch_sim.sh
Border demo:         scripts/run_border_demo.py
Terrain matcher:     src/gps_denied_state_estimation/gps_denied_state_estimation/terrain_matcher.py
Surveillance:        src/gps_denied_perception/gps_denied_perception/surveillance_manager.py
Safety:              src/gps_denied_safety/gps_denied_safety/mission_supervisor.py
Waypoints:           src/gps_denied_planning/gps_denied_planning/waypoint_manager.py
VIO:                 src/gps_denied_state_estimation/gps_denied_state_estimation/vio_node.py
```
