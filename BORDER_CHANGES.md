# GPS-Denied Border Surveillance — Change Report

## What Changed and Why

The project pivoted from **indoor warehouse inspection** to **outdoor cross-border surveillance operations**. Every component was adapted for the fundamentally different operational environment.

---

## Side-by-Side Comparison

| Parameter | Before (Warehouse) | After (Border Ops) | Why |
|-----------|-------------------|-------------------|----|
| Operating range | 10m × 10m | 5km × 4km | Border patrol covers kilometers |
| Altitude | 1.5m | 60m AGL | Suburban terrain, wide FOV needed |
| Speed | 1.5 m/s | 8 m/s cruise, 15 m/s max | Must cover ground efficiently |
| Waypoint tolerance | 0.5m | 15m | GPS-precision not needed at range |
| Obstacles | Shelf units (0.5m wide) | Terrain features, structures | Scale difference |
| GPS denial cause | Being indoors | Jamming / spoofing | Active electronic warfare |
| Mission duration | ~45 seconds | ~15-20 minutes | Long-endurance patrol |
| Frame reference | Local odom (x/y/z) | NED (north/east/alt_agl) | Standard for outdoor aviation |
| Geofence shape | Square box (±10m) | Long corridor (5km × 4km) | Border is a line, not a box |
| Safety response | Hover + land | Hover + RTL + emergency land | Must be able to fly home |

---

## Files Changed — What Each Does Now

### 1. Mission File: `missions/border_patrol.yaml` [NEW]

**Before**: 3 waypoints at 2m/5m/0m over 10 seconds
**After**: 6 waypoints over 4.5km covering a border patrol sector

```
Launch (0,0) ──500m──▶ Patrol Start ──1500m──▶ Sweep West
                                                   │
     Return ◀──1500m── Turnaround ◀──1000m── Observation
  (fast, 80m)         (4500m north)          (loiter 60s)
```

**New concepts**:
- `action` field: `transit`, `surveillance_sweep`, `loiter_scan`, `rtl`
- `camera_mode`: `continuous_record`, `360_scan`, `thermal_scan`
- `alt_agl`: Altitude Above Ground Level (terrain following)
- `emergency_landing_sites`: Pre-surveyed safe landing locations
- `loiter_orbits`: Number of orbits for area scan waypoints

---

### 2. Waypoint Manager: `waypoint_manager.py` [REWRITTEN]

**Before**: Simple goto-waypoint sequencer
**After**: Full patrol mission manager with surveillance actions

**New capabilities**:
```
┌─────────────────────────────────────────────────────┐
│ Waypoint Manager (Border Ops)                       │
│                                                     │
│ ① Load mission with NED coordinates (km-scale)     │
│ ② Sequence waypoints with arrival detection (15m)   │
│ ③ Execute actions per waypoint:                     │
│    • "transit" → fly directly, no surveillance     │
│    • "surveillance_sweep" → camera ON, slow flight │
│    • "loiter_scan" → orbital flight, 360° scan    │
│    • "rtl" → fast return to launch                 │
│ ④ Switch camera modes at each waypoint              │
│ ⑤ Track emergency landing sites for safety          │
│ ⑥ Compute ETA and total mission distance           │
└─────────────────────────────────────────────────────┘
```

**Loiter orbit pattern** (new for border ops):
```
                ←────────────┐
                              │
          ┌──── loiter_radius=50m ────┐
          │                           │
    Drone orbits ● center      yaw → inward
          │                           │
          └───────────────────────────┘
                              │
                ─────────────→┘
```

The drone flies circular orbits around the waypoint, facing inward for surveillance. This is standard for ISR (Intelligence, Surveillance, Reconnaissance) operations.

---

### 3. Mission Supervisor: `mission_supervisor.py` [REWRITTEN]

**Before**: Simple MISSION → DEGRADED → HOVER → LAND
**After**: Full border operations safety system

**New state machine**:
```
                              GPS spoof detected
                              ──────────────────▶ CONTINUE (VIO-only)
                              
  MISSION ──▶ DEGRADED ──▶ HOVER ──▶ RTL ──▶ LAND ──▶ DISARMED
    ▲             │            ▲       ▲
    └─ recovery ──┘            │       │
                   comms loss ─┘       │
                   battery low ────────┘
                   geofence ───┘
```

**New safety features**:

| Feature | What It Does | Why It Matters |
|---------|-------------|----------------|
| **RTL (Return to Launch)** | Flies home at 80m altitude | Can't just hover forever at 4km range |
| **Battery-aware RTL** | Calculates fuel needed to return, triggers early if insufficient | Prevents dead-stick landing in hostile territory |
| **Comms loss protocol** | If no GCS heartbeat for 30s → auto RTL | Communication failure is common in contested environments |
| **GPS spoofing detection** | Detects position jumps >50m → ignores GPS, relies on VIO | Active threat in border scenarios |
| **Corridor geofence** | 5km × 4km corridor instead of square box | Border operations follow a linear corridor |
| **Altitude floor** | Minimum 30m AGL enforced | Terrain collision avoidance |
| **Emergency landing sites** | Pre-surveyed safe landing locations | Can't land just anywhere in border terrain |

---

### 4. Terrain Matcher: `terrain_matcher.py` [NEW]

**Why it's needed**: VIO drifts ~0.5% per km. Over a 4.5km patrol, that's 22.5m of drift. Terrain matching corrects this.

**How it works**:
```
Step 1: Pre-load terrain database
        (satellite images + GPS coordinates of features)
        │
Step 2: Every 5 seconds, capture downward camera image
        │
Step 3: Extract ORB features from live image
        │
Step 4: Match against terrain database (BFMatcher)
        │
Step 5: If match confidence > 50%:
        │   correction = database_position - VIO_position
        │   Apply 30% of correction (smooth, not jerky)
        │
Step 6: Publish corrected position for EKF fusion
```

**Terrain database** (10 reference points along the route):
```
Launch ─── Road ─── Clearing ─── River ─── Structure
  0km      0.5km     1.0km      1.5km      2.0km
                                              │
Turnaround ── Forest ── Observation ── Road ──┘
  4.5km       3.5km      3.0km       2.5km
```

---

### 5. Surveillance Manager: `surveillance_manager.py` [NEW]

**What it does**: Manages the camera payload for ISR operations.

```
Camera Modes:
  standby           → Camera off, saving power
  continuous_record → Recording video, running detection
  360_scan          → Full rotation scan during loiter
  thermal_scan      → Thermal/IR camera for heat signatures

When target detected:
  ① Geo-tag: position (NED) + timestamp + altitude
  ② Classify: person / vehicle / structure  
  ③ Log: JSONL file to /workspace/recordings/
  ④ Alert: Publish to /surveillance/alert topic
```

**Detection log format** (saved per session):
```json
{
  "timestamp": 1712345678,
  "class": "vehicle",
  "confidence": 0.87,
  "position_ned": [3000.0, -100.0, 60.0],
  "altitude_agl": 60.0,
  "camera_mode": "continuous_record",
  "session_id": 1712345600
}
```

---

### 6. System Parameters: `config/system_params.yaml` [REWRITTEN]

Every parameter section was retuned for outdoor operations:

| Section | Key Change | Before → After |
|---------|-----------|----------------|
| **Perception** | Depth range | 5m → 30m |
| **Perception** | Voxel size | 0.05m → 0.5m |
| **State estimation** | Feature threshold | 30 → 20 (more features outdoors) |
| **State estimation** | Relocalization timeout | 5s → 10s |
| **Mapping** | Resolution | 0.1m → 1.0m |
| **Mapping** | Local map radius | 8m → 200m |
| **Planning** | Planning horizon | 5m → 500m |
| **Planning** | Goal tolerance | 0.3m → 15m |
| **Planning** | Max velocity | 2 m/s → 15 m/s |
| **Control** | Tracking error limit | 1.0m → 10.0m |
| **Safety** | Recovery timeout | 5s → 15s |
| **Safety** | Heartbeat timeout | 2s → 3s |
| **NEW** | GPS spoof detection | — | Position jump >50m = spoofing |
| **NEW** | Comms loss timeout | — | 30s → RTL |
| **NEW** | Terrain matching | — | Every 5s, based on satellite imagery |
| **NEW** | Surveillance | — | YOLOv8n detection, thermal support |

---

## How the Full System Works Now

```
                    BORDER SURVEILLANCE PIPELINE
                    ════════════════════════════

Pre-mission:
  Ground crew loads:
    ├── Border patrol YAML (waypoints, geofence, emergency sites)
    ├── Terrain database (satellite imagery for position correction)
    └── Detection model (YOLOv8n for person/vehicle classification)

In-flight (fully autonomous):
  ┌─────────────────────────────────────────────────────────┐
  │                                                         │
  │  Camera ──▶ VIO (position) ──┐                         │
  │                               ├──▶ EKF ──▶ Pilot       │
  │  Terrain DB ──▶ Matcher ────┘    (fused    (waypoint    │
  │  (drift correction)              pose)     manager)    │
  │                                     │           │       │
  │                                     ▼           ▼       │
  │  Depth Camera ──▶ 3D Map ──▶ RRT* Planner ──▶ PX4     │
  │  (obstacles)    (1m voxels)  (500m horizon)  (offboard)│
  │                                                         │
  │  Surveillance ──▶ Detection ──▶ Geo-tag ──▶ Alert      │
  │  Camera          (YOLOv8n)    (JSONL log)  (to GCS)    │
  │                                                         │
  │  Safety Supervisor (5 Hz watchdog):                     │
  │    ├── VIO confidence monitoring                        │
  │    ├── Battery vs. distance-to-home calculation        │
  │    ├── Communication loss detection (30s timeout)       │
  │    ├── GPS spoofing detection (position jump >50m)     │
  │    ├── Corridor geofence enforcement                    │
  │    └── Altitude floor enforcement (≥30m AGL)           │
  └─────────────────────────────────────────────────────────┘
```

---

## Node Count

| Package | Nodes | New/Modified |
|---------|-------|-------------|
| `gps_denied_state_estimation` | vio_node, px4_vision_bridge, ground_truth_comparator, **terrain_matcher** | +1 new |
| `gps_denied_perception` | depth_processor, feature_extractor, obstacle_detector, **surveillance_manager** | +1 new |
| `gps_denied_mapping` | octomap_builder, map_manager | unchanged |
| `gps_denied_planning` | **waypoint_manager**, local_planner | 1 rewritten |
| `gps_denied_control` | trajectory_tracker, px4_commander | unchanged |
| `gps_denied_safety` | **mission_supervisor**, safety_monitor | 1 rewritten |
| `gps_denied_bringup` | telemetry_aggregator | unchanged |
| **Total** | **17 nodes** | 4 modified, 2 new |

---

## What "GPS-Denied" Means in Border Context

In indoor operations, GPS denial is passive (signals can't penetrate).
In border operations, GPS denial is **active and adversarial**:

```
Threat              What Happens              Our Response
────────            ────────────              ────────────
GPS Jamming         No GPS signal at all      VIO-only navigation
                                              (already our default)

GPS Spoofing        False GPS position        Detect via:
                    (enemy feeds fake          • Position jumps >50m
                    coordinates)               • VIO vs GPS velocity mismatch
                                              → Ignore GPS, trust VIO

Signal Intelligence Enemy detects our         Stealth mode:
                    radio emissions            • Terrain masking
                                              • Low altitude (50m)
                                              • Minimize transmissions

Electronic Warfare  Broader spectrum          INS dead reckoning
                    jamming (GPS+comms)        + terrain matching
                                              as backup navigation
```

---

## Summary of All Changes

| # | File | Change Type | What Changed |
|---|------|------------|--------------|
| 1 | `missions/border_patrol.yaml` | **NEW** | 6-WP 4.5km patrol with surveillance actions |
| 2 | `config/system_params.yaml` | **REWRITTEN** | All parameters tuned for outdoor/border ops |
| 3 | `waypoint_manager.py` | **REWRITTEN** | NED frame, loiter orbits, camera modes, ETA |
| 4 | `mission_supervisor.py` | **REWRITTEN** | Corridor geofence, RTL, comms loss, GPS spoof |
| 5 | `terrain_matcher.py` | **NEW** | VIO drift correction via terrain feature matching |
| 6 | `surveillance_manager.py` | **NEW** | Target detection, recording, geo-tagging |
| 7 | `setup.py` (estimation) | **MODIFIED** | Added terrain_matcher entry point |
| 8 | `setup.py` (perception) | **MODIFIED** | Added surveillance_manager entry point |
| 9 | `PROJECT_REPORT.md` | **NEW** | Comprehensive project documentation |
