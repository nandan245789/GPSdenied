# Mission Framing Template — GPS-Denied Vision Navigation

> **Version**: 1.0  
> **Date**: 2026-03-09  
> **Status**: DRAFT  

---

## 1. Mission Objective

**Primary**: Autonomously navigate a quadrotor UAV through GPS-denied indoor/urban environments using vision-based localization, achieving waypoint-to-waypoint flight with obstacle avoidance.

**Secondary**:
- Build a reusable, modular autonomy stack deployable across environment classes
- Validate all subsystems in simulation before any hardware flight
- Establish a continuous integration pipeline for regression-safe iteration

---

## 2. Operational Concept (CONOPS)

### 2.1 Scenario Description

| Parameter | Value |
|-----------|-------|
| Environment | Indoor warehouse, tunnel, urban canyon |
| GPS availability | **None** — fully denied |
| Lighting | Artificial (warehouse), variable (tunnel entry/exit), ambient (urban) |
| Dynamic obstacles | Possible (humans, forklifts) — not in MVP |
| Mission duration | 5–10 minutes per sortie |
| Operator presence | Ground station within radio link range |

### 2.2 Mission Profile

```
[ARM] → [TAKEOFF to 1.5m AGL]
      → [TRANSITION to OFFBOARD]
      → [NAVIGATE waypoint sequence]
      → [RETURN to launch / LAND at final WP]
      → [DISARM]
```

### 2.3 Operator Interaction Model

| Phase | Operator Role | Autonomy Level |
|-------|---------------|----------------|
| Pre-flight | Load mission, verify health checks | Manual |
| Takeoff | Initiate via GCS or RC | Semi-auto |
| En-route | Monitor, can intervene | Full auto |
| Landing | Auto-land or operator takeover | Semi-auto |
| Emergency | System auto-RTL; operator can override | Supervised auto |

---

## 3. System Boundaries & Constraints

### 3.1 Platform Constraints

| Constraint | Value | Rationale |
|------------|-------|-----------|
| Airframe | 450mm quadrotor | Payload capacity, stability |
| AUW | ≤ 2.5 kg | Regulatory + flight time |
| Max speed | 2 m/s (MVP) | VIO tracking reliability |
| Max altitude AGL | 5 m (indoor) | Ceiling clearance |
| Payload | Jetson Orin Nano + D435i | 180g compute + 72g sensor |
| Flight time | ~10 min with payload | 4S 5200mAh LiPo |
| Comms | 2.4 GHz RC + 5.8 GHz telemetry | Standard PX4 links |

### 3.2 Environmental Constraints

| Constraint | Mitigation |
|------------|------------|
| Textureless surfaces (white walls) | Ensure min feature count threshold; degrade to IMU-only hover |
| Low light (< 50 lux) | IR projector on D435i; adaptive exposure |
| Dust/fog | Depth filter outlier rejection; reduce confidence |
| Narrow passages (< 1.5m) | Conservative safety margins in planner |
| Dynamic obstacles | MVP: static only. Phase 2: dynamic replanning |

### 3.3 Regulatory Considerations

| Requirement | Status |
|-------------|--------|
| DGCA registration | Required if > 250g AUW |
| Indoor flight exemption | Generally exempt from airspace rules |
| Observer requirement | Required for outdoor urban canyon |
| RF compliance | Standard 2.4/5.8 GHz bands |

---

## 4. Success Criteria & KPIs

### 4.1 MVP Success Criteria (Simulation)

| Criterion | Target | Measurement |
|-----------|--------|-------------|
| Waypoint arrival accuracy | < 0.5 m RMS | Euclidean distance to goal |
| Pose estimation drift | < 1% of distance traveled | Comparison to ground truth |
| Obstacle avoidance | 0 collisions in 50 runs | Gazebo contact sensor |
| End-to-end latency (cmd loop) | < 50 ms | ROS2 timestamp analysis |
| Mission completion rate | > 95% over 50 trials | Success/failure counter |
| System uptime | 100% node availability | Health monitor |

### 4.2 Hardware Flight Criteria

| Criterion | Target |
|-----------|--------|
| Sim-to-real pose accuracy gap | < 2x degradation |
| Safe landing on any failure | 100% |
| Battery-aware RTL trigger | Activates at 20% remaining |
| Manual override latency | < 200 ms from RC input |

### 4.3 Key Performance Indicators (Tracked Over Time)

| KPI | Unit | Tracking Method |
|-----|------|-----------------|
| VIO tracking loss rate | events/min | ROS2 diagnostics |
| Replanning frequency | replans/mission | Topic counter |
| CPU utilization (Jetson) | % | `tegrastats` logging |
| Memory usage | MB | `/proc/meminfo` |
| Localization covariance | m² | EKF output |

---

## 5. Failure Modes & Degradation Strategy

| Failure Mode | Detection | Response | Priority |
|--------------|-----------|----------|----------|
| VIO tracking lost | Covariance spike > threshold | Hold position (IMU dead-reckoning), attempt re-localization for 5s, then land | **P0** |
| Depth sensor failure | No depth msgs for > 1s | Switch to monocular depth estimation; reduce speed to 0.5 m/s | P1 |
| Companion computer crash | Heartbeat timeout 2s | PX4 failsafe: auto-land at current position | **P0** |
| Low battery | Voltage < 14.0V (4S) | Abort mission, RTL to launch point | **P0** |
| Communication loss (GCS) | No heartbeat 10s | Continue mission autonomously; RTL after mission complete | P2 |
| Planning failure (no path) | Planner returns empty | Hover in place, request operator input via telemetry | P1 |
| Motor/ESC failure | PX4 internal detection | Emergency land (controlled descent) | **P0** |
| IMU saturation | Accel/gyro clipping | Reduce aggressiveness; notify operator | P1 |

### 5.1 Fail-Safe State Machine

```
                    ┌─────────┐
                    │  ARMED  │
                    └────┬────┘
                         │
                    ┌────▼────┐
              ┌─────│ NOMINAL │─────┐
              │     └────┬────┘     │
              │          │          │
         VIO_LOST    LOW_BAT    COMMS_LOST
              │          │          │
        ┌─────▼───┐  ┌──▼──┐  ┌───▼────┐
        │  HOVER   │  │ RTL │  │ CONT.  │
        │(recover) │  │     │  │MISSION │
        └────┬─────┘  └──┬──┘  └───┬────┘
             │           │         │
        timeout 5s       │    mission done
             │           │         │
        ┌────▼────┐      │    ┌────▼───┐
        │  LAND   │◄─────┘    │  RTL   │
        └────┬────┘           └───┬────┘
             │                    │
        ┌────▼────────────────────▼───┐
        │          DISARMED           │
        └─────────────────────────────┘
```

---

## 6. Stakeholder & Role Matrix

| Role | Responsibility | Decision Authority |
|------|---------------|--------------------|
| Systems Architect | Overall architecture, interface design | Architecture decisions |
| Perception Engineer | VIO, SLAM, depth processing | Algorithm selection |
| Planning Engineer | Path planning, obstacle avoidance | Planner design |
| Controls Engineer | Trajectory tracking, PX4 integration | Control loop tuning |
| Simulation Engineer | Gazebo worlds, SITL config, test harness | Sim fidelity |
| Flight Test Pilot | Hardware flights, safety officer | Go/no-go for flight |
| Project Lead | Schedule, priorities, stakeholder comms | Scope and timeline |

---

## 7. Risk Register

| # | Risk | Likelihood | Impact | Mitigation |
|---|------|------------|--------|------------|
| R1 | VIO drift accumulates in featureless corridors | High | High | Loop closure in SLAM; add visual markers at intervals |
| R2 | Sim-to-real gap in depth perception | Medium | High | Domain randomization; real-world calibration dataset |
| R3 | Jetson thermal throttling | Medium | Medium | Active cooling; offload non-critical processing |
| R4 | PX4 EKF rejects VIO data | Medium | High | Careful covariance tuning; log and replay for debugging |
| R5 | ROS2 DDS latency spikes | Low | Medium | Tune QoS; use intra-process comms where possible |
| R6 | RealSense D435i IR interference in multi-UAV | Low | Medium | Single UAV in MVP; frequency hopping in multi |

---

## 8. Mission Framing Checklist

Use this checklist when defining a new mission:

- [ ] Environment surveyed / modeled in sim
- [ ] Waypoints defined in mission file (YAML)
- [ ] Lighting conditions assessed
- [ ] Feature density verified (min 50 ORB features per frame)
- [ ] Obstacle map loaded or mapping mode selected
- [ ] Fail-safe parameters configured for environment
- [ ] Battery capacity sufficient for mission + 30% reserve
- [ ] Communication link verified
- [ ] Pre-flight health check passed (all nodes green)
- [ ] Ground truth system set up (if testing)
- [ ] Flight log recording enabled
- [ ] Emergency procedures briefed
