Nandan @ISPL             GPS denied Workflow Architecture
Reporting to Manager: Karan Dhaul

Project:Vision-Based Navigation System for Drones in GPS-Denied Environments

I will be integrating ROS2 + PX4 autonomy stack for quadcopter navigation using visual-inertial odometry in GPS-denied indoor/urban environments.

Starting out with a simulation-first algorithmic framework enabling autonomous navigation without GPS using onboard vision and inertial sensing. 

The initial phase focuses on validating the navigation algorithm in a simulated environment before integrating full robotics infrastructure or hardware.

I have broken this project in Phase-wise execution and hope to maintain and publish document records for each phase as much as possible.

## Phase 1 Simulation Only
This phase focuses on algorithm validation without hardware.
Implement a simple navigation loop using simulation ground truth to validate planning logic.
Apply realistic challenges:

Simulation environment will provide:
drone dynamics
camera sensor feed
IMU readings
obstacle environment
waypoint targets
The autonomy algorithm will process simulated sensor inputs and produce flight commands.

## My Deliverable:
A fully functional simulation-based autonomous navigation loop demonstrating:
    waypoint navigation
    obstacle avoidance
    safe behavior when perception fails


## Architecture
Simulated Sensors
(Camera + IMU)
        │
        ▼
Perception Layer
(Feature tracking / depth / obstacle detection)
        │
        ▼
State Estimation
(Visual Odometry / Visual-Inertial Odometry)
        │
        ▼
Local Environment Representation
(Obstacle map / free-space grid)
        │
        ▼
Navigation Planner
(goal seeking + obstacle avoidance)
        │
        ▼
Control Command Generator
(velocity / heading commands)
        │
        ▼
Drone Simulation

## Risks and Mitigation
  Risk :Vision algorithms may struggle in low-texture environments.

  Mitigation:Use simulated depth or stereo sensors during early experiments.

  Risk:Planner instability near obstacles.

  Mitigation
  Use conservative safety margins and reactive fallback behaviors.


## KPI 
  waypoint completion rate
  collision rate
  path efficiency
  navigation stability
  robustness to perception errors


## Packages

| Package | Purpose |
|---------|---------|
| `gps_denied_interfaces` | Custom msg/srv/action definitions |
| `gps_denied_perception` | Camera drivers, feature extraction, point cloud gen |
| `gps_denied_state_estimation` | VIO (VINS-Fusion), EKF fusion, pose output |
| `gps_denied_mapping` | OctoMap builder, obstacle inflation |
| `gps_denied_planning` | Waypoint manager, RRT* local planner |
| `gps_denied_control` | Trajectory tracking, PX4 offboard bridge |
| `gps_denied_safety` | Mission supervisor, fail-safe state machine |
| `gps_denied_sim` | Gazebo worlds, drone models, SITL config |
| `gps_denied_bringup` | Launch files, system config, telemetry |

## Quickstart
# Clone and build
cd ~/GPSdenied
colcon build --symlink-install
source install/setup.bash

# Launch simulation
ros2 launch gps_denied_bringup sim_full.launch.py

# Run a mission
ros2 service call /mission/set_mission gps_denied_interfaces/srv/SetMission \
  "{mission_file: 'missions/warehouse_inspection.yaml'}"
```

## Documentation

- [Mission Framing](docs/01_mission_framing.md)
- [MVP Architecture & Subsystem Specs](docs/02_mvp_architecture.md)
- [Subsystem Workflow](docs/03_subsystem_workflow.md)
- [Development Roadmap](docs/04_development_roadmap.md)

## Development

