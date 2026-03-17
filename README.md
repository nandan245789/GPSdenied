# Vision-Based Navigation System for Drones in GPS-Denied Environments

A ROS2 + PX4 autonomy stack for quadrotor navigation using visual-inertial odometry in GPS-denied indoor/urban environments.

## Architecture

```
Sensors → Perception → State Estimation → Mapping → Planning → Control → PX4
                              ↓                                    ↑
                        Mission Supervisor ─── Safety Monitor ─────┘
```

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

```bash
# Prerequisites: ROS2 Humble, Gazebo Harmonic, PX4 v1.15+
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

```bash
# Lint
colcon test --packages-select gps_denied_planning gps_denied_control
colcon test-result --verbose

# Record a bag
ros2 launch gps_denied_bringup record_bag.launch.py

# Run integration tests
colcon test --packages-select gps_denied_bringup
```

## License

MIT
