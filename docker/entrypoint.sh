#!/bin/bash
set -e

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source px4_msgs workspace
if [ -f /opt/ros2_ws/install/setup.bash ]; then
    source /opt/ros2_ws/install/setup.bash
fi

# Source our workspace
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# Set PX4 paths
export PX4_ROOT=/opt/PX4-Autopilot
export PATH=$PX4_ROOT/build/px4_sitl_default/bin:$PATH

echo "============================================="
echo " GPS-Denied Navigation — Dev Environment"
echo " ROS2: ${ROS_DISTRO}"
echo " Gazebo: Harmonic"
echo " PX4: v1.15"
echo "============================================="
echo ""
echo "Quick commands:"
echo "  launch_sim     — Start PX4 SITL + Gazebo + ROS2 bridge"
echo "  build          — Rebuild workspace"
echo "  topics         — List all ROS2 topics"
echo ""

# Convenience aliases
alias launch_sim='bash /workspace/scripts/launch_sim.sh'
alias build='cd /workspace && colcon build --symlink-install && source install/setup.bash'
alias topics='ros2 topic list'

exec "$@"
