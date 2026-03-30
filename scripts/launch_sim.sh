#!/bin/bash
# =============================================================
# launch_sim.sh — Start full simulation stack
# PX4 SITL + Gazebo + ROS2 bridge + camera topics
# =============================================================
set -e

echo ">>> Starting GPS-Denied Simulation Stack..."

# Source everything
source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f /opt/ros2_ws/install/setup.bash ] && source /opt/ros2_ws/install/setup.bash
[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash

export PX4_ROOT=/opt/PX4-Autopilot

# --- 1. Start PX4 SITL with Gazebo in background ---
echo ">>> [1/4] Starting PX4 SITL + Gazebo..."
cd $PX4_ROOT
# Disable GPS in EKF2
export PX4_SIM_MODEL=gz_x500
make px4_sitl gz_x500 &
PX4_PID=$!
sleep 15  # Wait for PX4 + Gazebo to initialize

# --- 2. Start the micro XRCE-DDS agent (PX4 ↔ ROS2 bridge) ---
echo ">>> [2/4] Starting micro-XRCE-DDS agent..."
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
sleep 3

# --- 3. Verify PX4 topics are visible ---
echo ">>> [3/4] Checking PX4 ROS2 topics..."
ros2 topic list | grep -c fmu && echo "✓ PX4 topics detected" || echo "✗ PX4 topics NOT found"

# --- 4. Report status ---
echo ""
echo "============================================="
echo " Simulation Stack Running"
echo "============================================="
echo " PX4 SITL PID:  $PX4_PID"
echo " DDS Agent PID: $AGENT_PID"
echo ""
echo " Verify with:"
echo "   ros2 topic list"
echo "   ros2 topic hz /fmu/out/vehicle_status"
echo ""
echo " To disable GPS (if not already):"
echo "   param set EKF2_GPS_CTRL 0"
echo "   param set SYS_HAS_GPS 0"
echo ""
echo " Press Ctrl+C to stop all processes"
echo "============================================="

# Keep alive — wait for Ctrl+C
trap "kill $PX4_PID $AGENT_PID 2>/dev/null; exit 0" SIGINT SIGTERM
wait
