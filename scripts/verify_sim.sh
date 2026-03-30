#!/bin/bash
# =============================================================
# verify_sim.sh — Verify simulation environment is working
# Run this INSIDE the Docker container after launch_sim.sh
# =============================================================

echo "============================================="
echo " GPS-Denied Simulation — Health Check"
echo "============================================="
echo ""

PASS=0
FAIL=0

check() {
    if eval "$2" > /dev/null 2>&1; then
        echo "  ✅ $1"
        PASS=$((PASS + 1))
    else
        echo "  ❌ $1"
        FAIL=$((FAIL + 1))
    fi
}

echo "--- ROS2 Environment ---"
check "ROS2 Humble installed" "ros2 --help"
check "Workspace built" "[ -f /workspace/install/setup.bash ]"

echo ""
echo "--- PX4 Topics ---"
check "/fmu/out/vehicle_status exists" "ros2 topic list | grep -q /fmu/out/vehicle_status"
check "/fmu/out/vehicle_local_position exists" "ros2 topic list | grep -q /fmu/out/vehicle_local_position"
check "/fmu/out/sensor_combined exists" "ros2 topic list | grep -q /fmu/out/sensor_combined"

echo ""
echo "--- Our Packages ---"
check "gps_denied_interfaces built" "ros2 pkg list | grep -q gps_denied_interfaces"
check "gps_denied_perception built" "ros2 pkg list | grep -q gps_denied_perception"
check "gps_denied_planning built" "ros2 pkg list | grep -q gps_denied_planning"
check "gps_denied_control built" "ros2 pkg list | grep -q gps_denied_control"
check "gps_denied_safety built" "ros2 pkg list | grep -q gps_denied_safety"
check "gps_denied_state_estimation built" "ros2 pkg list | grep -q gps_denied_state_estimation"
check "gps_denied_mapping built" "ros2 pkg list | grep -q gps_denied_mapping"
check "gps_denied_bringup built" "ros2 pkg list | grep -q gps_denied_bringup"

echo ""
echo "============================================="
echo " Results: $PASS passed, $FAIL failed"
echo "============================================="

if [ $FAIL -eq 0 ]; then
    echo " 🚀 Phase 0 COMPLETE — ready for Phase 1 (VIO)"
else
    echo " ⚠️  Fix the failures above before proceeding"
fi
