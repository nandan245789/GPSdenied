#!/bin/bash
# =============================================================
# setup.sh — One-command setup for GPS-Denied Navigation
# Run this on your Mac after installing Docker Desktop
# =============================================================
set -e

echo "============================================="
echo " GPS-Denied Navigation — Setup"
echo "============================================="

# Check Docker
if ! command -v docker &> /dev/null; then
    echo ""
    echo "❌ Docker not found. Please install Docker Desktop first:"
    echo "   https://www.docker.com/products/docker-desktop/"
    echo ""
    echo "   After installing, make sure Docker Desktop is RUNNING,"
    echo "   then run this script again."
    exit 1
fi

echo "✅ Docker found: $(docker --version)"
echo ""

# Check Docker is running
if ! docker info &> /dev/null; then
    echo "❌ Docker is installed but not running."
    echo "   Open Docker Desktop and wait for it to start, then retry."
    exit 1
fi

echo "✅ Docker daemon is running"
echo ""

# Build the image (this takes 15-30 min first time)
echo ">>> Building simulation environment (first time takes 15-30 min)..."
echo "    This installs: ROS2 Humble + Gazebo Harmonic + PX4 v1.15"
echo ""

cd "$(dirname "$0")"
docker compose -f docker/docker-compose.yml build dev

echo ""
echo "============================================="
echo " ✅ Setup Complete!"
echo "============================================="
echo ""
echo " Start the dev environment:"
echo "   docker compose -f docker/docker-compose.yml run dev"
echo ""
echo " Inside the container, run:"
echo "   bash scripts/launch_sim.sh    # Start simulation"
echo "   bash scripts/verify_sim.sh    # Verify everything works"
echo ""
