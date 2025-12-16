#!/bin/bash
# =============================================================================
# Mars Table Mapping - Full Pipeline
# 
# This script runs the complete mapping sequence:
# 1. Launches Gazebo simulation with Nav2
# 2. Starts MUSt3R bridge for 3D reconstruction
# 3. Runs waypoint navigator (start → end)
# 4. Saves PCD file automatically at the end
#
# Output: ~/ros2_ws/pointclouds/pointcloud_YYYYMMDD_HHMMSS.pcd
# =============================================================================

set -e

echo "=============================================="
echo "   MARS TABLE MAPPING PIPELINE"
echo "=============================================="

# Cleanup any existing processes
echo "[1/5] Cleaning up old processes..."
pkill -9 -f "ros2" 2>/dev/null || true
pkill -9 -f "ign gazebo" 2>/dev/null || true
pkill -9 -f "gzserver" 2>/dev/null || true
pkill -9 -f "gzclient" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
sleep 3

# Setup ROS2 environment
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Create output directories
mkdir -p ~/ros2_ws/pointclouds
mkdir -p /tmp/must3r_bridge/images
mkdir -p /tmp/must3r_bridge/points

echo "[2/5] Starting Gazebo + Nav2 simulation..."
ros2 launch must3r_nav mars_nav2.launch.py &
GAZEBO_PID=$!

# Wait for simulation to initialize
echo "    Waiting 30 seconds for simulation to start..."
sleep 30

echo "[3/5] Starting stereo depth node (3D reconstruction)..."
ros2 run must3r_nav stereo_depth &
DEPTH_PID=$!
sleep 5

echo "[4/5] Starting waypoint navigator..."
echo ""
echo "=============================================="
echo "   NAVIGATION STARTING"
echo "   Path: Start → Table → End"
echo "   PCD will be saved automatically at end"
echo "=============================================="
echo ""

ros2 run must3r_nav waypoint_navigator

echo ""
echo "[5/5] Mapping complete!"
echo ""
echo "=============================================="
echo "   OUTPUT FILES:"
echo "   - PCD: ~/ros2_ws/pointclouds/"
echo "   - Images: ~/ros2_ws/must3r_captures/"
echo "=============================================="

# Keep simulation running for review
echo ""
echo "Press Ctrl+C to exit and close simulation..."
wait $GAZEBO_PID

