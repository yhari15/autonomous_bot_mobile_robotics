#!/bin/bash
###
# Save Map Script
# Saves the current SLAM map to a file
# Assignment 4 Part 2 - SLAM Implementation
###

echo "========================================="
echo "Saving SLAM Map"
echo "========================================="

# Source workspace
source /home/hariprasad/ros2_ws/install/setup.bash

# Get map name from argument or use default
MAP_NAME=${1:-warehouse_map}
MAP_DIR="/home/hariprasad/ros2_ws/src/husky_ur3_slam/maps"

# Create maps directory if it doesn't exist
mkdir -p "$MAP_DIR"

echo ""
echo "Map will be saved as: ${MAP_DIR}/${MAP_NAME}"
echo ""

# Save the map
ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_NAME}"

echo ""
echo "========================================="
echo "Map saved successfully!"
echo ""
echo "Files created:"
echo "  - ${MAP_DIR}/${MAP_NAME}.pgm (image)"
echo "  - ${MAP_DIR}/${MAP_NAME}.yaml (metadata)"
echo ""
echo "To view the map:"
echo "  eog ${MAP_DIR}/${MAP_NAME}.pgm"
echo "========================================="
