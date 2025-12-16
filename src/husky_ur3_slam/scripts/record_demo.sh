#!/bin/bash
###
# Record Demo ROSbag Script
# Records all required topics for assignment submission
# Assignment 4 Part 2 - Minimum 5 minute recording
###

echo "========================================="
echo "Recording Demo ROSbag"
echo "Assignment 4 Part 2"
echo "========================================="

# Source workspace
source /home/hariprasad/ros2_ws/install/setup.bash

# Get recording name from argument or use timestamp
if [ -z "$1" ]; then
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    BAG_NAME="demo_${TIMESTAMP}"
else
    BAG_NAME="$1"
fi

BAG_DIR="/home/hariprasad/ros2_ws/rosbags"

# Create rosbags directory if it doesn't exist
mkdir -p "$BAG_DIR"

echo ""
echo "Recording will be saved to: ${BAG_DIR}/${BAG_NAME}"
echo ""
echo "Recording the following topics:"
echo "  - /scan (laser scanner)"
echo "  - /joint_states (robot joints)"
echo "  - /tf (transforms)"
echo "  - /tf_static (static transforms)"
echo "  - /robot_pose (EKF output)"
echo "  - /map (SLAM map)"
echo "  - /cmd_vel (velocity commands)"
echo "  - /odometry (wheel odometry)"
echo "  - /imu (IMU data)"
echo "  - /gps/fix (GPS data)"
echo ""
echo "Press Ctrl+C to stop recording"
echo "Minimum duration: 5 minutes (300 seconds)"
echo ""
echo "========================================="

# Start recording
ros2 bag record \
  -o "${BAG_DIR}/${BAG_NAME}" \
  /scan \
  /joint_states \
  /tf \
  /tf_static \
  /robot_pose \
  /map \
  /cmd_vel \
  /odometry \
  /imu \
  /gps/fix \
  /local_costmap/costmap \
  /global_costmap/costmap

echo ""
echo "========================================="
echo "Recording saved to: ${BAG_DIR}/${BAG_NAME}"
echo ""
echo "To play back the recording:"
echo "  ros2 bag play ${BAG_DIR}/${BAG_NAME}"
echo ""
echo "To get bag info:"
echo "  ros2 bag info ${BAG_DIR}/${BAG_NAME}"
echo "========================================="
