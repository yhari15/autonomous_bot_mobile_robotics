# Husky UR3 Mobile Manipulation with SLAM

A comprehensive ROS2 Humble workspace for autonomous mobile manipulation combining the Clearpath Husky mobile base with a Universal Robots UR3 arm, featuring 2D SLAM, autonomous navigation, and MASt3R-based 3D reconstruction capabilities.

## Project Overview

This workspace implements a complete mobile manipulation system for autonomous exploration and object manipulation in unknown environments. The system integrates:

- **Clearpath Husky** - 4-wheel differential drive mobile base
- **Universal Robots UR3** - 6-DOF robotic manipulator
- **RH-P12-RN Gripper** - Parallel jaw gripper for object manipulation
- **Stereolabs ZED Camera** - Stereo vision for 3D perception
- **2D SLAM** - Simultaneous localization and mapping
- **Nav2** - ROS2 navigation stack for autonomous path planning
- **MASt3R Integration** - 3D scene reconstruction from stereo imagery

## System Architecture

### Hardware Components

```
Husky Mobile Base
└── Top Plate
    ├── UR3 Robotic Arm
    │   └── RH-P12-RN Gripper
    └── ZED Stereo Camera
```

### Software Stack

- **ROS2 Humble** - Core framework
- **Ignition Gazebo (Fortress)** - Physics simulation
- **Nav2** - Navigation and path planning
- **SLAM Toolbox** - 2D mapping
- **MoveIt2** - Motion planning for manipulation
- **ZED ROS2 Wrapper** - Camera integration

## Features

- **Autonomous Navigation**: Global and local path planning with obstacle avoidance
- **2D SLAM**: Real-time mapping using laser scan data
- **Sensor Fusion**: EKF-based fusion of odometry, IMU, and GPS data
- **Object Manipulation**: Coordinated mobile manipulation with UR3 arm
- **3D Reconstruction**: MASt3R-based dense scene reconstruction
- **Mars Environment**: Bonus simulation with reduced gravity physics

## Workspace Structure

```
ros2_ws/
├── src/
│   ├── husky/                          # Clearpath Husky base packages
│   │   ├── husky_base/                 # Hardware drivers
│   │   ├── husky_control/              # Controllers and teleop
│   │   ├── husky_description/          # URDF/xacro models
│   │   ├── husky_gazebo/               # Gazebo integration
│   │   └── husky_navigation/           # Navigation configurations
│   │
│   ├── husky_ur3_description/          # Main integration package
│   │   ├── urdf/                       # Robot URDF/xacro files
│   │   ├── launch/                     # Launch files
│   │   ├── meshes/                     # 3D models (STL/DAE)
│   │   ├── worlds/                     # Gazebo world files
│   │   └── config/                     # Configuration files
│   │
│   ├── husky_ur3_simulator/            # Simulation packages
│   │   ├── husky_ur3_gazebo/           # Gazebo launch files
│   │   ├── husky_ur3_navigation/       # Nav2 configurations
│   │   ├── husky_ur3_gripper_moveit_config/  # MoveIt2 config
│   │   └── husky_ur3_nav_without_map/  # Map-less navigation
│   │
│   ├── husky_ur3_slam/                 # SLAM implementation
│   │   ├── launch/                     # SLAM launch files
│   │   └── config/                     # SLAM parameters
│   │
│   ├── must3r_nav/                     # MASt3R navigation integration
│   │   ├── launch/                     # Integrated launch files
│   │   ├── config/                     # Nav2 configurations
│   │   ├── scripts/                    # Python utilities
│   │   └── worlds/                     # Custom world files
│   │
│   └── zed-ros2-wrapper/               # ZED camera driver
│       ├── zed_wrapper/                # Main wrapper
│       ├── zed_components/             # Component nodes
│       └── zed_ros2/                   # Metapackage
│
├── build/                              # Build artifacts
├── install/                            # Installed packages
├── log/                                # Build and runtime logs
├── pointclouds/                        # Captured point clouds
├── must3r_captures/                    # MASt3R image captures
└── assignment4_recordings/             # Demo recordings
```

## Prerequisites

### System Requirements

- **OS**: Ubuntu 22.04 LTS (Jammy)
- **ROS**: ROS2 Humble Hawksbill
- **Gazebo**: Ignition Gazebo Fortress
- **Python**: 3.10+
- **CUDA**: 12.1+ (for ZED camera and MASt3R)

### Required System Packages

```bash
# ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop-full

# Gazebo Fortress
sudo apt install ignition-fortress

# ROS2-Gazebo Bridge
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge

# Navigation and SLAM
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox

# Robot State and Control
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-xacro
sudo apt install ros-humble-controller-manager

# Perception
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-image-transport

# MoveIt2
sudo apt install ros-humble-moveit
```

### ZED SDK Installation

```bash
# Download ZED SDK for Ubuntu 22 + CUDA 12.1
wget https://download.stereolabs.com/zedsdk/4.1/cu121/ubuntu22

# Install (requires NVIDIA GPU)
chmod +x ZED_SDK_Ubuntu22_cuda12.1_v4.1.4.zstd.run
sudo ./ZED_SDK_Ubuntu22_cuda12.1_v4.1.4.zstd.run -- silent skip_cuda
```

### Python Dependencies

```bash
# Install for MASt3R integration
pip3 install opencv-python numpy torch torchvision
```

## Installation

### 1. Clone the Repository

```bash
cd ~
git clone https://github.com/yhari15/ros2_ws.git
cd ros2_ws
```

### 2. Install ROS Dependencies

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace

```bash
source /opt/ros/humble/setup.bash
colcon build
```

For specific packages:
```bash
colcon build --packages-select husky_ur3_description
colcon build --packages-select must3r_nav
```

### 4. Source the Workspace

```bash
source install/setup.bash
```

Add to `.bashrc` for automatic sourcing:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Usage

### Basic Simulation

Launch the complete mobile manipulation system in Gazebo:

```bash
ros2 launch husky_ur3_description gazebo_sim.launch.py
```

This starts:
- Ignition Gazebo with warehouse environment
- Husky UR3 robot with gripper
- Robot state publisher for transforms
- ROS2-Gazebo topic bridges

### SLAM and Navigation

#### Launch SLAM with Navigation

```bash
ros2 launch must3r_nav full_navigation.launch.py
```

Features:
- Real-time 2D SLAM mapping
- Nav2 path planning
- Autonomous navigation
- Obstacle avoidance

#### Save Generated Map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

#### Launch with Pre-existing Map

```bash
ros2 launch must3r_nav navigation_with_map.launch.py map:=/path/to/map.yaml
```

### MASt3R 3D Reconstruction

#### Capture Images for Reconstruction

```bash
# Launch robot with ZED camera
ros2 launch must3r_nav capture_images.launch.py

# Images saved to must3r_captures/
```

#### Run MASt3R Processing

```bash
cd ~/MASt3R
python3 process_stereo.py --input ~/ros2_ws/must3r_captures --output pointcloud.ply
```

### Manipulation

#### Control Gripper

```bash
# Open gripper
ros2 topic pub /gripper_command std_msgs/msg/Float64 "data: 0.0"

# Close gripper
ros2 topic pub /gripper_command std_msgs/msg/Float64 "data: 0.8"
```

#### Move UR3 Arm (MoveIt2)

```bash
ros2 launch husky_ur3_gripper_moveit_config demo.launch.py
```

### Teleoperation

```bash
# Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Joystick control (if available)
ros2 launch husky_control teleop_joy.launch.py
```

### Visualization

```bash
# Launch RViz2
rviz2

# Configuration:
# - Fixed Frame: base_link
# - Add RobotModel
# - Add Map (topic: /map)
# - Add Path (topic: /plan)
# - Add LaserScan
```

## Key Scripts

### Navigation Testing

```bash
# Test autonomous navigation to goal
./test_nav_goal.py
```

### Gripper Testing

```bash
# Check gripper state
./check_gripper.py
```

### Mapping

```bash
# Run SLAM mapping session
./run_mapping.sh
```

## Package Descriptions

### husky_ur3_description

Main integration package containing URDF models, launch files, and configurations for the complete Husky UR3 system.

**Key Files:**
- `urdf/husky_ur3_gripper.urdf.xacro` - Complete robot model
- `launch/gazebo_sim.launch.py` - Main simulation launcher
- `worlds/warehouse.sdf` - Warehouse environment

### must3r_nav

MASt3R-integrated navigation package with SLAM and autonomous exploration capabilities.

**Features:**
- Nav2 configuration for Husky UR3
- SLAM parameter tuning
- MASt3R image capture integration
- Custom world environments (Mars simulation)

### husky_ur3_slam

2D SLAM implementation using SLAM Toolbox or gmapping.

**Capabilities:**
- Real-time mapping
- Loop closure detection
- Map serialization

### zed-ros2-wrapper

Stereolabs ZED camera driver for ROS2, providing:
- Stereo image streams
- Depth perception
- Point cloud generation
- Visual odometry
- Spatial mapping

## Configuration

### Navigation Parameters

Edit `src/must3r_nav/config/nav2_params.yaml`:

```yaml
# Controller frequency
controller_frequency: 10.0

# Planner
planner_server:
  plugin: "nav2_navfn_planner/NavfnPlanner"

# Controller
controller_server:
  plugin: "dwb_core::DWBLocalPlanner"
```

### SLAM Parameters

Edit `src/husky_ur3_slam/config/slam_params.yaml`:

```yaml
# Map resolution
resolution: 0.05

# Update frequency
map_update_interval: 5.0
```

## Troubleshooting

### Gazebo Not Responding

**Symptom**: Simulation time stuck at 0:00

**Solution**: Click Play (▶) in Gazebo GUI, or verify `-r` flag in launch file

### No Transform Errors in RViz

**Symptom**: `Transform [sender=unknown_publisher]` errors

**Solution**:
1. Ensure Gazebo is running (not paused)
2. Check `/clock` topic: `ros2 topic hz /clock`
3. Verify joint_states: `ros2 topic echo /joint_states`

### Navigation Not Working

**Symptom**: Robot doesn't move to goal

**Solution**:
1. Check if map is loaded: `ros2 topic echo /map`
2. Verify localization: `ros2 topic echo /amcl_pose`
3. Check costmaps: `ros2 topic list | grep costmap`

### ZED Camera Not Detected

**Symptom**: No camera topics published

**Solution**:
1. Verify ZED SDK: `ls /usr/local/zed`
2. Check CUDA: `nvidia-smi`
3. Test ZED: `ZED_Explorer`

### Build Errors

```bash
# Clean build
rm -rf build/ install/ log/

# Rebuild with verbose output
colcon build --event-handlers console_direct+
```

## Project Context

This workspace was developed as part of **CS498GC Mobile Robotics** coursework (Fall 2025), implementing:

- **Assignment 4 Part 1**: Mobile manipulator integration and simulation
- **Assignment 4 Part 2**: SLAM, navigation, and autonomous exploration
- **Bonus**: MASt3R 3D reconstruction and Mars environment simulation

## Key Technologies

- **SLAM**: SLAM Toolbox, gmapping, hector_slam
- **Navigation**: Nav2 (A*, DWA local planner)
- **Perception**: ZED stereo camera, MASt3R 3D reconstruction
- **Manipulation**: MoveIt2, UR3 kinematics
- **Simulation**: Ignition Gazebo with realistic physics

## Performance Benchmarks

- **Control Loop**: 50 Hz
- **SLAM Update**: 5-10 Hz
- **Navigation Planning**: 10 Hz
- **MASt3R Processing**: 15 FPS (CVPR 2025 paper)

## Documentation

Additional guides in the repository:

- `src/CLAUDE.md` - Claude Code integration guide
- `src/ASSIGNMENT4_PART2_COMPREHENSIVE_GUIDE.md` - Detailed assignment overview
- `src/must3r_nav/NAV2_USAGE_GUIDE.md` - Nav2 usage instructions
- `src/must3r_nav/MARS_WORLD_GUIDE.md` - Mars simulation setup
- `src/must3r_nav/TROUBLESHOOTING.md` - Common issues and fixes

## Testing

Verify complete system:

```bash
# 1. Launch simulation
ros2 launch husky_ur3_description gazebo_sim.launch.py

# 2. Check transforms
ros2 run tf2_tools view_frames
evince frames.pdf

# 3. Test navigation
ros2 launch must3r_nav full_navigation.launch.py

# 4. Send navigation goal
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}"
```

## Contributing

When making changes:

1. Create feature branch
2. Test in simulation
3. Verify transforms: `ros2 run tf2_tools view_frames`
4. Check no regressions in navigation
5. Update documentation

## Known Issues

- SLAM Toolbox may require manual loop closure in large environments
- ZED depth accuracy degrades beyond 15m
- Nav2 recovery behaviors may cause unexpected rotations in tight spaces
- MASt3R requires significant GPU memory (8GB+ recommended)

## License

This project integrates multiple open-source packages with different licenses:

- **Clearpath Husky**: BSD License
- **Universal Robots**: BSD-3-Clause
- **ZED ROS2 Wrapper**: MIT License
- **Nav2**: Apache 2.0

Custom code in this workspace follows the MIT License.

## Acknowledgments

- **Clearpath Robotics** - Husky mobile platform
- **Universal Robots** - UR3 manipulator
- **Stereolabs** - ZED camera and SDK
- **Open Robotics** - ROS2 and Gazebo
- **MASt3R Team** - CVPR 2025 3D reconstruction system
- **CS498GC Course Staff** - Assignment design and support

## Contact

**Repository**: https://github.com/yhari15/ros2_ws

For questions or issues, please open a GitHub issue.

## Quick Start Summary

```bash
# Install dependencies
sudo apt install ros-humble-desktop-full ros-humble-navigation2
sudo apt install ros-humble-slam-toolbox ignition-fortress

# Build workspace
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# Launch simulation
ros2 launch husky_ur3_description gazebo_sim.launch.py

# In new terminal - launch navigation
source ~/ros2_ws/install/setup.bash
ros2 launch must3r_nav full_navigation.launch.py

# Ready to navigate!
```

---

**Last Updated**: December 2025
**ROS2 Version**: Humble Hawksbill
**Gazebo Version**: Fortress
