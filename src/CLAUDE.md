# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 2 Humble workspace for a mobile manipulator system combining:
- **Clearpath Husky** mobile base
- **Universal Robots UR3** 6-DOF robotic arm
- **RH-P12-RN** gripper
- **Intel RealSense D435** camera (optional)

The system uses **Ignition Gazebo** for simulation and integrates navigation, manipulation, and visualization capabilities.

## Workspace Structure

```
ros2_ws/src/
├── husky/                          # Base Husky ROS2 packages (Clearpath)
│   ├── husky_base/                 # Hardware driver
│   ├── husky_bringup/              # Launch files
│   ├── husky_control/              # Controllers, teleop
│   ├── husky_description/          # URDF/xacro files
│   ├── husky_gazebo/               # Gazebo integration
│   ├── husky_navigation/           # Nav2 configs
│   └── husky_msgs/                 # Custom messages
├── husky_ur3_description/          # Main integration package (ROS2 Humble)
│   ├── urdf/                       # Robot models (xacro files)
│   ├── launch/                     # Launch files
│   ├── meshes/                     # 3D models
│   ├── worlds/                     # Gazebo world files
│   └── config/                     # Configuration files
└── husky_ur3_simulator/            # Legacy ROS1 Noetic package (not actively used)
```

## Build System

This workspace uses **colcon** (ROS2 build tool) with **ament_cmake**.

### Building the Workspace

```bash
# Full workspace build
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build

# Build specific package
colcon build --packages-select husky_ur3_description

# Build with verbose output
colcon build --event-handlers console_direct+
```

### Sourcing the Workspace

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Running the System

### Launch Gazebo Simulation

The main entry point for the simulation:

```bash
ros2 launch husky_ur3_description gazebo_sim.launch.py
```

**Key behaviors:**
- Launches Ignition Gazebo with warehouse world
- Spawns Husky UR3 robot with gripper
- Starts robot_state_publisher for TF transforms
- Bridges ROS2-Gazebo topics (/clock, /joint_states)
- Simulation auto-starts with `-r` flag (running mode)

**Critical:** If simulation appears paused (time stuck at 0:00), click the Play button (▶) in Gazebo's bottom-left corner.

### Killing Processes

Always clean up before relaunching:

```bash
killall -9 ros2 ign gz gazebo rviz2 python3 ruby 2>/dev/null
sleep 3
```

## Key Architecture Details

### URDF/Xacro Structure

Main robot model: `urdf/husky_ur3_gripper.urdf.xacro`

The robot is composed hierarchically:
- `husky.urdf.xacro` - Mobile base (4-wheel differential drive)
- `ur3.urdf.xacro` - 6-DOF manipulator arm
- `rh_p12_rn_gripper.xacro` - End effector
- `d435_camera.urdf.xacro` - Vision sensor (optional)

Key integration points:
- UR3 base is fixed to Husky's `top_plate_link`
- Gripper attached to UR3's `wrist_3_link`
- All links use Gazebo plugins for physics simulation

### Transform (TF) Tree

```
base_link (Husky base)
├── [wheels: front_left, front_right, rear_left, rear_right]
├── top_plate_link
    └── ur3_base_link
        └── shoulder_link
            └── upper_arm_link
                └── forearm_link
                    └── wrist_1_link
                        └── wrist_2_link
                            └── wrist_3_link
                                └── ee_link
                                    └── rh_p12_rn_base (gripper)
                                        └── [gripper fingers]
```

### ROS2-Gazebo Bridge

The `ros_gz_bridge` connects:
- `/clock` - Simulation time
- `/joint_states` - Joint positions/velocities/efforts

Gazebo publishes joint states via the `JointStatePublisher` plugin embedded in the URDF. **Do not** manually run `joint_state_publisher` - it conflicts with Gazebo.

### Critical Timestamp Behavior

- `robot_state_publisher` uses `use_sim_time: true` and `ignore_timestamp: false`
- It will **not** publish TF transforms if joint_states have zero timestamps
- Zero timestamps occur when Gazebo is paused
- This is the most common cause of "No transform" errors in RViz

## Common Development Tasks

### Modifying Robot Model

1. Edit URDF/xacro files in `husky_ur3_description/urdf/`
2. Rebuild package: `colcon build --packages-select husky_ur3_description`
3. Source workspace: `source install/setup.bash`
4. Relaunch simulation

### Debugging Transform Issues

```bash
# Check clock is advancing
ros2 topic echo /clock

# Verify joint states have valid timestamps
ros2 topic echo /joint_states --once

# Check TF is publishing
ros2 topic echo /tf | head -20

# Generate TF tree visualization
ros2 run tf2_tools view_frames
evince frames.pdf
```

### Visualizing in RViz2

```bash
rviz2
```

**Configuration:**
1. Global Options → Fixed Frame: `base_link`
2. Add → RobotModel
3. RobotModel → Description Topic: `/robot_description`

If RViz crashes, clean config: `rm -f ~/.rviz2/default.rviz`

### Testing Topics

```bash
# List all topics
ros2 topic list

# Check topic info
ros2 topic info /joint_states

# Monitor topic rate
ros2 topic hz /clock
```

## Known Issues and Fixes

### Issue: "No transform" errors in RViz

**Cause:** Gazebo simulation paused (clock at 0/0)

**Fix:** Click Play (▶) in Gazebo GUI, or verify `-r` flag in launch file

### Issue: RViz crashes on startup

**Cause:** Corrupted config file

**Fix:** `rm -f ~/.rviz2/default.rviz`

### Issue: Robot not spawning

**Cause:** Gazebo not fully initialized

**Fix:** Launch file includes 3-second delay (TimerAction). If still failing, increase delay in `gazebo_sim.launch.py` line 62.

### Issue: Mesh files not loading in Gazebo

**Cause:** Resource path not set

**Fix:** Launch file sets `GZ_SIM_RESOURCE_PATH` automatically. Verify meshes exist in `husky_ur3_description/meshes/`

## Project Context

This workspace was developed for a mobile robotics course assignment involving:
- Integration of mobile base and manipulator
- Simulation in Gazebo
- TF tree management
- Visualization in RViz

Key accomplishments documented in workspace root:
- Transform fixes (TRANSFORM_FIX_FINAL.md)
- System testing guide (HOW_TO_TEST_FIXED_SYSTEM.md)
- Implementation changes (CHANGES_SUMMARY.md)

## Dependencies

**System packages (apt):**
- ros-humble-ros-gz-sim
- ros-humble-ros-gz-bridge
- ros-humble-robot-state-publisher
- ros-humble-joint-state-publisher
- ros-humble-xacro
- ignition-fortress

**Verify installation:**
```bash
ros2 pkg list | grep -E "ros_gz|robot_state_publisher|xacro"
```

## File Locations

- Main launch: `src/husky_ur3_description/launch/gazebo_sim.launch.py`
- Main URDF: `src/husky_ur3_description/urdf/husky_ur3_gripper.urdf.xacro`
- World files: `src/husky_ur3_description/worlds/warehouse.sdf`
- Build artifacts: `build/`, `install/`, `log/`

## Additional Notes

- The `husky_ur3_simulator` package is legacy ROS1 Noetic code - do not use for ROS2 development
- The Husky package warns about migration to clearpath_common for Humble, but works fine as-is
- Always source both ROS2 and workspace setup files in every terminal
- Gazebo Ignition (not Gazebo Classic) is used - commands use `ign gazebo`, not `gazebo`
