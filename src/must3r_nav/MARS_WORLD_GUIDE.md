# Mars Thanksgiving World Setup Guide
## Assignment 4 Part 2 - MUST3R on Mars

### ✅ What's Been Set Up

All the Mars world files are now properly installed and configured:

```
must3r_nav/
├── worlds/
│   └── mars_thanksgiving.world      ✓ Mars environment with table, terrain, turkeys
├── config/
│   └── navigation_waypoints.yaml    ✓ 30 waypoints in 3 rings around table
├── scripts/
│   └── waypoint_navigator.py        ✓ Autonomous waypoint navigation script
└── launch/
    └── mars_thanksgiving.launch.py  ✓ Launch file for Mars world + Husky
```

### 🚀 Quick Start - Launch Mars World

**Terminal 1: Launch Mars Thanksgiving World with Husky**
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch must3r_nav mars_thanksgiving.launch.py
```

This will:
- Start Gazebo with Mars Thanksgiving World (Mars gravity: 3.711 m/s²)
- Spawn Husky UR3 robot at start line (-2.0, 0.0)
- Enable robot controllers (diff_drive, arm, gripper)
- Bridge ROS2-Gazebo topics

### 🎯 World Features

**Mars Environment:**
- **Thanksgiving table** at (3.0, 0.0) with chairs, plates, cups
- **2 Flying Turkeys**: One hovering above table, one running underneath
- **Mars terrain**: Hills, craters, rocks
- **Navigation obstacles**: Boulders for path planning
- **Start line** (green) at (-2.0, 0.0)
- **Finish line** (checkered) at (8.0, 0.0)

**Waypoint Coverage:**
- Ring 1: 10 waypoints at 1.5m radius (close-up arm config)
- Ring 2: 10 waypoints at 2.5m radius (medium arm config)
- Ring 3: 10 waypoints at 3.5m radius (wide arm config)

### 🎮 Manual Control (Testing)

**Terminal 2: Control robot manually**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

### 📸 Camera Topics

Check available camera feeds:
```bash
# Husky base camera
ros2 topic echo /camera/color/image_raw --once

# Check if tip camera is available
ros2 topic list | grep camera
```

### 🗺️ Next Steps

#### 1. **Nav2 Setup** (Required for Waypoint Navigation)
The waypoint navigator uses Nav2 for autonomous navigation. Need to:
- Configure Nav2 parameters for Mars gravity
- Set up costmaps and planners
- Launch Nav2 stack

#### 2. **MoveIt Setup** (Required for Arm Control)
Configure MoveIt for UR3 arm to:
- Control arm joint positions at each waypoint
- Set different arm configs (close_up, medium, wide)
- Capture images from optimal angles

#### 3. **MUST3R Installation**
```bash
# Create Python 3.11 environment
python3.11 -m venv ~/must3r_env
source ~/must3r_env/bin/activate
pip install must3r@git+https://github.com/naver/must3r.git
```

#### 4. **Test Waypoint Navigation**
Once Nav2 is configured:
```bash
ros2 run must3r_nav waypoint_navigator
```

### 📋 Troubleshooting

**Issue: Gazebo appears paused (time stuck at 0:00)**
- Click the Play button (▶) in Gazebo's bottom-left corner

**Issue: Robot not visible**
- Wait 5 seconds after Gazebo starts (spawn is delayed)
- Check terminal for spawn errors

**Issue: "No transform" errors**
- Verify Gazebo clock is running: `ros2 topic hz /clock`
- Check joint states: `ros2 topic echo /joint_states --once`

**Issue: World fails to load**
- Check Gazebo resource path is set correctly
- Verify world file exists: `ls ~/ros2_ws/src/must3r_nav/worlds/`

### 🔍 Verification Commands

```bash
# Check if world is loading
ros2 topic list

# Verify robot description
ros2 topic echo /robot_description --once | head -20

# Check TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Monitor joint states
ros2 topic echo /joint_states

# Check odometry
ros2 topic echo /diff_drive_controller/odom
```

### 📊 Assignment Requirements Checklist

- [x] Mars Gazebo World (Option 4 - Thanksgiving World)
- [x] Waypoint navigation YAML (30 waypoints)
- [x] Waypoint navigator script
- [x] Launch file for Mars + Husky
- [ ] Nav2 configuration for autonomous navigation
- [ ] MoveIt configuration for arm control
- [ ] MUST3R 3D reconstruction implementation
- [ ] Point cloud processing (PointCloud2 output)
- [ ] RViz2 visualization setup
- [ ] Rosbag recording capability
- [ ] Screen recording demo
- [ ] Technical documentation

### 📝 Grading Rubric Summary (175 pts)

1. **MUST3R 3D Reconstruction** (35 pts) - Multi-view capture & reconstruction
2. **Point Cloud Processing** (25 pts) - PointCloud2, filtering, RViz visualization
3. **Mars Navigation** (25 pts) - Obstacle detection, waypoint following, path planning
4. **Screen Recording Demo** (40 pts) - Show Gazebo, RViz, MoveIt, terminals
5. **Rosbag Recording** (20 pts) - Sensor data + point clouds
6. **Documentation** (15 pts) - Technical report + results
7. **Code Quality** (15 pts) - Clean code + README

### 🎓 Bonus Opportunities (+35 pts max)

- MAST3R implementation (+10 pts)
- SOTA methods (VGGT, CUT3R, UNI4D, TTT3R) (+15 pts)
- Multiple Mars worlds (+5 pts)
- Exceptional visualization (+5 pts)

---

**Status**: Mars world setup complete ✓ | Next: Nav2 + MoveIt configuration
