# Mars Navigation with Nav2 - Usage Guide

## Overview
This guide explains how to use the autonomous navigation system for the Mars Thanksgiving assignment.

## System Components

### 1. **Nav2 Navigation Stack**
- **Controller**: DWB (Dynamic Window Approach) for local planning
- **Planner**: NavFn with A* algorithm for global path planning
- **Costmaps**: Obstacle avoidance using laser scanner
- **Waypoint Follower**: Automatic waypoint following

### 2. **Robot Cameras**
- **Base Camera**: ZED 2 on Husky base (`/zed_base/left/image_raw`)
- **Arm Camera**: ZED 2 on UR3 wrist (`/zed_arm/left/image_raw`)

### 3. **Waypoint Navigator**
- Autonomous navigation through 30 waypoints
- Arm control for different viewing angles
- Automatic image capture for MUST3R

---

## Quick Start

### Step 1: Launch Mars World with Nav2

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch Mars world with Nav2 navigation
ros2 launch must3r_nav mars_nav2.launch.py
```

**What this does:**
- ✅ Starts Gazebo with Mars Thanksgiving world
- ✅ Spawns Husky UR3 robot at (-2.0, 0.0)
- ✅ Starts Nav2 navigation stack (takes ~10 seconds)
- ✅ Bridges all necessary topics (cameras, laser, odometry)

### Step 2: Run Waypoint Navigator

**In a new terminal:**

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run the waypoint navigator
ros2 run must3r_nav waypoint_navigator
```

**What this does:**
- ✅ Navigates through all 30 waypoints autonomously
- ✅ Avoids boulders and obstacles
- ✅ Positions arm at each waypoint (close_up, medium, wide)
- ✅ Captures images from both cameras
- ✅ Saves images to `~/ros2_ws/must3r_captures/`

---

## Understanding the Navigation

### Waypoint Rings

Your 30 waypoints are organized in 3 rings around the table:

```
Ring 1 (r=1.5m): Waypoints 1-10  → Arm: close_up
Ring 2 (r=2.5m): Waypoints 11-20 → Arm: medium
Ring 3 (r=3.5m): Waypoints 21-30 → Arm: wide
```

### Arm Configurations

```yaml
close_up:  [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # Arm extended
medium:    [0.0, -0.785, 0.785, -0.785, -1.57, 0.0]  # 45° angle
wide:      [0.0, -0.3, 0.3, -0.3, -1.57, 0.0]  # Horizontal
```

---

## Monitoring Navigation

### View Topics

```bash
# Check navigation status
ros2 topic list | grep nav

# Monitor robot position
ros2 topic echo /odom

# Watch laser scanner (obstacle detection)
ros2 topic echo /scan

# Check camera feeds
ros2 topic hz /zed_base/left/image_raw
ros2 topic hz /zed_arm/left/image_raw
```

### Visualize in RViz2

```bash
rviz2
```

**Add these displays:**
1. **RobotModel** - See the Husky UR3
2. **Map** - Topic: `/global_costmap/costmap`
3. **LaserScan** - Topic: `/scan`
4. **Path** - Topic: `/plan` (global path)
5. **Camera** - Topic: `/zed_base/left/image_raw`

**Set Fixed Frame:** `map`

---

## Output Files

After running the waypoint navigator, you'll find:

```
~/ros2_ws/must3r_captures/
├── wp01_base.jpg          # Waypoint 1, base camera
├── wp01_arm.jpg           # Waypoint 1, arm camera
├── wp02_base.jpg
├── wp02_arm.jpg
├── ...
├── wp30_base.jpg
├── wp30_arm.jpg
└── image_list.txt         # List of all captured images (60 total)
```

**Total images:** 60 (2 cameras × 30 waypoints)

---

## Troubleshooting

### Issue: "Nav2 action server not available"
**Cause:** Nav2 stack not fully started yet
**Fix:** Wait 15-20 seconds after launching mars_nav2.launch.py

```bash
# Check if Nav2 is running
ros2 node list | grep nav

# Should see:
# /bt_navigator
# /controller_server
# /planner_server
# /behavior_server
# /waypoint_follower
```

### Issue: Robot not moving
**Cause:** Gazebo might be paused
**Fix:** Click Play (▶) button in Gazebo GUI

### Issue: Robot gets stuck on boulder
**Cause:** Costmap not detecting obstacle
**Fix:** Check laser scanner is working

```bash
ros2 topic echo /scan --once
```

### Issue: Arm not moving
**Cause:** Arm controller not available
**Fix:** The waypoint navigator will continue without arm movement (it's optional)

### Issue: No images captured
**Cause:** Cameras not publishing
**Fix:** Check camera topics

```bash
ros2 topic list | grep zed
```

---

## Nav2 Parameters (Mars Adapted)

Located in: `src/must3r_nav/config/mars_nav2_params.yaml`

**Key settings for Mars:**
- **Max velocity:** 0.4 m/s (reduced for rough terrain)
- **Robot radius:** 0.6m (conservative for safety)
- **Inflation radius:** 1.0m (larger buffer around boulders)
- **Planner:** A* algorithm (assignment requirement)
- **Controller frequency:** 10 Hz

---

## Manual Testing

### Test Single Waypoint

```python
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 4.5, y: 0.0, z: 0.0},
    orientation: {z: 1.0, w: 0.0}
  }
}" --once
```

### Test Arm Movement

```bash
# Publish joint trajectory
ros2 topic pub /arm_controller/follow_joint_trajectory/goal \
  control_msgs/action/FollowJointTrajectory "{...}"
```

---

## Performance Tuning

### Slower Robot (More Accurate)
Edit `mars_nav2_params.yaml`:
```yaml
max_vel_x: 0.3  # Reduce from 0.4
```

### Faster Robot (Less Safe)
```yaml
max_vel_x: 0.6  # Increase from 0.4
inflation_radius: 0.7  # Reduce buffer
```

### Rebuild after changes:
```bash
colcon build --packages-select must3r_nav
source install/setup.bash
```

---

## Next Steps for MUST3R

Once you have all 60 images:

1. **Process with MUST3R**:
   ```python
   from must3r import MuST3R
   model = MuST3R.from_pretrained("naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt")
   ```

2. **Generate 3D Reconstruction**:
   - Use the images from `~/ros2_ws/must3r_captures/`
   - Follow MUST3R documentation for multi-view reconstruction
   - Export as point cloud (PointCloud2)

3. **Publish to RViz2**:
   - Use your existing point cloud nodes
   - Visualize the 3D reconstructed table

---

## Assignment Checklist

- [x] Mars Gazebo World with obstacles
- [x] Nav2 autonomous navigation
- [x] Waypoint following (30 waypoints)
- [x] Obstacle avoidance (boulders, table)
- [x] A* path planning
- [x] Arm control (3 configurations)
- [x] Dual camera setup (base + arm)
- [x] Automatic image capture
- [ ] MUST3R 3D reconstruction (next step)
- [ ] Point cloud processing
- [ ] RViz2 visualization
- [ ] Rosbag recording
- [ ] Screen recording demo
- [ ] Technical documentation

---

## Rubric Coverage

| Requirement | Implementation | Points |
|-------------|----------------|--------|
| Mars Navigation | Nav2 with DWB + A* | 25 pts |
| Obstacle Detection | Laser scanner + costmaps | ✓ |
| Waypoint Following | 30 waypoints, 3 rings | ✓ |
| Path Planning | A* global planner | ✓ |
| Camera Setup | 2 ZED cameras | For MUST3R |
| Arm Control | 3 configurations | For multi-view |

---

## Support

- **Nav2 Documentation**: https://navigation.ros.org/
- **MUST3R GitHub**: https://github.com/naver/dust3r
- **ROS 2 Humble Docs**: https://docs.ros.org/en/humble/

**Status:** ✅ Navigation system fully operational!
