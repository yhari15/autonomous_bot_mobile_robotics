# Mars Navigation Troubleshooting Guide

## Fixed Issues ✅

### 1. Module Import Error - FIXED
**Error:** `ModuleNotFoundError: No module named 'scripts.waypoint_navigator'`

**Fix Applied:**
- Moved waypoint_navigator to package directory
- Updated setup.py entry point
- Rebuilt package

**How to verify:**
```bash
source install/setup.bash
ros2 pkg executables must3r_nav | grep waypoint
# Should show: waypoint_navigator
```

### 2. Robot Not Spawning - FIXED
**Issue:** Robot doesn't appear in Gazebo

**Fix Applied:**
- Increased spawn delay from 5s to 8s
- Increased spawn height from 0.2m to 0.3m
- Adjusted Nav2 start time to 15s

---

## Current Testing Steps

### Step 1: Run Test Script
```bash
cd ~/ros2_ws
./test_mars_nav.sh
```

This will verify all packages and files are correctly installed.

### Step 2: Launch Mars Navigation

**Terminal 1 - Launch System:**
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch must3r_nav mars_nav2.launch.py
```

**What to watch for:**
1. Gazebo window opens (3 seconds)
2. Mars world loads with terrain visible
3. **WAIT 8 seconds** - Robot spawns at (-2, 0)
4. **WAIT 15 seconds** - Nav2 stack initializes

### Step 3: Verify Robot is Spawned

**In a new terminal:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check if robot description is published
ros2 topic echo /robot_description --once | head -20

# Check if joint states are being published
ros2 topic hz /joint_states

# Check if odometry is working
ros2 topic hz /odom

# List all nodes (should see robot_state_publisher)
ros2 node list
```

### Step 4: Run Waypoint Navigator

**Terminal 2 - After robot is visible:**
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run must3r_nav waypoint_navigator
```

---

## Common Issues and Solutions

### Issue: Robot Still Not Spawning

**Possible Causes:**

1. **Gazebo not fully initialized**
   - **Solution:** Wait longer (up to 20 seconds) before checking
   - **Test:** Look for robot model in Gazebo model tree (left panel)

2. **URDF/Robot description issue**
   - **Check:**
     ```bash
     ros2 topic echo /robot_description --once
     ```
   - **Should see:** XML robot description
   - **If empty:** Robot state publisher not running

3. **Spawn command timing**
   - **Check terminal output** for spawn errors
   - Look for: `[ros_gz_sim-...] [INFO] [...]`

**Manual spawn test:**
```bash
# Terminal 1: Just launch Gazebo
ign gazebo src/must3r_nav/worlds/mars_thanksgiving.world -r

# Terminal 2: Publish robot description
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro src/husky_ur3_description/urdf/husky_ur3_gripper.urdf.xacro)"

# Terminal 3: Manually spawn robot (after 10 seconds)
ros2 run ros_gz_sim create -topic robot_description -name husky_ur3 -x -2 -y 0 -z 0.5
```

### Issue: "Nav2 action server not available"

**Cause:** Nav2 stack not fully started

**Solution:**
1. Wait 20-25 seconds after launch
2. Check Nav2 nodes:
   ```bash
   ros2 node list | grep nav
   ```
3. Should see:
   - `/bt_navigator`
   - `/controller_server`
   - `/planner_server`
   - `/behavior_server`
   - `/waypoint_follower`

**If missing:**
- Check terminal for Nav2 errors
- Restart launch file

### Issue: Gazebo Appears Paused

**Symptoms:** Time stuck at 0:00:00

**Solution:** Click the Play (▶) button in Gazebo's bottom-left corner

### Issue: "No transform from map to base_link"

**Cause:** TF tree not complete

**Check TF tree:**
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

**Should see:** `map -> odom -> base_link`

**If broken:**
- Check odom_to_tf node is running
- Check static_transform_publisher is running
- Restart launch file

### Issue: Camera Topics Not Publishing

**Check:**
```bash
ros2 topic list | grep zed
ros2 topic hz /zed_base/left/image_raw
ros2 topic hz /zed_arm/left/image_raw
```

**If not publishing:**
- Robot not spawned yet
- Gazebo bridge not running
- Check bridge node: `ros2 node list | grep bridge`

### Issue: Laser Scanner Not Working

**Check:**
```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

**If not working:**
- Verify Gazebo bridge is running
- Check URDF has laser sensor plugin

---

## Debug Commands

### Check All Topics
```bash
ros2 topic list
```

### Check All Nodes
```bash
ros2 node list
```

### Check Nav2 Parameters
```bash
ros2 param list /controller_server
ros2 param get /controller_server use_sim_time
```

### Check TF Transforms
```bash
ros2 run tf2_ros tf2_echo map base_link
```

### Monitor Navigation
```bash
# Watch global path
ros2 topic echo /plan

# Watch velocity commands
ros2 topic echo /cmd_vel

# Watch current pose
ros2 topic echo /odom
```

---

## Performance Tips

### If Navigation is Too Slow
Edit `src/must3r_nav/config/mars_nav2_params.yaml`:
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 0.6  # Increase from 0.4
```

### If Robot Hits Obstacles
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      inflation_radius: 1.2  # Increase from 1.0
```

### If Arm Moves Too Slowly
Edit `src/must3r_nav/must3r_nav/waypoint_navigator_node.py`:
```python
point.time_from_start = Duration(seconds=3).to_msg()  # Reduce from 4
```

**After any changes:**
```bash
colcon build --packages-select must3r_nav
source install/setup.bash
```

---

## Getting Help

### Collect Debug Information

```bash
# Save all topics
ros2 topic list > topics.txt

# Save all nodes
ros2 node list > nodes.txt

# Save TF tree
ros2 run tf2_tools view_frames
# Creates frames.pdf

# Save launch terminal output
# Copy error messages from terminal
```

### Useful Log Files
- Build logs: `log/latest_build/must3r_nav/`
- Runtime logs: Check terminal output

---

## Clean Restart

If everything is broken:

```bash
# Kill all ROS/Gazebo processes
killall -9 ros2 ign gz gazebo ruby python3

# Clean build
cd ~/ros2_ws
rm -rf build/must3r_nav install/must3r_nav log/latest_build/must3r_nav

# Rebuild
source /opt/ros/humble/setup.bash
colcon build --packages-select must3r_nav --symlink-install

# Re-source
source install/setup.bash

# Launch again
ros2 launch must3r_nav mars_nav2.launch.py
```

---

## Success Checklist

Before running waypoint navigator, verify:

- [  ] Gazebo is running and showing Mars world
- [  ] Robot is visible in Gazebo at (-2, 0)
- [  ] Clock is advancing: `ros2 topic hz /clock`
- [  ] Odometry working: `ros2 topic hz /odom`
- [  ] Laser working: `ros2 topic hz /scan`
- [  ] Cameras working: `ros2 topic hz /zed_base/left/image_raw`
- [  ] Nav2 nodes running: `ros2 node list | grep nav`
- [  ] TF tree complete: `ros2 run tf2_ros tf2_echo map base_link`

If ALL checked ✓, you're ready to run the waypoint navigator!

---

**Status:** Fixes applied, ready for testing
