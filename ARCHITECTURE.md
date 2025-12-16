# Mars Table Mapping - System Architecture

## Overview

This project implements autonomous navigation and 3D reconstruction of a table in a Mars environment using a Husky robot with a UR3 arm equipped with stereo cameras.

---

## System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              GAZEBO IGNITION                                     │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │                      Mars Thanksgiving World                             │    │
│  │   ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐               │    │
│  │   │  Table   │  │ Boulders │  │  Chairs  │  │  Terrain │               │    │
│  │   └──────────┘  └──────────┘  └──────────┘  └──────────┘               │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
│                                                                                  │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │                         Husky UR3 Robot                                  │    │
│  │   ┌────────────┐    ┌────────────┐    ┌────────────┐                    │    │
│  │   │   Husky    │───▶│  UR3 Arm   │───▶│  Gripper   │                    │    │
│  │   │   Base     │    │ (6 joints) │    │(RH-P12-RN) │                    │    │
│  │   └────────────┘    └────────────┘    └────────────┘                    │    │
│  │         │                 │                                              │    │
│  │   ┌─────┴─────┐    ┌─────┴─────┐                                        │    │
│  │   │ ZED Base  │    │ ZED Arm   │                                        │    │
│  │   │  Camera   │    │  Camera   │                                        │    │
│  │   │(forward)  │    │(forward)  │                                        │    │
│  │   └───────────┘    └───────────┘                                        │    │
│  │         │                 │                                              │    │
│  │   ┌─────┴─────┐    ┌─────┴─────┐                                        │    │
│  │   │  LiDAR    │    │    GPS    │                                        │    │
│  │   │  Scanner  │    │  Sensor   │                                        │    │
│  │   └───────────┘    └───────────┘                                        │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      │ ros_gz_bridge
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                               ROS2 HUMBLE                                        │
│                                                                                  │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │                          Topic Bridge                                     │   │
│  │  /cmd_vel  /odom  /scan  /joint_states  /zed_*/image_raw  /clock        │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│         │         │      │         │              │                              │
│         ▼         ▼      ▼         ▼              ▼                              │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │                        Core Nodes                                        │    │
│  │                                                                          │    │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐         │    │
│  │  │ robot_state_    │  │   odom_to_tf    │  │  temp_map_      │         │    │
│  │  │   publisher     │  │     _node       │  │   publisher     │         │    │
│  │  │                 │  │                 │  │                 │         │    │
│  │  │ URDF → TF tree  │  │ /odom → TF      │  │ Empty map for   │         │    │
│  │  │ (static joints) │  │ odom→base_link  │  │ Nav2 bootstrap  │         │    │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────┘         │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
│                                                                                  │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │                      Navigation Stack (Nav2)                             │    │
│  │                                                                          │    │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐         │    │
│  │  │  SLAM Toolbox   │  │   Nav2 Stack    │  │    Waypoint     │         │    │
│  │  │                 │  │                 │  │    Navigator    │         │    │
│  │  │ /scan → /map    │  │ Path planning   │  │                 │         │    │
│  │  │ Localization    │  │ Obstacle avoid  │  │ YAML waypoints  │         │    │
│  │  │                 │  │ /cmd_vel output │  │ → Nav2 goals    │         │    │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────┘         │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
│                                                                                  │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │                    3D Reconstruction Pipeline                            │    │
│  │                                                                          │    │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐         │    │
│  │  │  stereo_depth   │  │  Point Cloud    │  │   PCD Export    │         │    │
│  │  │     _node       │  │  Accumulator    │  │    Service      │         │    │
│  │  │                 │  │                 │  │                 │         │    │
│  │  │ ZED L+R images  │  │ Frame-by-frame  │  │ /must3r/save_pcd│         │    │
│  │  │ → Disparity     │  │ accumulation    │  │ → .pcd file     │         │    │
│  │  │ → Depth         │  │ in base_link    │  │                 │         │    │
│  │  │ → PointCloud2   │  │                 │  │                 │         │    │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────┘         │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                               OUTPUT FILES                                       │
│                                                                                  │
│   ~/ros2_ws/pointclouds/table_map_YYYYMMDD_HHMMSS.pcd                           │
│   ~/ros2_ws/must3r_captures/wp*_base.jpg, wp*_arm.jpg                           │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## Package Structure

```
ros2_ws/
├── src/
│   ├── husky_ur3_description/          # Robot model
│   │   ├── urdf/
│   │   │   ├── husky_ur3_gripper.urdf.xacro    # Main robot URDF
│   │   │   ├── sensors.urdf.xacro              # GPS + ZED cameras
│   │   │   └── zed2_camera.urdf.xacro          # Stereo camera macro
│   │   ├── config/
│   │   │   ├── ur3_moveit.srdf                 # MoveIt config
│   │   │   └── moveit_controllers.yaml         # Arm controllers
│   │   └── launch/
│   │       ├── gazebo_sim.launch.py            # Basic simulation
│   │       └── moveit.launch.py                # MoveIt launch
│   │
│   ├── must3r_nav/                     # Navigation & 3D reconstruction
│   │   ├── must3r_nav/
│   │   │   ├── waypoint_navigator_node.py      # Main navigation
│   │   │   ├── stereo_depth_node.py            # 3D reconstruction
│   │   │   ├── odom_to_tf_node.py              # TF publisher
│   │   │   ├── temp_map_publisher.py           # Empty map for Nav2
│   │   │   └── must3r_bridge_node.py           # MUSt3R bridge
│   │   ├── config/
│   │   │   ├── navigation_waypoints.yaml       # Waypoint definitions
│   │   │   └── mars_nav2_params.yaml           # Nav2 parameters
│   │   ├── worlds/
│   │   │   └── mars_thanksgiving.world         # Gazebo world file
│   │   └── launch/
│   │       └── mars_nav2.launch.py             # Full system launch
│   │
│   └── husky/                          # Husky base packages
│       └── ...
│
├── run_mapping.sh                      # Main execution script
├── test_nav_goal.py                    # Navigation test
└── check_gripper.py                    # Gripper test
```

---

## Data Flow

```
                    ┌──────────────┐
                    │   Gazebo     │
                    │  Simulation  │
                    └──────┬───────┘
                           │
           ┌───────────────┼───────────────┐
           │               │               │
           ▼               ▼               ▼
    ┌──────────┐    ┌──────────┐    ┌──────────┐
    │  /odom   │    │  /scan   │    │ /zed_*/  │
    │          │    │          │    │ image_raw│
    └────┬─────┘    └────┬─────┘    └────┬─────┘
         │               │               │
         ▼               ▼               ▼
    ┌──────────┐    ┌──────────┐    ┌──────────┐
    │odom_to_tf│    │   SLAM   │    │ stereo_  │
    │          │    │ Toolbox  │    │  depth   │
    └────┬─────┘    └────┬─────┘    └────┬─────┘
         │               │               │
         ▼               ▼               ▼
    ┌──────────┐    ┌──────────┐    ┌──────────┐
    │   /tf    │    │   /map   │    │/stereo/  │
    │          │    │          │    │point_cloud
    └────┬─────┘    └────┬─────┘    └────┬─────┘
         │               │               │
         └───────────────┼───────────────┘
                         │
                         ▼
                  ┌──────────────┐
                  │    Nav2      │
                  │    Stack     │
                  └──────┬───────┘
                         │
                         ▼
                  ┌──────────────┐
                  │  /cmd_vel    │
                  │  (velocity)  │
                  └──────┬───────┘
                         │
                         ▼
                  ┌──────────────┐
                  │   Gazebo     │
                  │ (robot moves)│
                  └──────────────┘
```

---

## TF Tree

```
map
 └── odom                          (from SLAM / odom_to_tf)
      └── base_link                (robot base)
           ├── front_left_wheel
           ├── front_right_wheel
           ├── rear_left_wheel
           ├── rear_right_wheel
           ├── top_plate_link
           │    ├── gps_link
           │    ├── zed_base_camera_base_link
           │    │    ├── zed_base_left_camera_frame
           │    │    │    └── zed_base_left_camera_optical_frame
           │    │    └── zed_base_right_camera_frame
           │    │         └── zed_base_right_camera_optical_frame
           │    └── ur3_base_link
           │         └── ur3_joint1 → ... → wrist_3_link
           │                                  ├── rh_p12_rn_base (gripper)
           │                                  └── zed_arm_camera_base_link
           │                                       ├── zed_arm_left_camera_frame
           │                                       └── zed_arm_right_camera_frame
           └── base_laser (LiDAR)
```

---

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands |
| `/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan data |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM-generated map |
| `/zed_base/left/image_raw` | `sensor_msgs/Image` | Base camera left |
| `/zed_base/right/image_raw` | `sensor_msgs/Image` | Base camera right |
| `/zed_arm/left/image_raw` | `sensor_msgs/Image` | Arm camera left |
| `/zed_arm/right/image_raw` | `sensor_msgs/Image` | Arm camera right |
| `/stereo/point_cloud` | `sensor_msgs/PointCloud2` | 3D point cloud |
| `/joint_states` | `sensor_msgs/JointState` | All joint positions |
| `/tf` | `tf2_msgs/TFMessage` | Coordinate transforms |

---

## Key Services

| Service | Type | Description |
|---------|------|-------------|
| `/must3r/save_pcd` | `std_srvs/Trigger` | Save accumulated point cloud to PCD |
| `/must3r/clear_points` | `std_srvs/Trigger` | Clear accumulated points |

---

## Execution Flow

```
1. Launch Simulation
   └── mars_nav2.launch.py
        ├── Gazebo (mars_thanksgiving.world)
        ├── robot_state_publisher (URDF → TF)
        ├── ros_gz_bridge (Gazebo ↔ ROS2)
        ├── odom_to_tf_node (odom → TF)
        ├── SLAM Toolbox (mapping)
        └── Nav2 Stack (navigation)

2. Start 3D Reconstruction
   └── stereo_depth_node
        ├── Subscribe to ZED cameras
        ├── Compute stereo disparity
        ├── Generate point clouds
        └── Accumulate points

3. Navigate (Manual or Autonomous)
   ├── Teleop: ros2 run teleop_twist_keyboard teleop_twist_keyboard
   └── Auto:   ros2 run must3r_nav waypoint_navigator

4. Save Output
   └── ros2 service call /must3r/save_pcd std_srvs/srv/Trigger
        └── ~/ros2_ws/pointclouds/table_map_*.pcd
```

---

---

## Operational Sequence Diagram (Start to Finish)

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                         PHASE 1: SYSTEM STARTUP                                  │
└─────────────────────────────────────────────────────────────────────────────────┘

User runs: ./run_mapping.sh
                │
                ▼
┌───────────────────────────────────────┐
│ 1. CLEANUP OLD PROCESSES              │
│    pkill ros2, ign gazebo, gzserver   │
│    Wait 3 seconds                     │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│ 2. LAUNCH GAZEBO SIMULATION           │
│    mars_nav2.launch.py                │
│    ┌─────────────────────────────┐    │
│    │ • Load mars_thanksgiving    │    │
│    │   world (table, boulders)   │    │
│    │ • Spawn Husky UR3 robot     │    │
│    │ • Start physics engine      │    │
│    └─────────────────────────────┘    │
└───────────────────┬───────────────────┘
                    │ (wait 5s)
                    ▼
┌───────────────────────────────────────┐
│ 3. START ROS-GAZEBO BRIDGE            │
│    ros_gz_bridge                      │
│    ┌─────────────────────────────┐    │
│    │ Bridge topics:              │    │
│    │ • /clock                    │    │
│    │ • /cmd_vel                  │    │
│    │ • /odom                     │    │
│    │ • /scan                     │    │
│    │ • /joint_states             │    │
│    │ • /zed_base/* (cameras)     │    │
│    │ • /zed_arm/* (cameras)      │    │
│    └─────────────────────────────┘    │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│ 4. START CORE ROS2 NODES              │
│    ┌─────────────────────────────┐    │
│    │ robot_state_publisher       │    │
│    │ → Publishes URDF TF tree    │    │
│    ├─────────────────────────────┤    │
│    │ odom_to_tf_node             │    │
│    │ → /odom → odom→base_link TF │    │
│    ├─────────────────────────────┤    │
│    │ temp_map_publisher          │    │
│    │ → Empty map for Nav2 init   │    │
│    └─────────────────────────────┘    │
└───────────────────┬───────────────────┘
                    │ (wait 5s)
                    ▼
┌───────────────────────────────────────┐
│ 5. START SLAM TOOLBOX                 │
│    ┌─────────────────────────────┐    │
│    │ Input: /scan (LiDAR)        │    │
│    │ Output: /map, map→odom TF   │    │
│    │ Mode: Online async SLAM     │    │
│    └─────────────────────────────┘    │
└───────────────────┬───────────────────┘
                    │ (wait 5s)
                    ▼
┌───────────────────────────────────────┐
│ 6. START NAV2 STACK                   │
│    ┌─────────────────────────────┐    │
│    │ • bt_navigator (behavior)   │    │
│    │ • planner_server (paths)    │    │
│    │ • controller_server (drive) │    │
│    │ • recoveries_server         │    │
│    │ • lifecycle_manager         │    │
│    └─────────────────────────────┘    │
└───────────────────┬───────────────────┘
                    │ (wait 10s for Nav2 ready)
                    ▼
┌───────────────────────────────────────┐
│ 7. PUBLISH INITIAL POSE               │
│    ┌─────────────────────────────┐    │
│    │ Tell Nav2 where robot is:   │    │
│    │ x=0, y=0, yaw=0 in map      │    │
│    └─────────────────────────────┘    │
└───────────────────┬───────────────────┘
                    │
                    ▼
        ┌───────────────────────┐
        │  SYSTEM READY ✓       │
        │  (30 seconds elapsed) │
        └───────────┬───────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                      PHASE 2: 3D RECONSTRUCTION START                            │
└─────────────────────────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│ 8. START STEREO DEPTH NODE            │
│    ros2 run must3r_nav stereo_depth   │
│    ┌─────────────────────────────┐    │
│    │ Subscribe to:               │    │
│    │ • /zed_base/left/image_raw  │    │
│    │ • /zed_base/right/image_raw │    │
│    │ • /zed_arm/left/image_raw   │    │
│    │ • /zed_arm/right/image_raw  │    │
│    │                             │    │
│    │ Processing (2 Hz):          │    │
│    │ • Stereo matching (SGBM)    │    │
│    │ • Disparity → Depth         │    │
│    │ • Depth → Point Cloud       │    │
│    │ • Accumulate points         │    │
│    │                             │    │
│    │ Publish to:                 │    │
│    │ • /stereo/point_cloud       │    │
│    └─────────────────────────────┘    │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                       PHASE 3: NAVIGATION EXECUTION                              │
└─────────────────────────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│ 9. START WAYPOINT NAVIGATOR           │
│    ros2 run must3r_nav waypoint_nav   │
│    ┌─────────────────────────────┐    │
│    │ Load: navigation_waypoints  │    │
│    │       .yaml (5 waypoints)   │    │
│    │                             │    │
│    │ WP1: Start (0,0)            │    │
│    │ WP2: Approach (1.5, 0.5)    │    │
│    │ WP3: Table view (3.0, 1.5)  │    │
│    │ WP4: East (4.5, 0.5)        │    │
│    │ WP5: End (5.5, 0)           │    │
│    └─────────────────────────────┘    │
└───────────────────┬───────────────────┘
                    │
                    ▼
        ┌───────────────────────────────────────────────────────────┐
        │                   FOR EACH WAYPOINT                        │
        └───────────────────────────────┬───────────────────────────┘
                                        │
                    ┌───────────────────┴───────────────────┐
                    │                                       │
                    ▼                                       │
    ┌───────────────────────────────────┐                  │
    │ 10. SEND GOAL TO NAV2             │                  │
    │     NavigateToPose action         │                  │
    │     ┌─────────────────────────┐   │                  │
    │     │ Goal: PoseStamped       │   │                  │
    │     │ • frame_id: "map"       │   │                  │
    │     │ • position: x, y        │   │                  │
    │     │ • orientation: yaw      │   │                  │
    │     └─────────────────────────┘   │                  │
    └───────────────┬───────────────────┘                  │
                    │                                       │
                    ▼                                       │
    ┌───────────────────────────────────┐                  │
    │ 11. NAV2 PLANS PATH               │                  │
    │     ┌─────────────────────────┐   │                  │
    │     │ NavFn/Smac Planner      │   │                  │
    │     │ • A* on costmap         │   │                  │
    │     │ • Obstacle avoidance    │   │                  │
    │     │ • Generate path         │   │                  │
    │     └─────────────────────────┘   │                  │
    └───────────────┬───────────────────┘                  │
                    │                                       │
                    ▼                                       │
    ┌───────────────────────────────────┐                  │
    │ 12. ROBOT FOLLOWS PATH            │                  │
    │     ┌─────────────────────────┐   │                  │
    │     │ DWB Local Planner       │   │                  │
    │     │ • Track path            │   │                  │
    │     │ • Avoid obstacles       │   │                  │
    │     │ • Publish /cmd_vel      │   │                  │
    │     │   → Gazebo moves robot  │   │                  │
    │     └─────────────────────────┘   │                  │
    │                                   │                  │
    │     MEANWHILE (parallel):         │                  │
    │     ┌─────────────────────────┐   │                  │
    │     │ stereo_depth_node       │   │                  │
    │     │ • Captures images       │   │                  │
    │     │ • Generates points      │   │                  │
    │     │ • Accumulates cloud     │   │                  │
    │     └─────────────────────────┘   │                  │
    └───────────────┬───────────────────┘                  │
                    │                                       │
                    ▼                                       │
    ┌───────────────────────────────────┐                  │
    │ 13. WAYPOINT REACHED              │                  │
    │     ┌─────────────────────────┐   │                  │
    │     │ Nav2 returns SUCCESS    │   │                  │
    │     │ Wait 2s (capture_delay) │   │                  │
    │     │ Capture images to disk  │   │                  │
    │     │ • wp01_base.jpg         │   │                  │
    │     │ • wp01_arm.jpg          │   │                  │
    │     └─────────────────────────┘   │                  │
    └───────────────┬───────────────────┘                  │
                    │                                       │
                    └───────────────────────────────────────┘
                                        │
                                        │ (repeat for all waypoints)
                                        │
                                        ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                       PHASE 4: FINALIZATION                                      │
└─────────────────────────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│ 14. ALL WAYPOINTS COMPLETE            │
│     ┌─────────────────────────────┐   │
│     │ Navigation finished!        │   │
│     │ Wait 10s for final points   │   │
│     └─────────────────────────────┘   │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│ 15. SAVE POINT CLOUD                  │
│     Call: /must3r/save_pcd service    │
│     ┌─────────────────────────────┐   │
│     │ stereo_depth_node:          │   │
│     │ • Combine all accumulated   │   │
│     │   points                    │   │
│     │ • Voxel grid downsample     │   │
│     │   (2cm resolution)          │   │
│     │ • Write ASCII PCD file      │   │
│     └─────────────────────────────┘   │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│ 16. OUTPUT FILES GENERATED            │
│     ┌─────────────────────────────┐   │
│     │ ~/ros2_ws/pointclouds/      │   │
│     │   table_map_20241216.pcd    │   │
│     │                             │   │
│     │ ~/ros2_ws/must3r_captures/  │   │
│     │   wp01_base.jpg             │   │
│     │   wp01_arm.jpg              │   │
│     │   wp02_base.jpg             │   │
│     │   ...                       │   │
│     │   image_list.txt            │   │
│     └─────────────────────────────┘   │
└───────────────────┬───────────────────┘
                    │
                    ▼
        ┌───────────────────────────────┐
        │      MISSION COMPLETE ✓       │
        │                               │
        │  • Robot navigated start→end  │
        │  • Table mapped in 3D         │
        │  • PCD file ready to submit   │
        └───────────────────────────────┘
```

---

## Timeline Summary

```
Time (s)    Event
────────    ─────────────────────────────────────────
  0         User runs ./run_mapping.sh
  0-3       Cleanup old processes
  3         Launch Gazebo + mars_nav2.launch.py
  3-8       Gazebo loads world, spawns robot
  8         ros_gz_bridge starts
  8-13      Core nodes start (RSP, odom_to_tf, map)
  13        SLAM Toolbox starts
  18        Nav2 stack starts
  28        Initial pose published
  30        stereo_depth_node starts
  35        waypoint_navigator starts
  
  35-40     Navigate to WP1 (Start)
  40-60     Navigate to WP2 (Approach)
  60-90     Navigate to WP3 (Table view) ← Main capture
  90-110    Navigate to WP4 (East side)
  110-130   Navigate to WP5 (End)
  
  130-140   Wait for final point cloud processing
  140       Call /must3r/save_pcd
  141       PCD file written
  
  ~2.5 min  TOTAL MISSION TIME
```

---

## Quick Start

```bash
# Full automated pipeline
./run_mapping.sh

# OR Manual control
# Terminal 1: Simulation
ros2 launch must3r_nav mars_nav2.launch.py

# Terminal 2: 3D reconstruction
ros2 run must3r_nav stereo_depth

# Terminal 3: Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4: Save PCD when done
ros2 service call /must3r/save_pcd std_srvs/srv/Trigger
```

