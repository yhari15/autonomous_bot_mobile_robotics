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

## Navigation Deep Dive: How the Robot Actually Moves

### The Big Picture: Waypoints + Sensor Feedback

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                    NAVIGATION IS A HYBRID APPROACH                               │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│   MANUAL WAYPOINTS                    SENSOR-BASED EXECUTION                     │
│   (What to do)                        (How to do it)                             │
│                                                                                  │
│   ┌─────────────────┐                ┌─────────────────┐                        │
│   │ navigation_     │                │ LiDAR (/scan)   │                        │
│   │ waypoints.yaml  │                │ Odometry (/odom)│                        │
│   │                 │                │ IMU             │                        │
│   │ • Goal 1: x,y   │                │ Camera          │                        │
│   │ • Goal 2: x,y   │                └────────┬────────┘                        │
│   │ • Goal 3: x,y   │                         │                                 │
│   └────────┬────────┘                         │                                 │
│            │                                  ▼                                 │
│            │                         ┌─────────────────┐                        │
│            │                         │ NAV2 uses these │                        │
│            └────────────────────────▶│ to SAFELY reach │                        │
│              "Go to this point"      │ the waypoint    │                        │
│                                      └─────────────────┘                        │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘

So: WAYPOINTS define WHERE to go, SENSORS define HOW to get there safely.
```

---

### Sensor Data Flow (How Robot Perceives World)

```
GAZEBO SIMULATION                           ROS2 NODES
═══════════════════                         ══════════════════

┌─────────────────┐                         
│   Mars World    │                         
│  ┌───────────┐  │                         
│  │  Table    │  │                         
│  │  Chairs   │  │                         
│  │  Boulders │  │                         
│  └───────────┘  │                         
│                 │                         
│  ┌───────────┐  │                         
│  │ Husky UR3 │  │                         
│  │  Robot    │──┼────────────────────────────────────────┐
│  └───────────┘  │                                        │
│    │  │  │  │   │                                        │
└────┼──┼──┼──┼───┘                                        │
     │  │  │  │                                            │
     │  │  │  │  ros_gz_bridge (translates Gazebo↔ROS2)    │
     │  │  │  │         │                                  │
     ▼  ▼  ▼  ▼         ▼                                  │
┌────────────────────────────────────────────────────────────────────┐
│                      SENSOR TOPICS                                  │
├────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  /scan (LaserScan)              /odom (Odometry)                   │
│  ┌─────────────────────┐        ┌─────────────────────┐            │
│  │ 2D LiDAR data       │        │ Wheel encoder data  │            │
│  │ • 360° scan         │        │ • Position x,y,z    │            │
│  │ • Range to objects  │        │ • Orientation quat  │            │
│  │ • Detect obstacles  │        │ • Velocity linear   │            │
│  │   (chairs, table)   │        │ • Velocity angular  │            │
│  └──────────┬──────────┘        └──────────┬──────────┘            │
│             │                              │                        │
│             ▼                              ▼                        │
│      ┌─────────────┐              ┌─────────────────┐              │
│      │ COSTMAP     │              │ STATE ESTIMATION│              │
│      │ (Nav2)      │              │ (SLAM/EKF)      │              │
│      └─────────────┘              └─────────────────┘              │
│                                                                     │
│  /zed_base/left/image_raw        /joint_states                     │
│  ┌─────────────────────┐        ┌─────────────────────┐            │
│  │ Camera images       │        │ Arm joint positions │            │
│  │ • RGB frames        │        │ • 6 UR3 joints      │            │
│  │ • For 3D mapping    │        │ • Gripper joints    │            │
│  └─────────────────────┘        └─────────────────────┘            │
│                                                                     │
└────────────────────────────────────────────────────────────────────┘
```

---

### Localization: Where Am I? (SLAM + TF)

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                         LOCALIZATION SYSTEM                                      │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│  THE PROBLEM: Robot needs to know its position in the world                     │
│                                                                                  │
│  THE SOLUTION: SLAM Toolbox + TF Transform Tree                                 │
│                                                                                  │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │                           SLAM TOOLBOX                                   │    │
│  │                      (async_slam_toolbox_node)                          │    │
│  │                                                                          │    │
│  │    Input:                          Output:                               │    │
│  │    ┌──────────────┐               ┌──────────────┐                      │    │
│  │    │ /scan        │──────────────▶│ /map         │ (OccupancyGrid)     │    │
│  │    │ (LiDAR data) │               │              │ Where are walls?    │    │
│  │    └──────────────┘               └──────────────┘                      │    │
│  │    ┌──────────────┐               ┌──────────────┐                      │    │
│  │    │ /odom        │──────────────▶│ map→odom TF  │ Corrects drift      │    │
│  │    │ (wheel odom) │               │              │                      │    │
│  │    └──────────────┘               └──────────────┘                      │    │
│  │                                                                          │    │
│  │    SLAM = Simultaneous Localization And Mapping                         │    │
│  │    • Builds map from LiDAR scans                                        │    │
│  │    • Tracks robot position in that map                                  │    │
│  │    • Corrects odometry drift using scan matching                        │    │
│  │                                                                          │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
│                                                                                  │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │                        TF TRANSFORM TREE                                 │    │
│  │                                                                          │    │
│  │                         map                                              │    │
│  │                          │                                               │    │
│  │                          │ (SLAM publishes this)                         │    │
│  │                          ▼                                               │    │
│  │                        odom                                              │    │
│  │                          │                                               │    │
│  │                          │ (odom_to_tf_node publishes this)              │    │
│  │                          ▼                                               │    │
│  │                      base_link ◄─── Robot's body center                  │    │
│  │                      /   |   \                                           │    │
│  │                     /    |    \                                          │    │
│  │                    ▼     ▼     ▼                                         │    │
│  │              base_laser  ur3_   zed_base_                                │    │
│  │              (LiDAR)   base_link  camera                                 │    │
│  │                          │                                               │    │
│  │                          ▼ (6 joints)                                    │    │
│  │                     wrist_3_link                                         │    │
│  │                          │                                               │    │
│  │                          ▼                                               │    │
│  │                    zed_arm_camera                                        │    │
│  │                                                                          │    │
│  │    robot_state_publisher: Publishes URDF joint transforms               │    │
│  │    odom_to_tf_node: Converts /odom topic to odom→base_link TF           │    │
│  │                                                                          │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘

NOTE: This project uses SLAM Toolbox instead of EKF (robot_localization).
      EKF would fuse multiple sensors (IMU, odom, GPS) for better estimates,
      but for this simulation, wheel odometry + SLAM is sufficient.
```

---

### Nav2: The Brain of Navigation

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              NAV2 ARCHITECTURE                                   │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│  waypoint_navigator_node                                                        │
│         │                                                                        │
│         │ NavigateToPose Action                                                  │
│         │ (goal: x=3.0, y=1.5, yaw=0)                                           │
│         ▼                                                                        │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │                         BT_NAVIGATOR                                     │    │
│  │                    (Behavior Tree Navigator)                            │    │
│  │                                                                          │    │
│  │    ┌─────────────────────────────────────────────────────────┐          │    │
│  │    │                   Behavior Tree                          │          │    │
│  │    │                                                          │          │    │
│  │    │    NavigateToPose                                        │          │    │
│  │    │         │                                                │          │    │
│  │    │         ├── ComputePathToPose ──▶ Call Planner           │          │    │
│  │    │         │                                                │          │    │
│  │    │         └── FollowPath ──────────▶ Call Controller       │          │    │
│  │    │               │                                          │          │    │
│  │    │               └── Recovery (if stuck)                    │          │    │
│  │    │                     ├── Spin                             │          │    │
│  │    │                     ├── BackUp                           │          │    │
│  │    │                     └── Wait                             │          │    │
│  │    │                                                          │          │    │
│  │    └─────────────────────────────────────────────────────────┘          │    │
│  └────────────────────────────────┬────────────────────────────────────────┘    │
│                                   │                                              │
│           ┌───────────────────────┼───────────────────────┐                     │
│           │                       │                       │                     │
│           ▼                       ▼                       ▼                     │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐             │
│  │ PLANNER_SERVER  │    │CONTROLLER_SERVER│    │RECOVERIES_SERVER│             │
│  │                 │    │                 │    │                 │             │
│  │ NavFn or Smac   │    │ DWB Local       │    │ Spin, Backup    │             │
│  │ Planner         │    │ Planner         │    │ Wait, Clear     │             │
│  │                 │    │                 │    │                 │             │
│  │ Input:          │    │ Input:          │    │ When robot is   │             │
│  │ • Start pose    │    │ • Global path   │    │ stuck, these    │             │
│  │ • Goal pose     │    │ • Current odom  │    │ behaviors try   │             │
│  │ • Global costmap│    │ • Local costmap │    │ to recover      │             │
│  │                 │    │                 │    │                 │             │
│  │ Output:         │    │ Output:         │    │                 │             │
│  │ • Path (poses)  │    │ • /cmd_vel      │    │                 │             │
│  │                 │    │   (Twist msg)   │    │                 │             │
│  └─────────────────┘    └────────┬────────┘    └─────────────────┘             │
│                                  │                                              │
│                                  ▼                                              │
│                        ┌─────────────────┐                                      │
│                        │   /cmd_vel      │                                      │
│                        │ linear.x: 0.26  │                                      │
│                        │ angular.z: 0.1  │                                      │
│                        └────────┬────────┘                                      │
│                                 │                                               │
│                                 │ ros_gz_bridge                                 │
│                                 ▼                                               │
│                        ┌─────────────────┐                                      │
│                        │ GAZEBO PHYSICS  │                                      │
│                        │ Moves robot     │                                      │
│                        └─────────────────┘                                      │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

### Costmaps: How Nav2 Sees Obstacles

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              COSTMAP SYSTEM                                      │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│  GLOBAL COSTMAP                           LOCAL COSTMAP                         │
│  (For path planning)                      (For real-time avoidance)             │
│                                                                                  │
│  ┌─────────────────────────┐              ┌─────────────────────────┐           │
│  │ ░░░░░░░░░░░░░░░░░░░░░░░ │              │                         │           │
│  │ ░░░░░░░░░░░░░░░░░░░░░░░ │              │      ┌─────┐            │           │
│  │ ░░░░░░█████░░░░░░░░░░░░ │              │      │█████│ ← Chair    │           │
│  │ ░░░░░░█TABLE█░░░░░░░░░░ │              │      │█████│   detected │           │
│  │ ░░░░░░█████░░░░░░░░░░░░ │              │      └─────┘   by LiDAR │           │
│  │ ░░░░░░░░░░░░░░░░░░░░░░░ │              │                         │           │
│  │ ░░░░░░░░░░░░░░░░░░░░░░░ │              │    ★ Robot              │           │
│  │ ░░★░░░░░░░░░░░░░░░░░░░░ │              │    (center)             │           │
│  │ Start                   │              │                         │           │
│  └─────────────────────────┘              └─────────────────────────┘           │
│                                                                                  │
│  • Size: Entire map (10m x 10m)           • Size: Around robot (3m x 3m)        │
│  • Updates: Slower                        • Updates: Fast (real-time)           │
│  • Used by: Global planner                • Used by: Local planner              │
│  • Source: SLAM map + static obstacles    • Source: LiDAR /scan                 │
│                                                                                  │
│  COST VALUES:                                                                    │
│  ┌────────────────────────────────────────────────────────────────────┐         │
│  │  0      = Free space (safe to drive)                               │         │
│  │  1-252  = Inflation zone (getting close to obstacle)               │         │
│  │  253    = Inscribed (robot would touch obstacle)                   │         │
│  │  254    = Lethal (obstacle detected)                               │         │
│  │  255    = Unknown                                                  │         │
│  └────────────────────────────────────────────────────────────────────┘         │
│                                                                                  │
│  The planner finds paths through LOW COST areas, avoiding HIGH COST areas.      │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

### MoveIt2: Arm Control (Currently Minimal)

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                             MOVEIT2 INTEGRATION                                  │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│  STATUS: Configured but minimally used in this project.                         │
│          The arm is primarily set to fixed poses, not dynamic motion planning.  │
│                                                                                  │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │                        WHAT MOVEIT2 CAN DO                               │    │
│  │                                                                          │    │
│  │  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐                │    │
│  │  │ User says:  │     │ MoveIt2:    │     │ Result:     │                │    │
│  │  │ "Move arm   │────▶│ Plan path   │────▶│ Smooth arm  │                │    │
│  │  │  to pose X" │     │ avoiding    │     │ motion      │                │    │
│  │  │             │     │ collisions  │     │             │                │    │
│  │  └─────────────┘     └─────────────┘     └─────────────┘                │    │
│  │                                                                          │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
│                                                                                  │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │                    HOW IT'S CONFIGURED HERE                              │    │
│  │                                                                          │    │
│  │  Files:                                                                  │    │
│  │  ├── ur3_moveit.srdf         # Semantic robot description               │    │
│  │  │   • Defines "ur3_arm" planning group                                 │    │
│  │  │   • Defines "gripper" end effector                                   │    │
│  │  │   • Self-collision matrix                                            │    │
│  │  │                                                                       │    │
│  │  ├── moveit_controllers.yaml  # Which controllers to use                │    │
│  │  │   • Maps to arm_trajectory_bridge                                    │    │
│  │  │                                                                       │    │
│  │  └── arm_trajectory_bridge.py # Custom bridge                           │    │
│  │      • Receives FollowJointTrajectory actions                           │    │
│  │      • Sends joint positions to Gazebo                                  │    │
│  │                                                                          │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
│                                                                                  │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │                      ARM CONTROL DATA FLOW                               │    │
│  │                                                                          │    │
│  │  MoveIt2                arm_trajectory_bridge              Gazebo        │    │
│  │  ┌───────────┐          ┌───────────────────┐          ┌───────────┐    │    │
│  │  │ Plan arm  │          │ Receive trajectory│          │ Physics   │    │    │
│  │  │ trajectory│─────────▶│ Extract positions │─────────▶│ moves arm │    │    │
│  │  │           │ Action   │ Publish to topics │ Topics   │           │    │    │
│  │  └───────────┘          └───────────────────┘          └───────────┘    │    │
│  │                                                                          │    │
│  │  Topics published by arm_trajectory_bridge:                             │    │
│  │  • /ur3_joint1_cmd (Float64) ──▶ Shoulder pan                           │    │
│  │  • /ur3_joint2_cmd (Float64) ──▶ Shoulder lift                          │    │
│  │  • /ur3_joint3_cmd (Float64) ──▶ Elbow                                  │    │
│  │  • /ur3_joint4_cmd (Float64) ──▶ Wrist 1                                │    │
│  │  • /ur3_joint5_cmd (Float64) ──▶ Wrist 2                                │    │
│  │  • /ur3_joint6_cmd (Float64) ──▶ Wrist 3                                │    │
│  │                                                                          │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
│                                                                                  │
│  CURRENT USAGE IN THIS PROJECT:                                                 │
│  • Arm stays in fixed "mapping pose" during navigation                         │
│  • Camera on wrist captures images at each waypoint                            │
│  • No dynamic arm motion planning needed for this task                         │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

### Complete Integration Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                    HOW ALL MODULES WORK TOGETHER                                 │
└─────────────────────────────────────────────────────────────────────────────────┘

                              ┌─────────────────┐
                              │  YAML CONFIG    │
                              │ (navigation_    │
                              │  waypoints.yaml)│
                              └────────┬────────┘
                                       │
                                       ▼
┌──────────────────┐         ┌─────────────────┐
│ waypoint_        │         │                 │
│ navigator_node   │◀────────│ "Go to x,y"     │
│                  │         │                 │
└────────┬─────────┘         └─────────────────┘
         │
         │ NavigateToPose
         │ Action
         ▼
┌─────────────────────────────────────────────────────────────────────┐
│                            NAV2                                      │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐         │
│  │ BT Navigator   │──│ Planner Server │──│ Controller     │         │
│  │ (coordinates)  │  │ (global path)  │  │ Server (/cmd)  │         │
│  └────────────────┘  └───────▲────────┘  └───────┬────────┘         │
│                              │                   │                   │
│                     ┌────────┴────────┐          │                   │
│                     │ Global Costmap  │          │                   │
│                     │ (from SLAM map) │          │                   │
│                     └────────▲────────┘          │                   │
│                              │                   │                   │
└──────────────────────────────┼───────────────────┼───────────────────┘
                               │                   │
                               │                   │
         ┌─────────────────────┘                   │
         │                                         │
         │                                         ▼ /cmd_vel
┌────────┴──────────────────────┐        ┌─────────────────┐
│         SLAM TOOLBOX          │        │  ros_gz_bridge  │
│  ┌─────────────────────────┐  │        └────────┬────────┘
│  │ Input: /scan, /odom     │  │                 │
│  │ Output: /map, map→odom  │  │                 ▼
│  └─────────────────────────┘  │        ┌─────────────────┐
└───────────────▲───────────────┘        │     GAZEBO      │
                │                        │                 │
                │                        │  ┌───────────┐  │
    ┌───────────┴───────────┐            │  │  Husky    │  │
    │                       │            │  │  Robot    │──┼──▶ Moves
    │                       │            │  └───────────┘  │
    │                       │            │                 │
┌───┴───────────┐   ┌───────┴───────┐    │  Sensors:       │
│  /scan        │   │  /odom        │    │  • LiDAR        │
│  (LiDAR)      │   │  (Wheels)     │    │  • Cameras      │
└───────────────┘   └───────────────┘    │  • Wheels       │
        ▲                   ▲            │                 │
        │                   │            └─────────────────┘
        │                   │                    │
        └───────────────────┴────────────────────┘
                    ros_gz_bridge
                (Gazebo → ROS2 topics)
```

---

### Key Insight: Waypoints vs Sensors

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                                                                                  │
│   Q: Is navigation based on sensor input or manually written waypoints?         │
│                                                                                  │
│   A: BOTH!                                                                       │
│                                                                                  │
│   ┌─────────────────────────────────────────────────────────────────────────┐   │
│   │                                                                          │   │
│   │  WAYPOINTS (Manual)              SENSORS (Automatic)                    │   │
│   │  ═══════════════════             ════════════════════                   │   │
│   │                                                                          │   │
│   │  • Define HIGH-LEVEL goals       • Enable LOW-LEVEL execution           │   │
│   │  • "Go to the table"             • "Avoid the chair in front"           │   │
│   │  • Set by the programmer         • Detected in real-time                │   │
│   │  • Static (in YAML file)         • Dynamic (changes as robot moves)     │   │
│   │                                                                          │   │
│   │  Without waypoints:              Without sensors:                        │   │
│   │  Robot doesn't know              Robot would drive                       │   │
│   │  where to go                     straight into obstacles                 │   │
│   │                                                                          │   │
│   └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                  │
│   ANALOGY:                                                                       │
│   ┌─────────────────────────────────────────────────────────────────────────┐   │
│   │  Waypoints = GPS destination ("Navigate to Starbucks")                  │   │
│   │  Sensors   = Your eyes ("Don't hit that pedestrian!")                   │   │
│   │                                                                          │   │
│   │  You need BOTH to drive safely to your destination.                     │   │
│   └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

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

