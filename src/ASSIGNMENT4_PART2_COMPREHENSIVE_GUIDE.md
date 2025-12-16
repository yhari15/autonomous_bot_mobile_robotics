# Assignment 4 Part 2: Mobile Manipulation with SLAM
## Comprehensive Understanding and Implementation Guide

**Course:** CS498GC Mobile Robotics - Fall 2025
**Due Date:** December 9, 2025 @ 11:00 PM
**Total Points:** 75 points (+ 10 bonus)

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Understanding MASt3R-SLAM](#understanding-mast3r-slam)
3. [Assignment Requirements Breakdown](#assignment-requirements-breakdown)
4. [Mars Environment Integration](#mars-environment-integration)
5. [Technical Architecture](#technical-architecture)
6. [Implementation Roadmap](#implementation-roadmap)
7. [Key Challenges and Solutions](#key-challenges-and-solutions)
8. [Resources and References](#resources-and-references)

---

## Project Overview

### What We're Building

This assignment requires implementing a **mobile manipulation system with SLAM capabilities** that can:

1. **Navigate autonomously** in unknown environments using sensor fusion
2. **Create maps** using 2D SLAM from laser scanner data
3. **Manipulate objects** during exploration using the UR3 arm
4. **Operate in challenging environments** including the Mars simulation (bonus)

### What We Have (From Part 1)

- ✅ Husky mobile base with 4-wheel differential drive
- ✅ UR3 6-DOF robotic arm mounted on top plate
- ✅ RH-P12-RN gripper for manipulation
- ✅ Working Gazebo simulation with TF transforms
- ✅ Basic RViz visualization setup

### What We Need to Add (Part 2)

- 🔄 **Sensor Fusion**: Extended Kalman Filter (EKF) combining odometry + IMU + GPS
- 🔄 **Path Planning**: Global (A*/RRT*) and local (DWA/TEB) planners
- 🔄 **SLAM**: 2D mapping using gmapping or hector_slam
- 🔄 **Integration**: Unified system for autonomous exploration and manipulation
- 🔄 **Mars Environment**: Optional bonus challenge with modified physics

---

## Understanding MASt3R-SLAM

### What is MASt3R-SLAM?

**MASt3R-SLAM** (CVPR 2025) is a real-time monocular dense SLAM system that represents the state-of-the-art in visual SLAM. While we won't replicate it exactly, understanding its principles will inform our implementation.

#### Key Innovation

- **Foundation**: Built on MASt3R, a two-view 3D reconstruction and matching prior
- **Performance**: Achieves 15 FPS real-time operation with global consistency
- **Flexibility**: Works with uncalibrated cameras and time-varying camera models
- **Dense Reconstruction**: Produces globally-consistent poses AND detailed 3D geometry

### Core Technical Concepts from MASt3R-SLAM

#### 1. Pointmap Processing
```
Input: MASt3R 3D pointmaps
↓
Normalize into rays (central camera model)
↓
Enable compatibility with diverse imaging systems
```

#### 2. Efficient Matching Strategy
- Instead of traditional feature matching in descriptor space
- Performs **massively parallel pixel-wise matching**
- Minimizes **angular error** between predicted and observed rays
- Much faster than sequential feature matching

#### 3. Backend Optimization
- Uses **Gauss-Newton optimization** instead of gradient descent
- Achieves efficient large-scale updates for global consistency
- Critical for maintaining real-time performance at scale

### What We Learn from MASt3R-SLAM for Our Assignment

While our assignment focuses on **2D SLAM** (not 3D dense reconstruction), we can apply these principles:

1. **Sensor Fusion Philosophy**: Combine multiple data sources efficiently
2. **Real-time Constraints**: Design for ≥10 Hz update rates (required by assignment)
3. **Robust Matching**: Use efficient algorithms for loop closure detection
4. **Backend Optimization**: Proper optimization techniques for map consistency

---

## Assignment Requirements Breakdown

### 1. Sensor Fusion with EKF (15 points)

#### What is Required

- **State Estimation**: Fuse odometry, IMU, and GPS data
- **Covariance Management**: Handle measurement and process noise matrices
- **Update Rate**: Maintain ≥10 Hz real-time performance
- **Deliverable**: EKF node publishing to `/robot_pose` topic

#### Implementation Components

```python
State Vector (x):
[px, py, pz, θ_roll, θ_pitch, θ_yaw]T  # 6D pose

Process Model:
x_k = f(x_{k-1}, u_k) + w_k
where:
  u_k = control input (wheel velocities)
  w_k ~ N(0, Q) = process noise

Measurement Model:
z_k = h(x_k) + v_k
where:
  z_k = [odometry, IMU, GPS] measurements
  v_k ~ N(0, R) = measurement noise

EKF Update Cycle:
1. Prediction: x̂_k⁻ = f(x̂_{k-1}, u_k)
               P_k⁻ = F_k P_{k-1} F_k^T + Q_k

2. Update: K_k = P_k⁻ H_k^T (H_k P_k⁻ H_k^T + R_k)^-1
           x̂_k = x̂_k⁻ + K_k(z_k - h(x̂_k⁻))
           P_k = (I - K_k H_k) P_k⁻
```

#### Key Parameters (from assignment hints)

```python
# Initial covariance (small uncertainty)
P = np.eye(6) * 0.01

# Process noise (model uncertainty)
Q = np.diag([0.1, 0.1, 0.01, 0.01, 0.01, 0.001])

# Measurement noise (sensor accuracy)
R = np.diag([0.5, 0.5, 0.1])  # Higher for less accurate sensors
```

### 2. Path Planning & Control (15 points)

#### Required Components

**Global Planner** (A* or RRT*)
- Plans optimal path from start to goal
- Considers static obstacles from map
- Outputs waypoint sequence

**Local Planner** (DWA or TEB)
- Dynamic Window Approach (DWA) - velocity space sampling
- Timed Elastic Band (TEB) - trajectory optimization
- Handles dynamic obstacles in real-time

**Obstacle Avoidance**
- Uses laser scanner data
- Updates costmap continuously
- Triggers replanning when needed

#### Navigation Stack Architecture

```
Goal Pose → Global Planner → Waypoints
              ↓
         Costmap (static + dynamic obstacles)
              ↓
Local Planner → Velocity Commands → Robot Controllers
              ↑
         Laser Scanner + Odometry
```

### 3. SLAM Implementation (10 points)

#### Requirements

- **Algorithm**: gmapping or hector_slam
- **Map Resolution**: Minimum 0.05m (5cm per pixel)
- **Loop Closure**: Basic detection to prevent drift
- **Deliverable**: Occupancy grid saved as `.pgm` file

#### Configuration (from assignment hints)

```yaml
# Critical topics
scan_topic: /scan
map_frame: map
odom_frame: odom
base_frame: base_link

# Map parameters
resolution: 0.05  # meters per pixel
width: 2048       # pixels
height: 2048      # pixels
origin: [-51.2, -51.2, 0.0]  # 102.4m x 102.4m world
```

#### Why These Algorithms?

**gmapping** (GMapping - Grid-based FastSLAM)
- Pros: Robust, well-tested, handles loop closures
- Cons: Requires odometry
- Best for: Controlled indoor environments

**hector_slam** (Hector SLAM)
- Pros: Works without odometry, fast scan matching
- Cons: Can drift in featureless environments
- Best for: Fast-moving robots, 3D terrain

### 4. Integration Demo (25 points)

#### Demo Requirements

The robot must:
1. Start from **unknown location**
2. Perform **autonomous exploration** and create map
3. **Manipulate objects** during exploration (pick/place with gripper)
4. **Return to start** using the created map

#### ROSbag Recording Requirements

- **Duration**: Minimum 5 minutes
- **Topics to include**:
  - `/scan` - laser data
  - `/joint_states` - robot joints
  - `/tf` - transforms
  - `/robot_pose` - EKF output
  - `/map` - SLAM output
  - `/cmd_vel` - velocity commands
  - Camera topics (optional)

### 5. Mars Environment Bonus (+10 points)

#### Additional Challenges

- **Reduced Gravity**: 3.71 m/s² instead of 9.81 m/s²
- **Reduced Traction**: Wheel slip on sandy terrain
- **Complex Obstacles**: Rocks and craters
- **Race Track Challenge**: Navigate predefined Mars course

#### Modified Physics Parameters

```yaml
# Mars-specific configuration
gravity_compensation: 3.71
wheel_slip_compensation: 0.15  # 15% slip
max_linear_velocity: 0.5  # Slower for safety
```

### 6. Technical Report (10 points)

#### RSS Format Requirements

- **Length**: 8-10 pages including references
- **Sections**:
  - Abstract (150-200 words)
  - Introduction (problem + motivation)
  - Related Work (cite MASt3R-SLAM and other SLAM papers)
  - Methodology (your implementation details)
  - Results (quantitative metrics + qualitative analysis)
  - Conclusion (lessons learned + future work)

#### Performance Metrics to Report

```
Localization Accuracy: < 0.5m RMSE
Mapping Quality: Clear obstacle boundaries
Path Following: < 0.3m cross-track error
Manipulation Success: 80% grasp success rate
Real-time Performance: Maintain 10 Hz control loop
```

---

## Mars Environment Integration

### Setup Process

#### 1. Install Mars Terrain Packages

```bash
cd ~/ros2_ws/src

# Clone Mars repositories
git clone https://github.com/nasa-jpl/mars_gazebo_terrains.git
git clone https://github.com/nasa/mars-rover-simulation.git

# Install dependencies
sudo apt-get install python3-pcl libgdal-dev

# Build
cd ~/ros2_ws
colcon build --packages-select mars_gazebo_terrains
source install/setup.bash
```

#### 2. Download Mars Assets

```bash
cd ~/ros2_ws/src/mars_gazebo_terrains

# Download terrain meshes (2GB+)
wget https://nasa-mars-data.s3.amazonaws.com/terrain/jezero_crater_mesh.dae
wget https://nasa-mars-data.s3.amazonaws.com/terrain/jezero_crater_heightmap.tif

# Download textures
./scripts/download_mars_textures.sh
```

#### 3. Key Mars Configuration

**Physics Configuration** (`mars_physics.yaml`):
```yaml
physics:
  gravity: 0 0 -3.71  # Mars gravity

contact:
  soft_cfm: 0.01  # Soft terrain
  soft_erp: 0.2

robot_parameters:
  wheel_slip: 0.15
  max_velocity: 0.5
```

**Lighting and Atmosphere** (`mars_sky.sdf`):
```xml
<scene>
  <ambient>0.4 0.3 0.2 1</ambient>  <!-- Reddish ambient -->
  <background>0.7 0.4 0.3 1</background>  <!-- Mars sky -->
</scene>

<light name="sun" type="directional">
  <diffuse>0.8 0.6 0.5 1</diffuse>  <!-- Reduced sunlight -->
</light>
```

### Mars-Specific Challenges

#### Challenge 1: Reduced Traction
**Problem**: Wheels slip on sandy terrain
**Solution**: Increase friction coefficients in URDF

```xml
<mu>100.0</mu>  <!-- Static friction -->
<mu2>100.0</mu2>  <!-- Dynamic friction -->
```

#### Challenge 2: Robot Sinking
**Problem**: Robot sinks into soft terrain
**Solution**: Adjust contact parameters

```xml
<kp>1000000.0</kp>  <!-- Contact stiffness -->
<kd>100.0</kd>      <!-- Contact damping -->
<min_depth>0.001</min_depth>
```

#### Challenge 3: Visual Odometry Drift
**Problem**: Uniform terrain causes feature tracking failure
**Solution**: Rely more heavily on IMU + wheel odometry in EKF

```python
# Adjust measurement noise - trust vision less on Mars
R_visual = np.eye(3) * 1.0  # Higher uncertainty
R_imu = np.eye(3) * 0.1     # Lower uncertainty
```

---

## Technical Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        Gazebo Simulation                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │  Husky   │  │   UR3    │  │ Gripper  │  │ Sensors  │   │
│  │  Base    │  │   Arm    │  │          │  │          │   │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘   │
└─────────────────────────────────────────────────────────────┘
         │              │              │              │
         ▼              ▼              ▼              ▼
┌─────────────────────────────────────────────────────────────┐
│                     ROS2 Bridge Layer                        │
│  /joint_states   /tf   /scan   /imu   /gps   /cmd_vel       │
└─────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────┐
│                    Perception Layer                          │
│  ┌──────────────────┐         ┌─────────────────┐          │
│  │   EKF Fusion     │ ──────→ │  /robot_pose    │          │
│  │  (odometry+IMU   │         │  (localization) │          │
│  │   +GPS)          │         └─────────────────┘          │
│  └──────────────────┘                                        │
│                                                               │
│  ┌──────────────────┐         ┌─────────────────┐          │
│  │   SLAM Node      │ ──────→ │     /map        │          │
│  │  (gmapping/      │         │  (occupancy     │          │
│  │   hector)        │         │   grid)         │          │
│  └──────────────────┘         └─────────────────┘          │
└─────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────┐
│                     Planning Layer                           │
│  ┌──────────────────┐         ┌─────────────────┐          │
│  │ Global Planner   │         │ Local Planner   │          │
│  │  (A*/RRT*)       │ ──────→ │  (DWA/TEB)      │          │
│  │                  │         │                 │          │
│  └──────────────────┘         └─────────────────┘          │
│           │                            │                     │
│           └────────────┬───────────────┘                     │
│                        ▼                                      │
│              ┌─────────────────┐                            │
│              │   Costmaps      │                            │
│              │  (static+dynamic)│                            │
│              └─────────────────┘                            │
└─────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────┐
│                     Control Layer                            │
│  ┌──────────────────┐         ┌─────────────────┐          │
│  │ Base Controller  │         │  Arm Controller │          │
│  │  (diff_drive)    │         │   (MoveIt2)     │          │
│  └──────────────────┘         └─────────────────┘          │
└─────────────────────────────────────────────────────────────┘
         │                               │
         ▼                               ▼
    /cmd_vel                      /joint_commands
```

### Package Structure

```
ros2_ws/src/
├── husky_ur3_description/          # From Part 1
│   ├── urdf/
│   ├── launch/
│   └── config/
│
├── husky_ur3_slam/                 # NEW for Part 2
│   ├── launch/
│   │   ├── ekf_fusion.launch.py
│   │   ├── slam.launch.py
│   │   ├── navigation.launch.py
│   │   └── full_system.launch.py
│   │
│   ├── config/
│   │   ├── ekf_params.yaml
│   │   ├── slam_params.yaml
│   │   ├── nav2_params.yaml
│   │   └── costmap_params.yaml
│   │
│   ├── src/
│   │   ├── ekf_fusion_node.cpp
│   │   ├── exploration_node.cpp
│   │   └── manipulation_controller.cpp
│   │
│   └── scripts/
│       ├── bag_recorder.py
│       └── performance_analyzer.py
│
└── mars_gazebo_terrains/           # For bonus
    ├── worlds/
    │   └── jezero_crater.world
    ├── models/
    └── config/
```

---

## Implementation Roadmap

### Phase 1: Sensor Fusion (Week 1)

#### Step 1.1: Add Sensors to URDF

```bash
# Edit: husky_ur3_description/urdf/husky_ur3_gripper.urdf.xacro
```

Add laser scanner, IMU, and GPS:

```xml
<!-- Laser Scanner (already exists in Husky) -->
<gazebo reference="base_laser">
  <sensor type="gpu_ray" name="laser_scanner">
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
      </range>
    </ray>
    <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>

<!-- IMU Sensor -->
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <update_rate>100</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <remapping>~/out:=imu/data</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>

<!-- GPS Sensor -->
<gazebo>
  <plugin name="gps_plugin" filename="libgazebo_ros_gps_sensor.so">
    <ros>
      <namespace>/gps</namespace>
      <remapping>~/out:=fix</remapping>
    </ros>
    <update_rate>4.0</update_rate>
  </plugin>
</gazebo>
```

#### Step 1.2: Implement EKF Node

Create `husky_ur3_slam/src/ekf_fusion_node.cpp`:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>

class EKFFusionNode : public rclcpp::Node {
public:
    EKFFusionNode() : Node("ekf_fusion_node") {
        // Initialize state vector [x, y, z, roll, pitch, yaw]
        state_ = Eigen::VectorXd::Zero(6);

        // Initialize covariance
        P_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;

        // Process noise
        Q_ = Eigen::MatrixXd::Identity(6, 6);
        Q_.diagonal() << 0.1, 0.1, 0.01, 0.01, 0.01, 0.001;

        // Measurement noise
        R_ = Eigen::MatrixXd::Identity(6, 6);
        R_.diagonal() << 0.5, 0.5, 0.1, 0.01, 0.01, 0.01;

        // Subscribers
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&EKFFusionNode::odomCallback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&EKFFusionNode::imuCallback, this, std::placeholders::_1));

        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10,
            std::bind(&EKFFusionNode::gpsCallback, this, std::placeholders::_1));

        // Publisher
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/robot_pose", 10);

        // Timer for prediction step
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&EKFFusionNode::predictionStep, this));
    }

private:
    void predictionStep() {
        // EKF prediction
        // x_k = F * x_{k-1} + B * u
        // P_k = F * P_{k-1} * F^T + Q

        // Publish current estimate
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = state_(0);
        pose_msg.pose.position.y = state_(1);
        pose_msg.pose.position.z = state_(2);
        // ... set orientation from state_(3), state_(4), state_(5)

        pose_pub_->publish(pose_msg);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // EKF update with odometry measurement
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // EKF update with IMU measurement
    }

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        // EKF update with GPS measurement
    }

    Eigen::VectorXd state_;
    Eigen::MatrixXd P_, Q_, R_;
    // ... subscribers, publishers, timer
};
```

#### Step 1.3: Test EKF

```bash
# Terminal 1: Launch simulation
ros2 launch husky_ur3_description gazebo_sim.launch.py

# Terminal 2: Launch EKF node
ros2 launch husky_ur3_slam ekf_fusion.launch.py

# Terminal 3: Monitor output
ros2 topic echo /robot_pose
ros2 topic hz /robot_pose  # Should be ~10 Hz
```

### Phase 2: SLAM Setup (Week 1-2)

#### Step 2.1: Choose SLAM Algorithm

**Option A: gmapping** (recommended for beginners)

Install:
```bash
sudo apt install ros-humble-slam-gmapping
```

Config (`config/gmapping_params.yaml`):
```yaml
slam_gmapping:
  ros__parameters:
    map_update_interval: 5.0
    maxUrange: 25.0
    sigma: 0.05
    kernelSize: 1
    lstep: 0.05
    astep: 0.05
    iterations: 5
    lsigma: 0.075
    ogain: 3.0
    lskip: 0
    minimumScore: 50
    srr: 0.1
    srt: 0.2
    str: 0.2
    stt: 0.2
    linearUpdate: 0.5
    angularUpdate: 0.3
    temporalUpdate: -1.0
    resampleThreshold: 0.5
    particles: 30
    xmin: -50.0
    ymin: -50.0
    xmax: 50.0
    ymax: 50.0
    delta: 0.05
    llsamplerange: 0.01
    llsamplestep: 0.01
    lasamplerange: 0.005
    lasamplestep: 0.005
```

**Option B: slam_toolbox** (modern alternative)

Install:
```bash
sudo apt install ros-humble-slam-toolbox
```

Config (`config/slam_toolbox_params.yaml`):
```yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    mode: mapping  # localization, mapping, or lifelong

    # Map settings
    map_frame: map
    odom_frame: odom
    base_frame: base_link
    scan_topic: /scan

    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5

    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5

    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    resolution: 0.05
```

#### Step 2.2: Create SLAM Launch File

`launch/slam.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('husky_ur3_slam')

    slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params, {'use_sim_time': True}],
            output='screen'
        ),
    ])
```

#### Step 2.3: Test SLAM

```bash
# Terminal 1: Simulation
ros2 launch husky_ur3_description gazebo_sim.launch.py

# Terminal 2: SLAM
ros2 launch husky_ur3_slam slam.launch.py

# Terminal 3: Teleop to drive around
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4: Visualize in RViz
rviz2 -d $(ros2 pkg prefix husky_ur3_slam)/share/husky_ur3_slam/rviz/slam.rviz
```

In RViz:
- Add Map display, topic: `/map`
- Add LaserScan, topic: `/scan`
- Fixed frame: `map`

### Phase 3: Navigation Stack (Week 2)

#### Step 3.1: Install Nav2

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

#### Step 3.2: Configure Costmaps

`config/costmap_common.yaml`:
```yaml
costmap_2d:
  ros__parameters:
    footprint: "[[0.45, 0.35], [0.45, -0.35], [-0.45, -0.35], [-0.45, 0.35]]"
    footprint_padding: 0.01

    obstacle_layer:
      enabled: True
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_max_range: 3.0
        raytrace_min_range: 0.0
        obstacle_max_range: 2.5
        obstacle_min_range: 0.0

    inflation_layer:
      enabled: True
      cost_scaling_factor: 3.0
      inflation_radius: 0.8

    static_layer:
      enabled: True
      map_subscribe_transient_local: True
```

`config/nav2_params.yaml`:
```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered
    bt_loop_duration: 10
    default_server_timeout: 20

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: -0.5
      max_vel_x: 0.5
      min_vel_y: 0.0
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 0.5
      acc_lim_y: 0.0
      acc_lim_theta: 1.0
      decel_lim_x: -0.5
      decel_lim_y: 0.0
      decel_lim_theta: -1.0
```

#### Step 3.3: Create Navigation Launch

`launch/navigation.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_dir = get_package_share_directory('husky_ur3_slam')

    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params,
            }.items()
        ),
    ])
```

### Phase 4: Autonomous Exploration (Week 3)

#### Step 4.1: Create Exploration Node

`src/exploration_node.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class ExplorationNode : public rclcpp::Node {
public:
    ExplorationNode() : Node("exploration_node") {
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&ExplorationNode::mapCallback, this, std::placeholders::_1));

        nav_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        // Find frontiers (boundaries between known and unknown)
        auto frontiers = findFrontiers(map);

        if (!frontiers.empty()) {
            // Choose best frontier
            auto goal = selectBestFrontier(frontiers);

            // Send navigation goal
            sendNavigationGoal(goal);
        }
    }

    std::vector<geometry_msgs::msg::Point> findFrontiers(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map) {
        std::vector<geometry_msgs::msg::Point> frontiers;

        // Scan map for frontier cells
        for (size_t i = 0; i < map->data.size(); i++) {
            if (map->data[i] == 0) {  // Free cell
                // Check if adjacent to unknown (-1)
                if (hasUnknownNeighbor(map, i)) {
                    geometry_msgs::msg::Point p;
                    p.x = (i % map->info.width) * map->info.resolution;
                    p.y = (i / map->info.width) * map->info.resolution;
                    frontiers.push_back(p);
                }
            }
        }

        return frontiers;
    }

    geometry_msgs::msg::PoseStamped selectBestFrontier(
        const std::vector<geometry_msgs::msg::Point>& frontiers) {
        // Select frontier closest to current position
        // or use more sophisticated criteria (information gain, etc.)
    }

    void sendNavigationGoal(const geometry_msgs::msg::PoseStamped& goal) {
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = goal;

        nav_action_client_->async_send_goal(goal_msg);
    }
};
```

### Phase 5: Manipulation Integration (Week 3)

#### Step 5.1: Add Object Detection

Use laser scanner to detect objects:

```cpp
class ObjectDetector : public rclcpp::Node {
public:
    ObjectDetector() : Node("object_detector") {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ObjectDetector::scanCallback, this, std::placeholders::_1));

        object_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
            "/detected_objects", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        // Simple clustering algorithm
        std::vector<std::vector<int>> clusters;

        // Group nearby points
        for (size_t i = 0; i < scan->ranges.size(); i++) {
            if (scan->ranges[i] < scan->range_max) {
                // Add to cluster or create new
            }
        }

        // Publish object poses
        geometry_msgs::msg::PoseArray objects;
        for (auto& cluster : clusters) {
            geometry_msgs::msg::Pose obj_pose;
            // Calculate centroid
            objects.poses.push_back(obj_pose);
        }
        object_pub_->publish(objects);
    }
};
```

#### Step 5.2: Create Manipulation Controller

```python
# scripts/manipulation_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from moveit_msgs.action import MoveGroup

class ManipulationController(Node):
    def __init__(self):
        super().__init__('manipulation_controller')

        self.object_sub = self.create_subscription(
            PoseArray, '/detected_objects',
            self.object_callback, 10)

        # MoveIt2 action client
        self.move_group_client = self.create_action_client(
            MoveGroup, 'move_action')

    def object_callback(self, msg):
        if len(msg.poses) > 0:
            # Pick closest object
            target = msg.poses[0]

            # Execute pick sequence
            self.pick_object(target)

    def pick_object(self, target_pose):
        # 1. Plan to pre-grasp pose
        # 2. Move to pre-grasp
        # 3. Open gripper
        # 4. Move to grasp pose
        # 5. Close gripper
        # 6. Lift object
        pass
```

### Phase 6: Full System Integration (Week 4)

#### Step 6.1: Create Unified Launch File

`launch/full_system.launch.py`:
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Start simulation
        IncludeLaunchDescription('gazebo_sim.launch.py'),

        # 2. Start EKF
        IncludeLaunchDescription('ekf_fusion.launch.py'),

        # 3. Start SLAM
        IncludeLaunchDescription('slam.launch.py'),

        # 4. Start navigation
        IncludeLaunchDescription('navigation.launch.py'),

        # 5. Start exploration
        Node(
            package='husky_ur3_slam',
            executable='exploration_node',
            output='screen'
        ),

        # 6. Start manipulation
        Node(
            package='husky_ur3_slam',
            executable='manipulation_controller.py',
            output='screen'
        ),

        # 7. Start ROSbag recording
        Node(
            package='rosbag2',
            executable='record',
            arguments=[
                '-o', 'demo_recording',
                '-a'  # Record all topics
            ]
        ),

        # 8. Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'full_system.rviz']
        ),
    ])
```

#### Step 6.2: Test Full Mission

```bash
# Launch everything
ros2 launch husky_ur3_slam full_system.launch.py

# The robot should:
# 1. Start exploring autonomously
# 2. Build a map
# 3. Detect and manipulate objects
# 4. Return to starting position
```

---

## Key Challenges and Solutions

### Challenge 1: EKF Divergence

**Problem**: EKF estimates diverge from ground truth

**Symptoms**:
- Robot pose jumps erratically
- Covariance grows unbounded
- Map alignment fails

**Solutions**:

1. **Check initialization**:
```python
# Too large initial uncertainty
P = np.eye(6) * 10.0  # BAD

# Appropriate initialization
P = np.eye(6) * 0.01  # GOOD
```

2. **Tune noise matrices**:
```python
# Process noise (how much robot motion is uncertain)
Q = np.diag([0.1, 0.1, 0.01, 0.01, 0.01, 0.001])

# Measurement noise (how much to trust sensors)
R_odom = np.diag([0.5, 0.5, 0.1])  # Less trust
R_imu = np.diag([0.01, 0.01, 0.01])  # More trust
R_gps = np.diag([2.0, 2.0, 1.0])  # Even less trust
```

3. **Verify sensor data**:
```bash
# Check for NaN or Inf values
ros2 topic echo /imu/data | grep -E "nan|inf"
ros2 topic echo /odometry/filtered | grep -E "nan|inf"
```

### Challenge 2: SLAM Not Building Map

**Problem**: Map remains empty or sparse

**Diagnosis**:
```bash
# Check laser data
ros2 topic echo /scan --once
ros2 topic hz /scan  # Should be ~10 Hz

# Check TF tree
ros2 run tf2_tools view_frames
evince frames.pdf  # Verify map → odom → base_link chain exists
```

**Solutions**:

1. **Verify frame IDs**:
```yaml
# In slam_params.yaml
map_frame: map        # NOT "map_frame" or "/map"
odom_frame: odom      # NOT "odom_frame"
base_frame: base_link # NOT "base"
```

2. **Check odometry**:
```bash
# SLAM needs wheel odometry
ros2 topic info /odometry/filtered
ros2 topic echo /odometry/filtered --once
```

3. **Drive the robot**:
- SLAM requires motion to build map
- Use teleop to drive around
- Ensure sufficient rotation for laser features

### Challenge 3: Navigation Failures

**Problem**: Robot won't navigate to goals or gets stuck

**Solutions**:

1. **Check costmap configuration**:
```yaml
# Ensure robot footprint matches actual size
footprint: "[[0.45, 0.35], [0.45, -0.35], [-0.45, -0.35], [-0.45, 0.35]]"

# Adjust inflation for narrow passages
inflation_radius: 0.8  # Reduce if robot is too cautious
cost_scaling_factor: 3.0
```

2. **Verify controller limits**:
```yaml
# Ensure velocities match robot capabilities
max_vel_x: 0.5   # Husky max is ~1.0 m/s
max_vel_theta: 1.0  # Adjust based on testing
```

3. **Monitor planning**:
```bash
# Visualize global and local plans in RViz
# Add Display → Path
# Topic: /plan (global)
# Topic: /local_plan (local)
```

### Challenge 4: TF Transform Issues

**Problem**: "Transform from [X] to [Y] does not exist"

**Solutions**:

1. **Verify robot_state_publisher is running**:
```bash
ros2 node list | grep robot_state_publisher
```

2. **Check for missing links**:
```bash
ros2 run tf2_tools view_frames
# Look for disconnected chains
```

3. **Ensure use_sim_time is consistent**:
```bash
# All nodes must use same time source
ros2 param get /slam_toolbox use_sim_time
ros2 param get /robot_state_publisher use_sim_time
# Both should return: Boolean value is: True
```

### Challenge 5: Performance Issues

**Problem**: System runs slowly or misses deadlines

**Solutions**:

1. **Reduce sensor rates**:
```xml
<!-- In URDF sensor plugins -->
<update_rate>5.0</update_rate>  <!-- Instead of 10.0 -->
```

2. **Optimize SLAM parameters**:
```yaml
# Reduce particle count
particles: 15  # Instead of 30

# Reduce map resolution
delta: 0.10  # Instead of 0.05 (10cm vs 5cm)
```

3. **Use multi-threaded executor**:
```cpp
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```

---

## Resources and References

### Required Reading

1. **MASt3R-SLAM Paper**: https://edexheim.github.io/mast3r-slam/
   - Focus on: Real-time optimization techniques, sensor fusion principles

2. **Probabilistic Robotics** (Thrun, Burgard, Fox)
   - Chapter 3: Gaussian Filters (EKF)
   - Chapter 10: SLAM
   - Available: http://www.probabilistic-robotics.org/

3. **Nav2 Documentation**: https://navigation.ros.org/
   - Tuning Guide: https://navigation.ros.org/tuning/index.html

### Tools and Packages

```bash
# Essential packages
sudo apt install \
  ros-humble-robot-localization \    # EKF implementation
  ros-humble-slam-toolbox \           # Modern SLAM
  ros-humble-navigation2 \            # Nav2 stack
  ros-humble-moveit \                 # Manipulation
  ros-humble-rviz2 \                  # Visualization
  ros-humble-robot-state-publisher \  # TF publishing
  ros-humble-joint-state-publisher    # Joint states
```

### Starter Code Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/kulbir-ahluwalia/cs498gc_assignment4_part2
```

### Office Hours and Support

- **Regular**: Wednesday 1:30-2:30 PM @ SC 4407
- **Special SLAM Sessions**: Dec 2, 4, 6 @ 3:00 PM
- **Campuswire**: Tag posts with `#assignment4-part2`
- **Email**: ksa5@illinois.edu

### Debugging Tools

```bash
# Monitor system performance
htop
ros2 topic list
ros2 topic hz /scan
ros2 topic bw /map

# TF debugging
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link

# ROS graph visualization
rqt_graph

# Parameter introspection
ros2 param list
ros2 param describe /slam_toolbox use_sim_time
```

---

## Submission Checklist

### Before Submission

- [ ] **EKF publishes** to `/robot_pose` at ≥10 Hz
- [ ] **SLAM creates map** saved as `.pgm` file
- [ ] **Navigation works** between arbitrary goals
- [ ] **Manipulation works** with ≥80% success
- [ ] **ROSbag recorded** for 5+ minutes
- [ ] **Video created** and uploaded to YouTube
- [ ] **Report written** in RSS format (8-10 pages)
- [ ] **Code documented** with clear comments
- [ ] **Git history** shows incremental progress

### Submission Files

```
assignment4_part2_submission/
├── src/
│   └── husky_ur3_slam/
│       ├── launch/
│       ├── config/
│       ├── src/
│       └── scripts/
├── rosbags/
│   ├── standard_world_demo/
│   └── mars_world_demo/  (if bonus)
├── maps/
│   ├── final_map.pgm
│   └── final_map.yaml
├── report/
│   └── lastname_firstname_report.pdf
└── README.md
```

### Grading Rubric

| Component | Points | Criteria |
|-----------|--------|----------|
| EKF Implementation | 15 | Correct fusion, 10Hz, low error |
| Path Planning | 15 | Global + local planners work |
| SLAM | 10 | Clean map, loop closure |
| Integration Demo | 25 | Autonomous, manipulation, return |
| Mars Bonus | +10 | Complete Mars challenge |
| Technical Report | 10 | RSS format, complete analysis |
| Code Quality | 10 | Clean, documented, modular |
| Video Demo | 10 | Clear demonstration of features |

---

## Final Notes

### Time Management

- **Week 1**: Sensors + EKF (15-20 hours)
- **Week 2**: SLAM + Navigation (20-25 hours)
- **Week 3**: Integration + Testing (15-20 hours)
- **Week 4**: Mars bonus + Report (10-15 hours)
- **Total**: ~60-80 hours

### Common Mistakes to Avoid

1. **Starting too late** - This is complex, start early!
2. **Skipping incremental testing** - Test each component separately
3. **Ignoring frame_id** - Most TF errors come from wrong frame names
4. **Poor parameter tuning** - Spend time tuning, don't use defaults blindly
5. **Not recording data** - Start recording early, not just at the end

### Success Tips

1. **Build incrementally** - Get EKF working before SLAM
2. **Test in isolation** - Each component separately before integration
3. **Use RViz extensively** - Visualize everything (TF, maps, plans, scans)
4. **Ask for help early** - Don't wait until the last minute
5. **Document as you go** - Write report sections as you complete them

### Inspiration from MASt3R-SLAM

Remember, while you're implementing 2D SLAM, the principles from MASt3R-SLAM apply:

- **Real-time matters**: Optimize for performance
- **Sensor fusion is key**: Properly combine multiple data sources
- **Backend optimization**: Use proper mathematical techniques (EKF, graph optimization)
- **Robust matching**: Handle challenging scenarios (featureless areas, dynamic objects)

---

**Good luck with your implementation! Start early, test often, and don't hesitate to ask for help.**

*Last updated: December 1, 2025*
