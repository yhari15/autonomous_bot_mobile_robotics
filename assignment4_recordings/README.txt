CS498GC Mobile Robotics - Assignment 4 Extra Credit

Student Information:

NetID: hya5cob
Assignment: Assignment 4 - Mobile Manipulation with Gripper

System Configuration:
- ROS 2 Version: Humble
- Gazebo Version: Ignition Gazebo (Fortress)
- Operating System: Ubuntu 22.04
- Robot Platform: Clearpath Husky with UR3 arm and Robotis RH-P12-RN gripper


Project Description:
This assignment demonstrates a mobile manipulation system integrating:

1. Mobile Base Control:
   - Clearpath Husky differential drive robot
   - 4-wheel drive configuration for improved traction
   - Teleoperation control for forward, backward, and turning motions
2. Gripper Actuation:
   - Robotis RH-P12-RN parallel gripper
   - Automatic open/close cycling (15-second intervals)
   - Ignition Gazebo JointPositionController plugin
   - Position control range: 0.0 rad (open) to 0.65 rad (closed with gap)
3. Simulation Environment:
   - Warehouse.sdf
   - Physics-based simulation with collision detection
   - Real-time sensor data publishing


Demonstration Overview:
The recorded demonstration shows:

1. Mobile Base Movement:
   - Forward and backward translation
   - Arc-based turning maneuvers (left and right)
   - Smooth velocity control via keyboard teleoperation
2. Gripper Operation:
   - Autonomous cycling between open and closed states
   - Maintains safe gap when closed to prevent collision
   - Demonstrates parallel jaw mechanism
   - Adjusted the close state gap between two arms to be high because after certain time 
   -i observed they got locked to each other and not open if the gap is less then mechanically simulated in gazebo to lock each other
3. Combined Operations:
   - Mobile base movement while gripper operates
   - Coordinated motion showing system integration


Technical Implementation:
1. Control Architecture:
   - DiffDrive plugin for mobile base control
   - JointPositionController for gripper actuation
   - ROS 2 topics for command and state publishing
   - TF tree for coordinate frame transforms
2. Key Topics Recorded:
   - /cmd_vel: Velocity commands to mobile base
   - /odom: Odometry data (position and velocity)
   - /joint_states: All joint positions (wheels, arm, gripper)
   - /rh_p12_rn_position/command: Gripper position commands
   - /tf: Transform tree for visualization
3. Configuration Parameters:
   - Wheel torque: 5000 Nm (maximized for rotation)
   - Gripper PID gains: P=5000, I=500, D=100
   - Gripper cycle time: 15 seconds
   - Odometry publish rate: 50 Hz


Files Included:
1. rosbag/ (or assignment4_hya5cob/)
   - ROS 2 bag file in rosbag2 format (.db3 file)
   - Contains all relevant topics for replay and analysis
   - Recorded using: ros2 bag record -a -o assignment4_hya5cob  
   To replay:
   $ ros2 bag play assignment4_hya5cob
2. screen_recording.mp4 (or assignment4_hya5cob_video.webm)
   - Video demonstration of robot operation
   - Shows Gazebo simulation with robot movement
   - Duration: ~30-60 seconds
   - Format: WebM or MP4 (converted from WebM)
   


Known Limitations:
1. Rotation Performance:
   - Pure in-place rotation is limited by Gazebo physics simulation
   - Arc turns (forward + rotation) work more reliably
   - This is a known limitation of differential drive simulation with heavy loads
2. Gripper Gap:
   - Gripper arms maintain small gap when closed (by design)
   - Prevents collision and allows for smooth actuation
   - Configured at 0.65 rad maximum closure
3. Visual Rendering:
   - Some mesh files may show errors in RViz2
   - Robot structure and movement are fully functional
   - Collision geometries work correctly in Gazebo


How to Run:
1. Launch simulation:
   $ cd ~/ros2_ws
   $ ./LAUNCH_ASSIGNMENT4.sh
2. Control robot (in teleop terminal):
   - 'i' = forward
   - ',' = backward
   - 'j' = arc turn left
   - 'l' = arc turn right
   - 'k' = stop
3. Gripper operates automatically every 15 seconds
4. Optional: View in RViz2:
   $ ./LAUNCH_RVIZ2.sh
   (Set Fixed Frame to "odom" in RViz2)


Recording Details:
- Recording method: ROS 2 bag record with GNOME screen recorder
- All topics captured using -a flag
- Video captured at native resolution
- Synchronized recording of bag data and video


Verification:

To verify the rosbag file:
$ ros2 bag info assignment4_hya5cob

Expected output should show:
- Duration: 30-60 seconds
- Topics: /cmd_vel, /odom, /joint_states, /tf, etc.
- Messages: Several thousand messages across all topics


Additional Notes:

This implementation successfully integrates mobile base control with gripper manipulation in a simulated environment. The system demonstrates coordinated control of multiple degrees of freedom and provides a
foundation for more complex mobile manipulation tasks.

The gripper configuration has been carefully tuned to provide reliable actuation while maintaining safe operation (gap when closed). The mobile base uses arc turns instead of pure rotation for better performance
in the Gazebo physics simulation.




