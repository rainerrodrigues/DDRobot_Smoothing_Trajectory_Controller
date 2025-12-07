# DDRobot_Smoothing_Trajectory_Controller

## Overview
This project implements a complete navigation pipeline for a differential-drive robot using ROS 2 (Kilted) and Gazebo.
The system takes a set of discrete 2D waypoints, generates a smooth trajectory, and tracks it using a Pure Pursuit controller on a simulated TurtleBot3.

The solution is modular, tunable at runtime, and designed to be easily deployable on a real robot.

## Features
- Path Smoothing using spline interpolation
- Time-parameterized trajectory generation
- Pure Pursuit trajectory tracking controller
- Gazebo simulation (TurtleBot3)
- RViz visualization
- Runtime tuning via ROS 2 parameters
- Tracking error logging and plotting
- Obstacle avoidance using LaserScan
![PathTrackingDemo](https://github.com/rainerrodrigues/DDRobot_Smoothing_Trajectory_Controller/blob/main/PathTrackingDemo.png)
![Error](https://github.com/rainerrodrigues/DDRobot_Smoothing_Trajectory_Controller/blob/main/Error%20vs%20Time%20Fig.png) 
![RViz](https://github.com/rainerrodrigues/DDRobot_Smoothing_Trajectory_Controller/blob/main/Rviz.png)
![Gazebo](https://github.com/rainerrodrigues/DDRobot_Smoothing_Trajectory_Controller/blob/main/Gazebo.png)
## Setup Instructions
### System Requirements
- Ubuntu 24.04
- ROS 2 Kilted
- Gazebo
- TurtleBot3 packages

### Install Dependencies
```sudo apt update
sudo apt install ros-kilted-turtlebot3 \
                 ros-kilted-turtlebot3-simulations
```
Set Turtlebot model
```
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```
### Build Workspace
```cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
### Run Simulation
```
ros2 launch ddrobot_nav tb3_navigation.launch.py
```
### Visualization
RViz displays:
1. Robot model
2. TF tree
3. Odometry
4. LaserScan
5. Smoothed trajectory path

### Runtime Parameter Tuning
```ros2 param set /controller_node lookahead 1.5
ros2 param set /controller_node v_cmd 0.7
ros2 param set /controller_node slowdown_gain 0.5
```
[[Video Submission](https://github.com/rainerrodrigues/DDRobot_Smoothing_Trajectory_Controller/blob/main/VideoRecordingDDR.mp4)]
# Other Qs
## Extending to a Real Robot
To deploy on a real TurtleBot3: we can replace gazebo with the relevant hardware drivers, subscribe to real /odom and /scan topics and tune parameters with consideration for friction and commmunication latency and include watchdog in the system.

AI Tools Used for assisting in the design of ROS2 package architecture design, debug topic mismatches and refine pure pursuit tuning strategies.
## How to extend this code with obstacle avoidance
Obstacle avoidance can be added as a local reactive layer on top of the trajectory-tracking controller. The robot uses LiDAR (LaserScan) data to detect nearby obstacles while following the global trajectory. When an obstacle is detected within a safety distance, the controller temporarily modifies the velocity commands—slowing down, stopping, or steering away—without changing the global path. Once the obstacle is cleared, the robot resumes trajectory tracking.

This layered approach keeps the system modular, stable, and real-robot ready, and can be further extended with smoother methods like potential fields or local re-planning for more complex environments.
