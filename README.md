# didactic-waffle

**TurtleBot3 Monitoring and Verification System**

A ROS-based framework for autonomous robot monitoring, human detection, trajectory prediction, and safety verification using TurtleBot3 in Gazebo simulation.

## Overview

This project implements a comprehensive robot safety system that:
- Detects humans using YOLOv5 with depth sensing
- Predicts human trajectories using Kalman filters
- Computes reachability sets for both robots and humans
- Verifies collision-free navigation in real-time
- Supports multi-robot coordination

## Project Structure

```
didactic-waffle/
├── apala_ws/              # Workspace with RRT global planner
│   └── src/
│       ├── rrt-global-planner/
│       └── tb_apala/
├── teb_ws/                # Main workspace
│   └── src/
│       ├── monitoring/    # Human detection & prediction
│       ├── verification/  # Safety verification & reachability
│       ├── Multi-Robot-Gazebo-NaviStack/
│       └── multibot_layers/
└── turtlebot3_navigation.zip
```

## ROS Packages

### Monitoring Package
Human detection and trajectory prediction pipeline:
- **yolo_depth.py** - YOLOv5 object detection with depth camera integration
- **organize.cpp** - Point cloud processing and clustering
- **prediction.py** - Kalman filter-based trajectory prediction
- **monitor.py** - Real-time monitoring and alert system
- **kf_predictors.py** - Kalman filter implementations

### Verification Package
Safety verification and reachability analysis:
- **robot_reachability.py** - Robot reachability set computation
- **human_reachability.py** - Human reachability set computation
- **collision_tb3_*.py** - Collision detection for multiple robots
- **clustering_tb3_*.py** - Point cloud clustering per robot
- **case1-4.py** - Verification test cases

## Setup

```bash
mkdir -p ~/teb_ws/src
cd ~/teb_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
cd src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
cd .. && catkin_make
```

Add world and launch files to `turtlebot3_simulations/turtlebot3_gazebo/`.

## Usage

### Single Robot Monitoring

```bash
# Launch Gazebo environment
roslaunch turtlebot3_gazebo scene1.launch

# Launch navigation
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=<path_to_map.yaml>

# Launch detection and monitoring pipeline
rosrun monitoring yolo_depth.py
rosrun monitoring organize
rosrun monitoring prediction.py
rosrun monitoring monitor.py
```

### Multi-Robot System

```bash
# Launch multi-robot Gazebo
roslaunch turtlebot3_gazebo multi_gz.launch

# Launch multi-robot navigation
roslaunch turtlebot3_navigation multi_nav.launch

# View TF tree
rosrun rqt_tf_tree rqt_tf_tree
```

### Verification

```bash
rosrun verification robot_reachability.py

# Or use launch files
roslaunch verification scenario.launch
roslaunch verification yolo.launch
roslaunch verification clustering.launch
roslaunch verification cloud_processing.launch
```

## Technologies Used
- ROS Noetic
- Gazebo Simulation
- TurtleBot3 (Waffle Pi)
- YOLOv5 (Object Detection)
- OpenCV
- Point Cloud Library (PCL)
- Kalman Filters
- Python / C++

## Dependencies
- turtlebot3_simulations
- turtlebot3_msgs
- turtlebot3
- turtlebot3_navigation








