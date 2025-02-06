# ROS Jenkins Pipeline with AGV Simulation

This repository contains a ROS package with an Automated Guided Vehicle (AGV) simulation and Jenkins pipeline integration for continuous integration and testing. The AGV can be controlled using arrow keys in a Gazebo simulation environment.

## Project Structure

```
.
├── Jenkinsfile              # Jenkins pipeline definition
├── docker/                  # Docker configuration
│   └── Dockerfile          # Build environment definition
├── ros_ws/                 # ROS workspace
│   └── src/               
│       └── agv_sim/        # AGV simulation package
           ├── launch/      # Launch files
           │   └── simulation.launch
           ├── urdf/        # Robot model
           │   └── agv.urdf.xacro
           ├── worlds/      # Simulation environments
           │   └── warehouse.world
           ├── rviz/        # Visualization configs
           │   └── agv.rviz
           ├── src/         # Source code
           │   └── arrow_teleop.py
           └── test/        # Test files
               ├── simulation_test.py
               └── simulation_test.test
```

## Features

1. **AGV Simulation**
   - Differential drive robot model
   - Warehouse simulation environment
   - Arrow key control interface
   - RViz visualization

2. **Continuous Integration**
   - Automated build process
   - Unit and integration tests
   - Simulation-based testing
   - Code quality checks

3. **Docker Integration**
   - Containerized build environment
   - ROS Noetic base image
   - GPU support for Gazebo
   - All dependencies included

## Quick Start

1. **Install Dependencies**
   ```bash
   sudo apt-get update
   sudo apt-get install -y \
       ros-noetic-desktop-full \
       python3-catkin-tools \
       python3-pygame
   ```

2. **Build the Project**
   ```bash
   mkdir -p ~/ros_ws/src
   cd ~/ros_ws/src
   git clone https://github.com/yourusername/ros-jenkins-pipeline.git
   cd ~/ros_ws
   catkin build
   ```

3. **Run the Simulation**
   ```bash
   source /opt/ros/noetic/setup.bash
   source ~/ros_ws/devel/setup.bash
   roslaunch agv_sim simulation.launch
   ```

## Controls

- ↑ (Up Arrow): Move forward
- ↓ (Down Arrow): Move backward
- ← (Left Arrow): Turn left
- → (Right Arrow): Turn right
- Spacebar: Stop
- Q: Quit

## Requirements

- ROS Noetic
- Gazebo 11
- Python 3 with pygame
- Docker (for CI/CD)
- Jenkins (for CI/CD)
- ROS Noetic
- Gazebo 11

To run the simulation:

Open a new terminal and run:

source /opt/ros/noetic/setup.bash
source /home/malcom/ros-jenkins-pipeline/ros_ws/devel/setup.bash
roslaunch agv_sim simulation.launch
