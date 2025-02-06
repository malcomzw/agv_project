# ROS Jenkins Pipeline with AGV Simulation

This repository contains a ROS project with an Automated Guided Vehicle (AGV) simulation and Jenkins pipeline integration for continuous integration and testing. It includes a demo package for testing the CI/CD pipeline setup.

## Project Structure

```
.
├── Jenkinsfile              # Jenkins pipeline definition
├── docker/                  # Docker configuration
│   ├── Dockerfile          # Build environment definition
│   └── scripts/            # Build and setup scripts
├── ros_ws/                 # ROS workspace
│   └── src/               
│       ├── agv_sim/        # AGV simulation package
│       └── demo_pkg/       # Demo package for CI/CD testing
           └── test/        # Test files
               ├── demo_test.py
               └── demo_test.test
```

## Features

1. **CI/CD Pipeline**
   - Automated build process
   - Unit and integration tests
   - Docker-based build environment
   - Jenkins pipeline integration

2. **Docker Integration**
   - Containerized build environment
   - ROS Noetic base image
   - All dependencies included
   - Reproducible builds

3. **Test Framework**
   - ROS test integration
   - Python-based test suite
   - Automated test execution
   - Test results reporting

## Quick Start

1. **Clone the Repository**
   ```bash
   git clone https://github.com/malcomzw/agv_project.git
   cd agv_project
   ```

2. **Build with Docker**
   ```bash
   docker build -f docker/Dockerfile -t ros-jenkins .
   ```

3. **Build the ROS Workspace**
   ```bash
   cd ros_ws
   catkin build
   source devel/setup.bash
   ```

4. **Run Tests**
   ```bash
   catkin test demo_pkg --no-deps
   ```

## Requirements

- ROS Noetic
- Docker
- Jenkins
- Python 3
- catkin-tools

## Jenkins Pipeline Configuration

1. Create a new Pipeline job in Jenkins
2. Configure Git SCM with repository URL
3. Set the pipeline script path to `Jenkinsfile`
4. Configure build triggers as needed

## Development

1. **Build Workspace**
   ```bash
   source /opt/ros/noetic/setup.bash
   cd ros_ws
   catkin build
   source devel/setup.bash
   ```

2. **Run Tests Locally**
   ```bash
   catkin test demo_pkg --no-deps
   ```
roslaunch agv_sim simulation.launch
