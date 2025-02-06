#!/bin/bash
set -e

# Function to log messages
log_msg() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

# Source ROS setup files
if [ -f /opt/ros/noetic/setup.sh ]; then
    log_msg "Sourcing ROS setup files"
    . /opt/ros/noetic/setup.sh
else
    log_msg "ERROR: ROS setup.sh not found at /opt/ros/noetic/setup.sh"
    exit 1
fi

# Add ROS environment variables
export ROS_ROOT=/opt/ros/noetic
export PATH=$PATH:$ROS_ROOT/bin
export PYTHONPATH=$PYTHONPATH:$ROS_ROOT/lib/python3/dist-packages

# Set the shell for ROS
export SHELL=/bin/bash
export CATKIN_SHELL=/bin/bash

# Set ROS specific environment variables
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

# Set Gazebo environment variables
export GAZEBO_MODEL_PATH=${HOME}/workspace/src/agv_sim/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=${HOME}/workspace/src/agv_sim/worlds:${GAZEBO_RESOURCE_PATH}

# If a workspace exists and is built, source it
if [ -f "${HOME}/workspace/devel/setup.bash" ]; then
    log_msg "Sourcing workspace setup"
    . "${HOME}/workspace/devel/setup.bash"
else
    log_msg "WARNING: Workspace setup.bash not found at ${HOME}/workspace/devel/setup.bash"
fi

# Initialize rosdep if needed
if ! [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    log_msg "Initializing rosdep"
    sudo rosdep init
    rosdep update
fi

log_msg "Environment setup completed successfully"
