FROM osrf/ros:noetic-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    ros-noetic-rostest \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

WORKDIR /workspace
