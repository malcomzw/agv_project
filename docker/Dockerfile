FROM osrf/ros:noetic-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    ros-noetic-rostest \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /workspace

# Source ROS setup in bashrc
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc

# By default, run bash
CMD ["/bin/bash"]
