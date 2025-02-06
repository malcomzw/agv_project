FROM osrf/ros:noetic-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    ros-noetic-rostest \
    python3-pip \
    python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /workspace

# Setup ROS environment
SHELL ["/bin/bash", "-c"]
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc && \
    echo 'source /opt/ros/noetic/setup.bash' >> /etc/bash.bashrc

# Default command
CMD ["/bin/bash"]
