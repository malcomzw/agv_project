FROM osrf/ros:noetic-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    ros-noetic-rostest \
    && rm -rf /var/lib/apt/lists/*

# Setup shell and ROS environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc && \
    echo "export TERM=xterm" >> /etc/bash.bashrc

# Set workspace directory
WORKDIR /workspace

# Default command
CMD ["/bin/bash"]
