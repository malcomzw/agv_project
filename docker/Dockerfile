FROM osrf/ros:noetic-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    ros-noetic-rostest \
    ros-noetic-navigation \
    ros-noetic-move-base \
    python3-pip \
    python3-setuptools \
    ros-noetic-roslint \
    ros-noetic-rosdoc-lite \
    python3-catkin-lint \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-plugins \
    python3-pytest \
    python3-coverage \
    python3-osrf-pycommon \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Create workspace directory
WORKDIR /workspace

# Setup ROS environment
SHELL ["/bin/bash", "-c"]
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc && \
    echo 'source /opt/ros/noetic/setup.bash' >> /etc/bash.bashrc

# Install additional Python packages
RUN pip3 install \
    rospkg \
    catkin_pkg \
    empy \
    defusedxml \
    nose

# Default command
CMD ["/bin/bash"]
