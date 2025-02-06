FROM osrf/ros:noetic-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    ros-noetic-rostest \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /workspace

# Create a ROS entrypoint script
COPY docker/scripts/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

# Source ROS setup in bashrc
RUN echo 'source /opt/ros/noetic/setup.bash' >> /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
