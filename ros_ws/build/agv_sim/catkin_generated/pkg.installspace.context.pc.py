# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;geometry_msgs;nav_msgs;sensor_msgs;tf2;tf2_ros;gazebo_ros;gazebo_plugins;move_base".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lagv_sim".split(';') if "-lagv_sim" != "" else []
PROJECT_NAME = "agv_sim"
PROJECT_SPACE_DIR = "/home/malcom/ros-jenkins-pipeline/ros_ws/install"
PROJECT_VERSION = "0.1.0"
