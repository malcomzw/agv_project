cmake_minimum_required(VERSION 3.0.2)
project(demo_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  rostest
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add test target
if(CATKIN_ENABLE_TESTING)
  add_rostest(test/demo_test.test)
endif()
