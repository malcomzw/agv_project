cmake_minimum_required(VERSION 3.0.2)
project(test_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/example_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED)
  
  # Unit tests
  catkin_add_gtest(test_example test/test_example.cpp)
  target_link_libraries(test_example
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
  )
  
  # Integration tests
  add_rostest_gtest(integration_test
    test/integration_test.test
    test/integration_test.cpp
  )
  target_link_libraries(integration_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
  )
  
  # Enable coverage if requested
  if(COVERAGE_ENABLED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  endif()
endif()
