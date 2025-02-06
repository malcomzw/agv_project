# CMake generated Testfile for 
# Source directory: /home/malcom/ros-jenkins-pipeline/ros_ws/src/agv_sim
# Build directory: /home/malcom/ros-jenkins-pipeline/ros_ws/build/agv_sim
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_agv_sim_rostest_test_simulation_test.test "/home/malcom/ros-jenkins-pipeline/ros_ws/build/agv_sim/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/malcom/ros-jenkins-pipeline/ros_ws/build/agv_sim/test_results/agv_sim/rostest-test_simulation_test.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/malcom/ros-jenkins-pipeline/ros_ws/src/agv_sim --package=agv_sim --results-filename test_simulation_test.xml --results-base-dir \"/home/malcom/ros-jenkins-pipeline/ros_ws/build/agv_sim/test_results\" /home/malcom/ros-jenkins-pipeline/ros_ws/src/agv_sim/test/simulation_test.test ")
set_tests_properties(_ctest_agv_sim_rostest_test_simulation_test.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/malcom/ros-jenkins-pipeline/ros_ws/src/agv_sim/CMakeLists.txt;77;add_rostest;/home/malcom/ros-jenkins-pipeline/ros_ws/src/agv_sim/CMakeLists.txt;0;")
subdirs("gtest")
