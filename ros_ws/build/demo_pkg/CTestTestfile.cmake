# CMake generated Testfile for 
# Source directory: /home/malcom/ros-jenkins-pipeline/ros_ws/src/demo_pkg
# Build directory: /home/malcom/ros-jenkins-pipeline/ros_ws/build/demo_pkg
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_demo_pkg_rostest_test_demo_test.test "/home/malcom/ros-jenkins-pipeline/ros_ws/build/demo_pkg/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/malcom/ros-jenkins-pipeline/ros_ws/build/demo_pkg/test_results/demo_pkg/rostest-test_demo_test.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/malcom/ros-jenkins-pipeline/ros_ws/src/demo_pkg --package=demo_pkg --results-filename test_demo_test.xml --results-base-dir \"/home/malcom/ros-jenkins-pipeline/ros_ws/build/demo_pkg/test_results\" /home/malcom/ros-jenkins-pipeline/ros_ws/src/demo_pkg/test/demo_test.test ")
set_tests_properties(_ctest_demo_pkg_rostest_test_demo_test.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/malcom/ros-jenkins-pipeline/ros_ws/src/demo_pkg/CMakeLists.txt;21;add_rostest;/home/malcom/ros-jenkins-pipeline/ros_ws/src/demo_pkg/CMakeLists.txt;0;")
subdirs("gtest")
