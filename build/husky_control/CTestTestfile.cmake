# CMake generated Testfile for 
# Source directory: /home/galliumos/kinetic_final/src/husky_control
# Build directory: /home/galliumos/kinetic_final/build/husky_control
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_husky_control_roslaunch-check_launch "/home/galliumos/kinetic_final/build/husky_control/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/galliumos/kinetic_final/build/husky_control/test_results/husky_control/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/galliumos/kinetic_final/build/husky_control/test_results/husky_control" "/opt/ros/kinetic/share/roslaunch/cmake/../scripts/roslaunch-check -o '/home/galliumos/kinetic_final/build/husky_control/test_results/husky_control/roslaunch-check_launch.xml' '/home/galliumos/kinetic_final/src/husky_control/launch' ")
subdirs(gtest)
