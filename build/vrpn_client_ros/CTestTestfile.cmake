# CMake generated Testfile for 
# Source directory: /home/jan/apurva_joshi/crazygames_ws/src/vrpn_client_ros
# Build directory: /home/jan/apurva_joshi/crazygames_ws/build/vrpn_client_ros
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(_ctest_vrpn_client_ros_roslint_package "/home/jan/apurva_joshi/crazygames_ws/build/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/indigo/share/catkin/cmake/test/run_tests.py" "/home/jan/apurva_joshi/crazygames_ws/build/test_results/vrpn_client_ros/roslint-vrpn_client_ros.xml" "--working-dir" "/home/jan/apurva_joshi/crazygames_ws/build/vrpn_client_ros" "--return-code" "/opt/ros/indigo/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/jan/apurva_joshi/crazygames_ws/build/test_results/vrpn_client_ros/roslint-vrpn_client_ros.xml make roslint_vrpn_client_ros")
ADD_TEST(_ctest_vrpn_client_ros_roslaunch-check_launch "/home/jan/apurva_joshi/crazygames_ws/build/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/indigo/share/catkin/cmake/test/run_tests.py" "/home/jan/apurva_joshi/crazygames_ws/build/test_results/vrpn_client_ros/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/jan/apurva_joshi/crazygames_ws/build/test_results/vrpn_client_ros" "/opt/ros/indigo/share/roslaunch/cmake/../scripts/roslaunch-check -o '/home/jan/apurva_joshi/crazygames_ws/build/test_results/vrpn_client_ros/roslaunch-check_launch.xml' '/home/jan/apurva_joshi/crazygames_ws/src/vrpn_client_ros/launch' ")
