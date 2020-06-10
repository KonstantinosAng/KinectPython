#!/bin/bash
# Always run setup.bash to initialize environment parameters that ROS requires
# Otherwise roslaunch will not find the files
. ~/catkin_ws/devel/setup.bash
# Assuming that the start.sh file has been called and ROS started correctly we can publish
# to the hand node
# python3 gripper.py For ros indigo
python3 ros_handler.py
