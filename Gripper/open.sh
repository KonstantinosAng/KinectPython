#!/bin/bash
# Always run setup.bash to initialize environment parameters that ROS requires.
# Otherwise roslaunch will not find the files
. ~/catkin_ws/devel/setup.bash
# Assuming that the start.sh file has been called and ROS started correctly we can publish
# to the hand node
rostopic pub -1 /reflex_one/command_position reflex_one_msgs/PoseCommand "{f1: 0.0, f2: 0.0, f3: 0.0, preshape1: 0.0, preshape2: 0.0}"

