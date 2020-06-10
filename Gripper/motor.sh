#!/bin/bash
# Always run setup.bash to initialize environment parameters that ROS requires.
# Otherwise roslaunch will not find the files
. ~/catkin_ws/devel/setup.bash
# Assuming that the start.sh file has been called and ROS started correctly we can access the motor state
rostopic echo /reflex_one/hand_state/motor
