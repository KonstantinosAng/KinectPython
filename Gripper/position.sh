#! /bin/bash
# Always run setup.bash to initialize environment parameters that ROS requires
# Otherwise roslaunch will not find the files
. ~/catkin_ws/devel/setup.bash
# Assuming that the start.sh file has been called and ROS started correctly we can publish
# to the hand node
rostopic pub -1 /reflex_one/command_position reflex_one_msgs/PoseCommand "{f1: 1.8, f2: 1.8, f3: 1.9, preshape1: .75, preshape2: .75}"
