#!/bin/sh
cd ~/raspirover/ros_catkin_ws
sh devel/setup.sh
export ROS_IP=raspirover
roslaunch raspirover raspirover.launch
