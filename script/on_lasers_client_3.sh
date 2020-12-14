#!/bin/bash

# ydlidar ver.1

# Red is 1
# Green is 2
# Reset is sgr0

export ROS_MASTER_URI=http://192.168.0.7:11311
export ROS_HOSTNAME=192.168.0.4
source ~/.bashrc
roslaunch ydlidar_ros G4.launch
