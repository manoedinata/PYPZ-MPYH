#!/bin/bash

sudo fuser -k 9090/tcp
sudo fuser -k 8080/tcp 
sudo fuser -k 8000/tcp

. install/setup.bash 
export ROS_LOCALHOST_ONLY=1
ros2 launch ros2_utils all.launch.py 
