#!/bin/bash
export ROS_DISTRO=humble
colcon build --symlink-install --executor parallel --parallel $(nproc) --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
