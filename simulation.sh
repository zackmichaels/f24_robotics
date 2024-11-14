#!/bin/bash
source /opt/ros/humble/setup.bash
export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
cd f24_robotics
colcon build
source install/setup.bash
ros2 launch webots_ros2_homework1_python f23_robotics_1_launch.py
