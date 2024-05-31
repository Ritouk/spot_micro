#!/bin/bash

#Sourcing ros2
source /opt/ros/humble/setup.bash
source ~/spot_ws/install/setup.bash


echo "ROS2 env sourced"

# First Command
echo "launching spot bringup"
ros2 launch spot_bringup spot_app.launch.py &

sleep 1

#Open terminal and excute the command inside:
ros2 run motion_control controller_input 
