#!/usr/bin/env bash

echo "Starting rviz"
source devel/setup.sh
roslaunch ur3_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur3_moveit_config)/launch/moveit.rviz
#Now kill simulation on exit
script_full_path=$(dirname "$0")
source ${script_full_path}/kill_simulation.sh
