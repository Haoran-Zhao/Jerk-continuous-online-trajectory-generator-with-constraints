#!/usr/bin/env bash

echo "Starting Simulation"
source devel/setup.sh
roslaunch endo_gazebo endo_gazebo_controller.launch > /tmp/gazebo_launch.log 2>&1 &
sleep 2
roslaunch endo_moveit_config custom.launch >/tmp/move_it.log 2>&1 &
sleep 2
rosrun robot_controller robot_controller | tee /tmp/robot_controller.log
#Now kill simulation on exit
script_full_path=$(dirname "$0")
source ${script_full_path}/kill_simulation.sh
