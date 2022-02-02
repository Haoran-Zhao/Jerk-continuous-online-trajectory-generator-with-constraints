#!/usr/bin/env bash

echo "Starting Simulation"
source devel/setup.sh
rosservice call /controller_manager/switch_controller "start_controllers:
- 'joint_group_position_controller'
stop_controllers:
- 'arm_controller'
strictness: 2"
sleep 2
roslaunch moveit_servo start_servo_server.launch
#Now kill simulation on exit
script_full_path=$(dirname "$0")
source ${script_full_path}/kill_simulation.sh
