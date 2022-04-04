#!/usr/bin/env bash

echo "Starting Simulation"
source devel/setup.sh
roslaunch ur3_gazebo ur3_gazebo_controller.launch > /tmp/gazebo_launch.log 2>&1 &
sleep 2
roslaunch ur3_moveit_config custom.launch >/tmp/move_it.log 2>&1 &
sleep 2
gnome-terminal -- bash -c "sh ./src/Jerk-continuous-online-trajectory-generator-with-constraints/ROS/script/start_rviz.sh;exec bash"
sleep 5
rosrun robot_controller robot_controller | tee /tmp/robot_controller.log
#Now kill simulation on exit
script_full_path=$(dirname "$0")
source ${script_full_path}/kill_simulation.sh
