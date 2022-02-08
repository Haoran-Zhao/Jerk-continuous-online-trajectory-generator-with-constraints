#!/usr/bin/env bash

echo "Killing Simulation"
killall roslaunch
killall gzserver
killall gzclient
killall robot_controller
killall rosmaster
sudo kill -9 $(lsof -i:1234 | grep -v "NODE" | awk -F ' ' '{print $2}') 2> /dev/null
for p in $(ps -a | grep -e 'gazebo_.*' -e 'roslaunch' -e 'robot.*' -e 'cpp_interface.*'| awk -F ' ' '{print $1}'); do sudo kill -9 $p; done
killall -9 gazebo & killall -9 gzserver & killall -9 gzclient
