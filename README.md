# Jerk-continuous-online-trajectory-generator-with-constraints

This work presents an online trajectory generation algorithm using a sinusoidal jerk profile. The generator takes initial acceleration, velocity and position
as input, and plans a multi-segment trajectory to a goal position under jerk, acceleration, and velocity limits.
By analyzing the critical constraints and conditions, the corresponding closed-form solution for the time factors and
trajectory profiles are derived. The proposed algorithm was first derived in Mathematica and then converted into
a C++ implementation. Finally, the algorithm was utilized and demonstrated in ROS & Gazebo using a UR3 robot.

The C++ trajectory planner code cab be accessed at " ./src/Jerk-continuous-online-trajectory-generator-with-constraints/ROS/robot_controller/src/OTG/"

## Quick start
. Open a ROS terminal, cd to ros_workspace
. To run execute following command to start the simulation
```
$ ./src/Jerk-continuous-online-trajectory-generator-with-constraints/ROS/script/start_simulation_ur3.sh
```
. After robot arm move to the initial position

. Open another ROS terminal, cd to ros_workspace
. To run execute following command
```
$ ./src/Jerk-continuous-online-trajectory-generator-with-constraints/ROS/script/start_servo.sh
```
## System Configuration

### Hardware Configuration

. A standard PC.

### Software Configuration

. Ubuntu [18.04 LTS, Melodic] as Host OS.

## Ubuntu [18.04 LTS, Melodic]

### ROS Setup

. Install ROS melodic by following the instruction found in the official website here (Install all tools):
 http://wiki.ros.org/melodic/Installation/Ubuntu
. Install caktin tools by following the instruction found in the official website here:
https://catkin-tools.readthedocs.io/en/latest/installing.html
. Install moveit by following the instruction found in the official website here:
https://moveit.ros.org/install/
. Install Eigen 3 (math library), using command
```
$ sudo apt install libeigen3-dev
```
. Install ros-control using command:
```
$ sudo apt-get install ros-melodic-ros-control
```
. install ros-controller using command:
```
$ sudo apt-get install ros-melodic-ros-controllers
```
### Robot Controller source download and compilation

### Download and install Ruckig
. Install Ruckig OTG libray by following the instruction found in the official github:
https://github.com/pantor/ruckig

. Install Ruckig in a system system-wide directory
```
$ sudo make install
```
### Running binaries

. Open terminal, cd to ros_workspace
. To source required environmental variables, execute the command
```
$ source devel/setup.bash
```
. Separate terminals are required to run  gazebo environment, moveit and the controller. So follow previous two step in all three terminals.
. To run gazebo execute the command:
```
$ roslaunch ur3_gazebo ur3_gazebo_controller.launch
```
. To run moveit, execute the command:
```
$ roslaunch ur3_moveit_config custom.launch
```
. To run the controller, execute the command:
```
$ rosrun robot_controller robot_controller
```
### ROS Setup

. Install visual studio - 2019 community version if not installed
. Install ROS for windows as described in the following link:
http://wiki.ros.org/Installation/Windows
. Install moveit for windows as described in the following link:
https://moveit.ros.org/install/
. The above step install all dependencies as well as gazebo
. Create ROS  commandline terminal shortcut as in step 2
. Install Win10SDK_10.0.18362 from visual studio

### Source download and compilation

. Clone git repository from “https://github.com/Haoran-Zhao/Jerk-continuous-online-trajectory-generator-with-constraints.git” or using command :
```
$ git clone https://github.com/Haoran-Zhao/Jerk-continuous-online-trajectory-generator-with-constraints.git
```
. Source code is present in "Jerk-continuous-online-trajectory-generator-with-constraints" folder.
. Build and compile the project using CMake and Visual Studio.
