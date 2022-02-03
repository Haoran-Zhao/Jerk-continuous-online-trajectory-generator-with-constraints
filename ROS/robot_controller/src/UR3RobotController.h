#pragma once

//ROS and Moveit header file
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <visualization_msgs/Marker.h>
#include <Eigen/Core>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <iostream>
#include <string>
#include <thread>
#include <mutex>

#include "GazeboMarker.h"
#include "GazeboModelState.h"
#include "UR3PathComputation.h"
#include "Semaphore.h"
#include "robot_controller/RotationAngle.h"
#include "RobotController.h"



using namespace std;

/**
* Responsible for controlling the movements of the robot, This is the main class which links all other classes
*/
class UR3RobotController : public RobotController {

public:
    /**
    * Constructor
    * Creates UR3RobotController object
    */
    UR3RobotController();

    /**
    * Destructor
    * Called when UR3RobotController object get destroyed
    */
    ~UR3RobotController();

    /**
    * Initializes UR3RobotController object
    */
    bool initialize(int argc, char **argv) override;

    /**
    * Resets the cylinder position and orientation to match scope tip
    */
    void reset_cylinder() override;


private:

    ros::Publisher _target_pose_pub;

   /**
   * Moves the robot through a list of specified waypoints moveit trajectory planner
   * @param wayPoints List of waypoints
   * @returns True if successful, false otherwise
   */
    bool _move_robot_to_pose(vector<geometry_msgs::Pose> end_effector_way_points) {};

};
