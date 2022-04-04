// Copyright (c) 2020 SURGE, All rights reserved.
/**
 * @file       AngulatedRobotController.cpp
 * @class      AngulatedRobocompute_end_effector_waypointstController
 * @brief      Responsible for controlling robot.
 */
/*=============================================================================
==================================================================================*/

#include "UR3RobotController.h"
#include "Utilities.h"

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Constructor
*/
UR3RobotController::UR3RobotController() {
  _cylinder_angle = 0.523599; // 30 degree in radian  0.523599
  _scope_length = 0.37;
  _compute_path = new UR3PathComputation();
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Destructor
*/
UR3RobotController::~UR3RobotController() {

}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Initializes the node handle, async spinner, publishers and thread
*/
bool UR3RobotController::initialize(int argc, char **argv)
{
    bool status = true;
    try
    {
        RobotController::initialize(argc, argv);
    }
    catch (string exception)
    {
        cout << "Exception : " << exception << endl;
        status = false;
    }

    _target_pose_pub = _node_handle->advertise<geometry_msgs::PoseStamped>("/waypoints", 1 , true);

    return status;
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Resets the cylinder pose in the gazebo envionment to match the view of the scope tip camera
*/
void UR3RobotController::reset_cylinder(){
    while(_new_position_available_sem->getCount() != 0)
        continue;

    //Get cylinder pose
    geometry_msgs::Pose cylinder_pose = _model_state.get_model_position_world("targetCylinder");

    //Get ee pose
    geometry_msgs::Pose ee_pose = _move_group_ptr->getCurrentPose().pose;

    //Setting the cylinder matrix pose in the gazebo environment
    _model_state.set_model_position_world("targetCylinder", ee_pose);
}
