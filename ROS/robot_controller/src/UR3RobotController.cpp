// Copyright (c) 2020 SURGE, All rights reserved.
/**
 * @file       AngulatedRobotController.cpp
 * @class      AngulatedRobocompute_end_effector_waypointstController
 * @brief      Responsible for controlling robot.
 */
/*=============================================================================
 * Change history:
 * ----------------------------------------------------------------------------
 * Revision DATE            BY           		SPR/US        REMARKS
 * 1.0      10-May-2020    	J.Padhan/Nihal 		NA 		   	  Initial Version
 * 2.0      24-May-2020     Nihal				NA			  Fixed orientation.
 * 3.0      24-May-2020     J.Padhan            NA            Added Marker.
 * 4.0      24-Oct-2020     J.Padhan            NA            Made path replacement configurable
 * 5.0      30-Oct-2020     Nihal               NA            Restructed the code
==================================================================================*/

#include "UR3RobotController.h"
#include "Utilities.h"

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Constructor
*/
UR3RobotController::UR3RobotController() {
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
