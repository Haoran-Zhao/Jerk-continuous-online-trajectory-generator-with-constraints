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

#include "EndoCameleonRobotController.h"
#include "Utilities.h"

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Constructor
*/
EndoCameleonRobotController::EndoCameleonRobotController() {
    _cylinder_angle = 0.523599; // 30 degree in radian  0.523599
    _scope_length = 0.37;
    _compute_path = new EndoCameleonPathComputation();
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Destructor
*/
EndoCameleonRobotController::~EndoCameleonRobotController() {

}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Initializes the node handle, async spinner, publishers and thread
*/
bool EndoCameleonRobotController::initialize(int argc, char **argv)
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
    _camera_joint_publisher = _node_handle->advertise<std_msgs::Float64>("/camera_controller/command", 1);

    _target_pose_pub = _node_handle->advertise<geometry_msgs::PoseStamped>("/waypoints", 1 , true); 

    return status;
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Clears the way point list and calls compute path function of the AngulatedPathComputation object
*/
void EndoCameleonRobotController::compute_path_func() {
    //Compute the path
    _end_effector_pose_list = _compute_path->compute_end_effector_waypoints(_articulation_angle, _rotation_angle, _target_matrix);
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Resets the cylinder pose in the gazebo envionment to match the view of the scope tip camera
*/
void EndoCameleonRobotController::reset_cylinder(){
    while(_new_position_available_sem->getCount() != 0)
        continue;

    //Get cylinder pose
    geometry_msgs::Pose cylinder_pose = _model_state.get_model_position_world("ScopeCylinder");
    
    //Get scope tip pose
    geometry_msgs::Pose scope_tip_pose = _model_state.get_model_position_world("ScopeTipSphere");

    //Convert scope tip pose to 4x4 matrix
    Eigen::Matrix4d scope_top_matrix = Utilities::pose_to_matrix(scope_tip_pose);
    
    //X direction of the scope tip matrix
    Eigen::Vector3d scope_tip_frame_X = scope_top_matrix.block<3,1>(0,0);
    
    //Y direction of the scope tip matrix
    Eigen::Vector3d scope_tip_frame_Y = scope_top_matrix.block<3,1>(0,1);

    //Z direction of the scope tip matrix
    Eigen::Vector3d scope_tip_frame_Z = scope_top_matrix.block<3,1>(0,2);
    
    //Vector at the origin
    Eigen::Vector3d origin_vector(0,0,0);

    //Rotate the scope_tip_frame_Z
    scope_tip_frame_X = _compute_path->rotate(scope_tip_frame_X, _rotation_angle*(M_PI/180), origin_vector, scope_tip_frame_Z);
    scope_tip_frame_X = scope_tip_frame_X.normalized();

    //Get scope tip y direction as the cross product of X and Z frame
    scope_tip_frame_Y = (scope_tip_frame_Z.cross(scope_tip_frame_X)).normalized();
    
    //Update the scope tip matrix 
    scope_top_matrix.block<3,1>(0,0)  = scope_tip_frame_X;
    scope_top_matrix.block<3,1>(0,1)  = scope_tip_frame_Y;

    //Converting 4x4 scope tip matrix back to pose
    scope_tip_pose = Utilities::matrix_to_pose(scope_top_matrix);

    //Setting the cylinder matrix pose in the gazebo environment
    _model_state.set_model_position_world("ScopeCylinder", scope_tip_pose);
}