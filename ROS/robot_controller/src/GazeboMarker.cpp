// Copyright (c) 2020 SURGE, All rights reserved. 
/**
 * @file       GazeboMarker.cpp
 * @class      GazeboMarker
 * @brief      Responsible for darwing markers in gazebo.
 */
/*=============================================================================
 * Change history:
 * ----------------------------------------------------------------------------
 * Revision DATE            BY           		SPR/US        REMARKS
 * 1.0      19-May-2020    	J.Padhan/Nihal 		NA 		   	  Initial Version
 * 2.0      30-Oct-2020    	Nihal 			    NA 		   	  Code restructuring and documentation
==================================================================================*/

#include "GazeboMarker.h"
/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Constructor
*/
GazeboMarker::GazeboMarker()
{
	_marker_msg.set_ns("default");
	_mat_msg = _marker_msg.mutable_material();
	
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Destructor
*/
GazeboMarker::~GazeboMarker()
{
	erase_markers();
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Sends request to clear all the markers currently in the gazebo environment
*/
void GazeboMarker::erase_markers()
{
	_marker_msg.set_action(ignition::msgs::Marker::DELETE_ALL);
	_node.Request("/marker", _marker_msg);
}

/*
	IN 			: Pose of the sphere marker
	OUT 		: None
	DESCRIPTION	: Sends request to create a sphere in gazebo environment at the specified pose
*/
void GazeboMarker::draw_sphere_marker(geometry_msgs::Pose pose)
{
	//Sets the id the marker
	_marker_msg.set_id(MessageID::SPHERE);

	//Sets the action that needs to be done
	_marker_msg.set_action(ignition::msgs::Marker::ADD_MODIFY);

	//Sets the type of the marker
	_marker_msg.set_type(ignition::msgs::Marker::SPHERE);

	//Sets the color of the marker
	_mat_msg->mutable_script()->set_name("Gazebo/Red");

	//Sets the scale of the marker
	ignition::msgs::Set(_marker_msg.mutable_scale(),
                    ignition::math::Vector3d(0.05, 0.05, 0.05));

	//Sets the pose of the marker
	ignition::msgs::Set(_marker_msg.mutable_pose(),
                    ignition::math::Pose3d(pose.position.x,pose.position.y,pose.position.z,pose.orientation.x,pose.orientation.y,pose.orientation.z));

	//Send request to create the marker in gazebo environment				
	_node.Request("/marker", _marker_msg);
}

/*
	IN 			: Pose of the the start of the line and end of the line
	OUT 		: None
	DESCRIPTION	: Sends request to create a line in gazebo environment at the specified pose
*/
void GazeboMarker::draw_line_marker(geometry_msgs::Pose start_pose, geometry_msgs::Pose end_pose, bool clear)
{
	//Check is clear flag is true, if rue clear all the markers
	if(clear)
	    _marker_msg.clear_point();
	
	//Sets the id the marker
	_marker_msg.set_id(MessageID::LINE);

	//Sets the action that needs to be done
	_marker_msg.set_action(ignition::msgs::Marker::ADD_MODIFY);

	//Sets the type of the marker
    _marker_msg.set_type(ignition::msgs::Marker::LINE_LIST);

	//Sets the color of the marker
	_mat_msg->mutable_script()->set_name("Gazebo/BlueLaser");

	//Creates the set of lines at the speicified poses
	ignition::msgs::Set(_marker_msg.add_point(),ignition::math::Vector3d(start_pose.position.x,start_pose.position.y,start_pose.position.z));
	ignition::msgs::Set(_marker_msg.add_point(),ignition::math::Vector3d(end_pose.position.x,end_pose.position.y,end_pose.position.z));

	//Send request to create the marker in gazebo environment				
	_node.Request("/marker", _marker_msg);
}

/*
	IN 			: Vector of the the start of the line and end of the line
	OUT 		: None
	DESCRIPTION	: Sends request to create a line in gazebo enviroment at the specified pose
*/
void GazeboMarker::draw_line_marker(Eigen::Vector3d start_vector, Eigen::Vector3d end_vector, bool clear)
{
    if(clear)
        _marker_msg.clear_point();
    _marker_msg.set_id(MessageID::LINE);
    _marker_msg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    _marker_msg.set_type(ignition::msgs::Marker::LINE_LIST);
    _mat_msg->mutable_script()->set_name("Gazebo/BlueLaser");
    ignition::msgs::Set(_marker_msg.add_point(),ignition::math::Vector3d(start_vector.x(),start_vector.y(),start_vector.z()));
    ignition::msgs::Set(_marker_msg.add_point(),ignition::math::Vector3d(end_vector.x(),end_vector.y(),end_vector.z()));
    _node.Request("/marker", _marker_msg);
}
