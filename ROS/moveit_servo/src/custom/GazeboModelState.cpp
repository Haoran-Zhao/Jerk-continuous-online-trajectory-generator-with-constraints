// Copyright (c) 2020 SURGE, All rights reserved. 
/**
 * @file       GazeboModelState.cpp
 * @class      GazeboModelState
 * @brief      Responsible for controlling state of the gazebo model
 */
/*=============================================================================
 * Change history:
 * ----------------------------------------------------------------------------
 * Revision DATE            BY           		SPR/US        REMARKS
 * 1.0      24-May-2020    	J.Padhan/Nihal 		NA 		   	  Initial Version
 * 2.0      30-Oct-2020    	Nihal 			    NA 		   	  Code restructuring and documentation
 ==================================================================================*/

#include "GazeboModelState.h"

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Constructor
*/
GazeboModelState::GazeboModelState() {

}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Destructor
*/
GazeboModelState::~GazeboModelState() {
}

/*
	IN 			: Ros Node Handle
	OUT 		: None
	DESCRIPTION	: Initialize node handle and service clients
*/
void GazeboModelState::initialize(ros::NodeHandle *node_ptr) {
    //Create pointer to node handle
    _node_ptr = node_ptr;

    //Creates a ros service client to set model states in gazebo
    _set_model_state_client = _node_ptr->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state", true);

    //Creates a ros service client to get model states in gazebo
    _get_model_state_client = _node_ptr->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true);

    //Creates a ros service client to spawn models in gazebo
    _spawn_model_client = _node_ptr->serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model", true);
}

/*
	IN 			: Name of the model and the pose that needs to be applied to the model
	OUT 		: None
	DESCRIPTION	: Calls gazebo service to move the model to the desired pose in its own reference frame
*/
void GazeboModelState::set_model_position(std::string model_name, geometry_msgs::Pose model_pose) {
    //Creates a new model state
    gazebo_msgs::ModelState model_state;

    //Set the model name
    model_state.model_name = model_name;

    //Set the model pose
    model_state.pose = model_pose;

    //Set the reference frame
    model_state.reference_frame = model_name;

    //Create gazebo SetModelState service message
    gazebo_msgs::SetModelState srv;

    //Set the model state
    srv.request.model_state = model_state;

    //Call client to set the model state
    if (_set_model_state_client.call(srv)) {
        //cout << "Setting model state success!!" << endl;
    } 
}

/*
	IN 			: Name of the model and the pose that needs to be applied to the model
	OUT 		: None
	DESCRIPTION	: Calls gazebo service to move the model to the desired pose in the world frame
*/
void GazeboModelState::set_model_position_world(std::string model_name, geometry_msgs::Pose model_pose) {
    //Creates a new model state
    gazebo_msgs::ModelState model_state;

    //Set the model name
    model_state.model_name = model_name;

    //Set the model pose
    model_state.pose = model_pose;

    //Set the reference frame
    model_state.reference_frame = "world";

    //Create gazebo SetModelState service message
    gazebo_msgs::SetModelState srv;
    srv.request.model_state = model_state;

    //Call client to set the model state
    // if (_set_model_state_client.call(srv)) {
    //     //cout << "Setting model state success!!" << srv.response.status_message << std::endl;
    // } 
}

/*
	IN 			: Name of the model for which the pose needs to be retrieved
	OUT 		: None
	DESCRIPTION	: Calls gazebo service to retrieve the model pose in its own frame
*/
geometry_msgs::Pose GazeboModelState::get_model_position(std::string model_name) {
    //Create a pose object
    geometry_msgs::Pose pose;

    //Create gazebo GetModelState service message
    gazebo_msgs::GetModelState srv;

    //Set the name of the mode to get the position of
    srv.request.model_name = model_name;

    //Set the reference frame
    srv.request.relative_entity_name = model_name;

    //Call client to set the model state
    // if (_get_model_state_client.call(srv)) {
    //     //Set the pose
    //     pose = srv.response.pose;
    // } 

    return pose;
}

/*
	IN 			: Name of the model for which the pose needs to be retrieved
	OUT 		: None
	DESCRIPTION	: Calls gazebo service to retrieve the model pose in the world frame
*/
geometry_msgs::Pose GazeboModelState::get_model_position_world(std::string model_name) {
    //Create a pose object
    geometry_msgs::Pose pose;

    //Create gazebo GetModelState service message
    gazebo_msgs::GetModelState srv;

    //Set the name of the mode to get the position of
    srv.request.model_name = model_name;

    //Set the reference frame
    srv.request.relative_entity_name = "world";

    //Call client to set the model state
    // if (_get_model_state_client.call(srv)) {
    //     //Set the pose
    //     pose = srv.response.pose;
    // } 
    return pose;
}

/*
	IN 			: Name of the frame that needs to be created, Pose of the frame
	OUT 		: None
	DESCRIPTION	: Calls gazebo service to create the frame at the desired pose
*/
void GazeboModelState::create_frame(std::string name, geometry_msgs::Pose pose) {
    //Create gazebo SpawnModel service message
    gazebo_msgs::SpawnModel srv;

    //Create file stream
    std::ifstream ifs;

    //Open frame.model file, which contains the model specification
    ifs.open(ros::package::getPath("r5_gazebo") + "/models/frame.model");
    if (!ifs) {
        throw std::runtime_error("failed to open model file");
    }

    //Create string stream
    std::stringstream str_stream;

    //Read the file
    str_stream << ifs.rdbuf();

    //Set the model xml 
    srv.request.model_xml = str_stream.str();

    //Set the model name
    srv.request.model_name = name;

    //Set the model pose
    srv.request.initial_pose = pose;

    //Set the model reference frame
    srv.request.reference_frame = "world";

    //Call the service
    // if (_spawn_model_client.call(srv)) {
    //     // cout << "Spawnig model state success!! " << srv.response.status_message << ","<< srv.response.success << endl;
    // } 
}