#pragma once

#include<ros/ros.h>
#include <ros/package.h>


#include<gazebo_msgs/SetModelState.h>
#include<gazebo_msgs/GetModelState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <gazebo_msgs/SpawnModel.h>

#include<iostream>
#include<fstream>
#include <tf/transform_datatypes.h>

using namespace std;

/**
* Responsible for creating, deleting and updating model positions and orientation in Gazebo
*/
class GazeboModelState {
public:

    /**
    * Constructor
    * Creates GazeboModelState object
    */
    GazeboModelState();

    /**
    * Destructor
    * Creates GazeboModelState object
    */
    ~GazeboModelState();

    /**
	*	Initializes GazeboModelState object
	*	@param node_ptr pointer to ros NodeHandle object
	*/
    void initialize(ros::NodeHandle *node_ptr);

    /**
    *	Renders (Updates) the position of the given model to a given positon (With respect to the model)
    *	@param model_name Name of the model that needs to be rendered (Updated)
    *	@param pose Position of where the model needs to be placed
    */
    void set_model_position(std::string model_name, geometry_msgs::Pose pose);

    /**
    *	Renders (Updates) the position of the given model to a given positon (With respect to the world)
    *	@param model_name Name of the model that needs to be rendered (Updated)
    *	@param pose Position of where the model needs to be placed
    */
    void set_model_position_world(std::string model_name, geometry_msgs::Pose pose);

    /**
    *	Gets the position of the specified model (With respect to itself)
    *	@param model_name Name of the model
    */
    geometry_msgs::Pose get_model_position(std::string model_name);

    /**
    *	Gets the position of the specified model (With respect to world)
    *	@param model_name Name of the model
    */
    geometry_msgs::Pose get_model_position_world(std::string model_name);

    /**
    *	Creates and renders a spherical model in the gazebo world
    *	@param name Name of the frame
    *	@param pose Position of the frame
    */
    void create_frame(std::string name, geometry_msgs::Pose pose);

private:

    /**
    *	Points to the ros NodeHandle
    */
    ros::NodeHandle *_node_ptr;

    /**
    *	ros service that sends the position where a model needs to be updated
    */
    ros::ServiceClient _get_model_state_client;

    /**
    *	ros service that gets the position of a model
    */
    ros::ServiceClient _set_model_state_client;

    /**
    *	ros service that sends creates a new model
    */
    ros::ServiceClient _spawn_model_client;

    /**
    *	Set false to disable incision frame, true otherwise
    */
    bool incision_frames = false;
};
