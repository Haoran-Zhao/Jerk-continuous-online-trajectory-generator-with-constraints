// Copyright (c) 2020 SURGE, All rights reserved.
/**
 * @file       RvizMarker.cpp
 * @class      RvizMarker
 * @brief      Responsible for publishes rviz markers.
 */
/*=============================================================================
 * Change history:
 * ----------------------------------------------------------------------------
  * Revision DATE            BY           		SPR/US        REMARKS
 * 1.0      28-Oct-2020    	Nihal 		        NA 		   	  Initial Version
 * 2.0      30-Oct-2020     Nihal				NA			  Code documentation
==================================================================================*/
#include "RvizMarker.h"

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Constructor
*/
RvizMarker::RvizMarker()
{
  
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Destructor
*/
RvizMarker::~RvizMarker()
{
	
}

/*
	IN 			: Ros node handle, move group interface pointer
	OUT 		: None
	DESCRIPTION	: Initializes the node ptr, move group interface, ros publihers, and thread
*/
void RvizMarker::initialize(ros::NodeHandle *node_ptr, moveit::planning_interface::MoveGroupInterface *move_group_ptr) {
    _move_group_ptr = move_group_ptr;
    _node_ptr = node_ptr;
    _marker_id = 0;
    _vis_pub = _node_ptr->advertise<visualization_msgs::Marker>( "/visualization_marker", 0 );
    _end_effector_id = 1000;
    _publish_end_effector_position_thread = std::thread(&RvizMarker::_publish_end_effector_position, this);

}

/*
	IN 			: Pose of the sphere, red value, green value, b value, radius of the sphere, alpha of the sphere
	OUT 		: None
	DESCRIPTION	: Creates a sphere marker message at the specified pose, color and radius, then publish it to the topic
*/
void RvizMarker::publish_sphere(geometry_msgs::Pose pose, double r, double g, double b, double radius, double alpha){
    //Creates the marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.id = _marker_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action =  visualization_msgs::Marker::ADD;
    marker.pose = pose;

    //Set the scale of the marker
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;
    marker.color.a = alpha; 

    //Set the color of the marker
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;

    //publish the marker to the topic
    _vis_pub.publish( marker );

    //Update the marker id to be used when called again later
    _marker_id+=1;
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Continuesly get the robot current pose, creates a marker at that pose and publishes it to the topic
*/
void RvizMarker::_publish_end_effector_position(){
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    while(true){
        //Get Current robot pose
        geometry_msgs::Pose robot_pose = _move_group_ptr->getCurrentPose().pose;

        //Create marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.id = _end_effector_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action =  visualization_msgs::Marker::ADD;
        marker.pose = robot_pose;

        //Set the scale of the marker
        marker.scale.x = 0.005;
        marker.scale.y = 0.005;
        marker.scale.z = 0.005;

        //Set the color of the marker
        marker.color.a = 1.0; 
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        //Publish the marker to the topic
        _vis_pub.publish(marker);

        //Update the marker id to be used for the next iteration
        _end_effector_id +=1;

        //Sleep for 30 millisecond
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}
