// Copyright (c) 2020 SURGE, All rights reserved.
/**
 * @file       RvizMarker.cpp
 * @class      RvizMarker
 * @brief      Responsible for publishes rviz markers.
 */
/*=============================================================================
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
    _sphere_id = 0;
    _cylinder_id = 1;
    _vis_pub = _node_ptr->advertise<visualization_msgs::Marker>( "/visualization_marker", 0 );
    _vis_pub1 = _node_ptr->advertise<visualization_msgs::MarkerArray>( "/visualization_marker_array", 0 );
    //_vis_sub1 = _node_ptr->->subscribe( "/visualization_marker_array", 1,  &RvizMarker::Callback, this);
    _end_effector_id = 1000;
    _publish_end_effector_position_thread = std::thread(&RvizMarker::_publish_end_effector_position, this);

}

void RvizMarker::Callback(const visualization_msgs::MarkerArray& state){
    visualization_msgs::Marker marker = state.markers[0];
    geometry_msgs::Pose current_pos = marker.pose;
}

/*
	IN 			: Pose of the sphere, red value, green value, b value, radius of the sphere, alpha of the sphere
	OUT 		: None
	DESCRIPTION	: Creates a sphere marker message at the specified pose, color and radius, then publish it to the topic
*/
void RvizMarker::publish_sphere(geometry_msgs::Pose pose){
    //Creates the marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.id = _sphere_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action =  visualization_msgs::Marker::ADD;
    marker.pose = pose;

    //Set the scale of the marker
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1;

    //Set the color of the marker
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;

    //publish the marker to the topic
    _vis_pub.publish( marker );

    //Update the marker id to be used when called again later
    //_marker_id+=1;
}

/*
	IN 			: Pose of the sphere, red value, green value, b value, radius of the sphere, alpha of the sphere
	OUT 		: None
	DESCRIPTION	: Creates a sphere marker message at the specified pose, color and radius, then publish it to the topic
*/
void RvizMarker::publish_cylinder(geometry_msgs::Pose pose){
    //Creates the marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.id = _cylinder_id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action =  visualization_msgs::Marker::ADD;
    marker.pose = pose;

    //Set the scale of the marker
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.scale.z = 0.008;

    //Set the color of the marker
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    //publish the marker to the topic
    _vis_pub.publish( marker );

    //Update the marker id to be used when called again later
    //_marker_id+=1;
}

void RvizMarker::_update_target_position(geometry_msgs::Pose pose){
  _target_pose = pose;
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

        //Creates the marker
        visualization_msgs::Marker marker1;
        marker1.header.frame_id = "world";
        marker1.id = _cylinder_id;
        marker1.type = visualization_msgs::Marker::CYLINDER;
        marker1.action =  visualization_msgs::Marker::ADD;
        marker1.pose = _target_pose;

        //Set the scale of the marker
        marker1.scale.x = 0.01;
        marker1.scale.y = 0.01;
        marker1.scale.z = 0.02;

        //Set the color of the marker
        marker1.color.a = 1.0;
        marker1.color.r = 0.0;
        marker1.color.g = 1.0;
        marker1.color.b = 0.0;

        //Creates the marker
        visualization_msgs::Marker marker2;
        marker2.header.frame_id = "world";
        marker2.id = _sphere_id;
        marker2.type = visualization_msgs::Marker::SPHERE;
        marker2.action =  visualization_msgs::Marker::ADD;
        marker2.pose = robot_pose;

        //Set the scale of the marker
        marker2.scale.x = 0.015;
        marker2.scale.y = 0.015;
        marker2.scale.z = 0.015;
        marker2.color.a = 1;

        //Set the color of the marker
        marker2.color.r = 1;
        marker2.color.g = 0;
        marker2.color.b = 0;

        visualization_msgs::MarkerArray rviz_markers;

        rviz_markers.markers.push_back(marker1);
        rviz_markers.markers.push_back(marker2);
        _vis_pub1.publish( rviz_markers );
        //Sleep for 30 millisecond
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
