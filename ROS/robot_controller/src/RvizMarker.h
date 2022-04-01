#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <iostream>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>



class RvizMarker {
public:
    /**
	 * Constructor
	 */
    RvizMarker();

    /**
	 * Destructor
	 * Called when RvizMarker object get destroyed
	 */
    ~RvizMarker();

    /**
    * Initializes RvizMarker object. Creates and sets various member variables
    * @param node_ptr Pointer to ros NodeHandle object
    * @param move_group_ptr Pointer to moveit plannint interface
    */
    void initialize(ros::NodeHandle *node_ptr, moveit::planning_interface::MoveGroupInterface *move_group_ptr);

    /**
    * Publishes sphere marker to the topic
    * @param pose Pose of the sphere
    * @param r red value
    * @param g green value
    * @param b blue value
    * @param raduis radius of the sphere
    * @param alpha opacity of the sphere
    */
    void publish_sphere(geometry_msgs::Pose pose);

    /**
    * Continuesly publishes the current robot end effector position
    */
    void _publish_end_effector_position();
    /**
    * Publishes cylinder marker to the topic
    * @param pose Pose of the cylinder
    */
    void publish_cylinder(geometry_msgs::Pose pose);
    void _update_target_position(geometry_msgs::Pose pose);
    void Callback(const visualization_msgs::MarkerArray& state);

    geometry_msgs::Pose _target_pose;
private:

    /**
	 * Ros publisher to publish the messaging to the topic
	 */
    ros::Publisher _vis_pub;
    ros::Publisher _vis_pub1;
    ros::Subscriber _vis_sub1;
    /**
	 * Stores the marker id
	 */
    int _sphere_id;
    /**
   * Stores the marker id
   */
    int _cylinder_id;
    /**
	 * Stores the end effector marker id
	 */
    int _end_effector_id;

    /**
    *	Points to the ros moveit planning interface
    */
    moveit::planning_interface::MoveGroupInterface *_move_group_ptr;

    /**
    *	Points to the ros NodeHandle
    */
    ros::NodeHandle *_node_ptr;

    /**
	 * Thread that continuesly publishes end effector position
	 */
    std::thread _publish_end_effector_position_thread;
};
