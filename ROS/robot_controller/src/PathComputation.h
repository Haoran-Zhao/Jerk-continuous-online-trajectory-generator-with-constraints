#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <ros/node_handle.h>
#include "GazeboModelState.h"
#include "GazeboMarker.h"

class PathComputation {
public:
    /**
    * Constructor
    */
    PathComputation();

    /**
    * Destructor
    */
    ~PathComputation();

    /**
    * Initializes UR3PathComputation object. Creates and sets various member variables
    * @param nodePtr Pointer to ros NodeHandle object
    * @param gazeboState Pointer to GazeboModelState object
    * @param markerEnabled Set to true to enable markers, false otherwise
    */
    void virtual initialize(ros::NodeHandle *nodePtr, GazeboModelState *model_state, Eigen::Matrix4d current_end_effector_matrix) = 0;

    /**
    * Rotates a point along the given line
    */
    Eigen::Vector3d rotate(Eigen::Vector3d p, double theta, Eigen::Vector3d p1, Eigen::Vector3d p2);
protected:
    /**
    * Points to the GazeboModelState object to set and receive model information from gazebo environment
    */
    GazeboModelState *_model_state;

    /**
    * Gazebo marker object used to create markers in gazebo
    */
    GazeboMarker _marker;

    /**
    * Set to true to enable marker, false otherwise
    */
    bool _marker_enabled;

    /**
    * Sets to true if the path frames are spawned, false otherwise
    */
    bool _path_frame_created;

    /**
    *	Points to the ros NodeHandle
    */
    ros::NodeHandle *_node_ptr;

    bool _debug = false;
};
