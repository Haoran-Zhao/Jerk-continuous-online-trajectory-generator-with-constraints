#pragma once

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <string>
#include <vector>
#include "std_msgs/String.h"

#include "GazeboMarker.h"
#include "GazeboModelState.h"
#include "Utilities.h"
#include "PathComputation.h"
#include <robot_controller/ScopeTip.h>


#include <cmath>
/**
* Responsible for computing and interpolating the robot path based on algorithms
*/
class UR3PathComputation : public PathComputation {

public:
    /**
	 * Constructor
	 * Creates AngulatedPathComputation object
	 */
    UR3PathComputation();

    /**
    * Destructor
    * Called when AngulatedPathComputation object get destroyed
    */
    ~UR3PathComputation();

    /**
    * Initializes AngulatedPathComputation object. Creates and sets various member variables
    * @param nodePtr Pointer to ros NodeHandle object
    * @param gazeboState Pointer to GazeboModelState object
    * @param markerEnabled Set to true to enable markers, false otherwise
    */
    void initialize(ros::NodeHandle *nodePtr, GazeboModelState *model_state, Eigen::Matrix4d current_end_effector_matrix) override;

    Eigen::Vector3d compute_velocity(Eigen::Matrix4d start_tip, Eigen::Matrix4d end_tip,
        Eigen::Matrix4d incision_matrix, Eigen::Matrix4d start_end_effector, Eigen::Matrix4d final_end_effector);

    double calculate_line_to_point_distance(Eigen::Vector3d p, Eigen::Vector3d a, Eigen::Vector3d b);

    Eigen::Vector3d compute_delta(Eigen::Matrix4d incision_matrix, Eigen::Matrix4d start_matrix, Eigen::Matrix4d end_matrix);

    Eigen::Vector3d compute_end_effector_position(Eigen::Vector3d Xa, Eigen::Vector3d Xb, Eigen::Vector3d Xc, double lamda);

    Eigen::Vector3d compute_end_effector_orientation(Eigen::Vector3d Xa, Eigen::Vector3d Xb, Eigen::Vector3d Xc, double lamda,
                                                        Eigen::Matrix4d start_end_effector, Eigen::Matrix4d final_end_effector);

private:
    /*
    * Contains the current end effector matrix
    */
    Eigen::Matrix4d _current_end_effector_matrix;

    /**
    * Interpolates from the current robot matrix to the target robot matrix
    * @param target_matrix The target location
    * @param num_of_interpolation_points Number of points for interpolation
    * @returns list of 4x4 matrix
    */
    std::vector<Eigen::Matrix4d> _interpolate_target_matrix(Eigen::Matrix4d target_matrix, int num_of_interpolation_points);

    Eigen::Matrix4d _rotate_end_effector_matrix(Eigen::Matrix4d end_effector_matrix);


    GazeboMarker _marker;
};
