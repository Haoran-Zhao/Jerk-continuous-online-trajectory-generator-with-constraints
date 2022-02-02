#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class Utilities {

public:
    /**
    * Constructor
    * Creates Utilities object
    */
    Utilities();

    /**
    * Destructor
    */
    ~Utilities();

    /**
    * Converts a pose message to 4x4 matrix
    */
    static Eigen::Matrix4d pose_to_matrix(geometry_msgs::Pose pose);

    /**
    * Converts a pose message to 4x4 matrix frame to pose message
    */
    static geometry_msgs::Pose matrix_to_pose(Eigen::Matrix4d matrix_frame);

    /**
    * Combines rotation matrix and translation vector to pose message
    * @param rot_mat Rotation matrix
    * @param trans_vector Translation vector
    * @returns pose message
    */
    static geometry_msgs::Pose matrix_to_pose(Eigen::Matrix3d rot_mat, Eigen::Vector3d trans_vector);

    /**
    * Converts Eigen ector to pose message
    * @param vector Eigen vector
    * @returns pose message
    */
    static geometry_msgs::Pose vector_to_pose(Eigen::Vector3d vector);

    /**
    * Combines rotation matrix and translation vector to 4x4 matrix
    * @param rotation_matrix Rotation matrix
    * @param translation_vector Translation vector
    * @returns 4x4 matrix
    */
    static Eigen::Matrix4d combine_rotation_position_to_matrix(Eigen::Matrix3d rotation_matrix, Eigen::Vector3d translation_vector);

    /**
    * Copies pose from source to destination
    * @param source Source pose
    * @param dest Destination variable
    */
    static void copy_geometry_pose(geometry_msgs::Pose source, geometry_msgs::Pose &dest);

    /**
    * Interpolates from the start quaternion to end quaternion
    * @param start_vector The starting quaternion of interpolation
    * @param end_vector The ending quaternion of interpolation
    * @param num_of_interpolation_points Number of points for interpolation
    * @returns list of 4x4 matrix
    */
    static std::vector<Eigen::Quaterniond> interpolate_orientation(Eigen::Quaterniond start_quat, Eigen::Quaterniond end_quat, int num_of_interpolation_points);

    /**
     * Interpolates from the start vector to end vector
     * @param start_vector The starting point of interpolation
     * @param end_vector The ending point of interpolation
     * @param num_of_interpolation_points Number of points for interpolation
     * @returns list of 4x4 matrix
     */
    static std::vector<Eigen::Vector3d> interpolate_position(Eigen::Vector3d start_vector, Eigen::Vector3d end_vector, int num_of_interpolation_points);

    /**
    * Compates two quaternions for equality
    * @param quat1 First quaternion
    * @param quat2 Second quaternion
    * @returns True if equal, false otherwise
    */
    static bool compare(Eigen::Quaterniond quat1, Eigen::Quaterniond quat2);

};