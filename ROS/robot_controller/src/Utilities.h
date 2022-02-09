#pragma once
#define _USE_MATH_DEFINES
#include <iostream>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ruckig/ruckig.hpp>
#include <TrigonometricOTG.h>
#include <numeric>
#include <math.h>
#include <cmath>

using namespace std;

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
    int idx_;

    static void correct_joint_range(double& current_x, double& current_y, double& current_z, double& target_x, double& target_y, double& target_z);

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

    double get_angle_betweem_two_vectors(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d n);

    static double Repeat(double t, double length);

    static double DeltaAngle(double current, double target);

    static double clamp(double n, double lower, double upper);

    static double SmoothDamp(double current, double target , double &currentVelocity, double smoothTime,
                  double maxSpeed, double deltaTime);

    static double SmoothDampAngle(double current, double target,  double &currentVelocity, double smoothTime,
                  double maxSpeed,  double deltaTime);

    static Eigen::Vector3d SmoothDampVector(Eigen::Vector3d current, Eigen::Vector3d target, Eigen::Vector3d &currentVelocity, double smoothTime,
                                 double maxSpeed, double deltaTime);

    static Eigen::Vector3d RuckigCalculation(Eigen::Vector3d current, Eigen::Vector3d target, Eigen::Vector3d &currentVelocity, Eigen::Vector3d &currentAcceleration, double maxVel, double maxAccel, double maxJerk, double deltaTime);
    static vector<double> RuckigCalculation_Jnt(vector<double> current, vector<double> target, vector<double> &currentVelocity, vector<double> &currentAcceleration, double maxVel, double maxAccel, double maxJerk, double deltaTime);

    static vector<vector<vector<double>>> trajOTG(Eigen::Vector3d current, Eigen::Vector3d target, Eigen::Vector3d currentVelocity, Eigen::Vector3d currentAcceleration, double maxVel, double maxAccel, double maxJerk, double alpha, double deltaTime);
    static TrigonometricOTG* trajOTG_ptr(Eigen::Vector3d current, Eigen::Vector3d target, Eigen::Vector3d currentVelocity, Eigen::Vector3d currentAcceleration, double maxVel, double maxAccel, double maxJerk, double alpha, double deltaTime);

    static Eigen::Vector3d OTGCalculation(Eigen::Vector3d current_pos, Eigen::Vector3d target_pos, Eigen::Vector3d& last_target_pos, vector<vector<vector<double>>>& profile_pos,int& idx, Eigen::Vector3d& current_linear_velocity, Eigen::Vector3d& current_linear_acceleration, double maxVel, double maxAccel, double maxJerk, double publish_period);
    static Eigen::Vector3d OTGCalculationS(Eigen::Vector3d current, Eigen::Vector3d target, Eigen::Vector3d& last_target,TrigonometricOTG*& otg_ptr, Eigen::Vector3d& currentVelocity, Eigen::Vector3d& currentAcceleration, double maxVel, double maxAccel, double maxJerk, double alpha, double publish_period);


    static vector<vector<vector<double>>> trajOTG_Jnt(vector<double> current, vector<double> target, vector<double> currentVelocity, vector<double> currentAcceleration, double maxVel, double maxAccel, double maxJerk, double alpha, double deltaTime);
    static TrigonometricOTG* trajOTG_Jnt_ptr(vector<double> current, vector<double> target, vector<double> currentVelocity, vector<double> currentAcceleration, double maxVel, double maxAccel, double maxJerk, double alpha, double deltaTime);

    static vector<double> OTGCalculation_Jnt(vector<double> current_pos, vector<double> target_pos, vector<double>& last_target_pos, vector<vector<vector<double>>>& profile_pos,int& idx, vector<double>& current_linear_velocity, vector<double>& current_linear_acceleration, double maxVel, double maxAccel, double maxJerk, double publish_period);
    static vector<double> OTGCalculation_JntS(vector<double> current_pos, vector<double> target_pos, vector<double>& last_target_pos, TrigonometricOTG*& otg_ptr, vector<double>& current_linear_velocity, vector<double>& current_linear_acceleration, double maxVel, double maxAccel, double maxJerk, double alpha, double publish_period);

    static double norm_between_vectors(vector<double> v1, vector<double> v2);
    static double get_angle_betweem_vectors(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d n);

};
