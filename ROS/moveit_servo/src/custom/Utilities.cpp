#include "Utilities.h"


/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Constuctor
*/
Utilities::Utilities(){

}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Destructor
*/
Utilities::~Utilities(){

}

/*
	IN 			: Pose message
	OUT 		: 4x4 frame matrix
	DESCRIPTION	: Converts a pose message to 4x4 matrix
*/
Eigen::Matrix4d Utilities::pose_to_matrix(geometry_msgs::Pose pose){
    Eigen::Vector3d trans_vector(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Matrix3d rot_mat = quat.normalized().toRotationMatrix();
    Eigen::Matrix4d result_matrix;
    result_matrix.setIdentity();
    result_matrix.block<3,3>(0,0) = rot_mat;
    result_matrix.block<3,1>(0,3) = trans_vector;
    return result_matrix;
}

/*
	IN 			: Matrix frame
	OUT 		: Pose message
	DESCRIPTION	: Converts a 4x4 matrix frame to pose message
*/
geometry_msgs::Pose Utilities::matrix_to_pose(Eigen::Matrix4d matrix_frame){
    geometry_msgs::Pose pose;
    Eigen::Matrix3d rot_mat = matrix_frame.block<3,3>(0,0);
    Eigen::Vector3d trans_vector = matrix_frame.block<3,1>(0,3);
    pose.position.x = trans_vector(0,0);
    pose.position.y = trans_vector(1,0);
    pose.position.z = trans_vector(2,0);

    Eigen::Quaterniond quat(rot_mat);
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    return pose;
}

/*
	IN 			: Rotation matrix and translation vector
	OUT 		: Pose message
	DESCRIPTION	: Combines rotation matrix and translation vector to pose message
*/
geometry_msgs::Pose Utilities::matrix_to_pose(Eigen::Matrix3d rot_mat, Eigen::Vector3d trans_vector){
    geometry_msgs::Pose pose;
    pose.position.x = trans_vector(0,0);
    pose.position.y = trans_vector(1,0);
    pose.position.z = trans_vector(2,0);

    Eigen::Quaterniond quat(rot_mat);
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    return pose;
}

/*
	IN 			: Rotation matrix and translation vector
	OUT 		: Pose message
	DESCRIPTION	: Combines rotation matrix and translation vector to pose message
*/
geometry_msgs::Pose Utilities::vector_to_pose(Eigen::Vector3d vector){
    geometry_msgs::Pose pose;
    pose.position.x = vector.x();
    pose.position.y = vector.y();
    pose.position.z = vector.z();

    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    return pose;
}

Eigen::Matrix4d Utilities::combine_rotation_position_to_matrix(Eigen::Matrix3d rotation_matrix, Eigen::Vector3d translation_vector){
    Eigen::Matrix4d mat;
    mat.setIdentity();
    mat.block<3,3>(0,0) = rotation_matrix;
    mat.block<3,1>(0,3) = translation_vector;
    return mat;
};

/*
	IN 			: source pose, destination pose
	OUT 		: None
	DESCRIPTION	: Copies pose from the source and set it to the destination pose
*/
void Utilities::copy_geometry_pose(geometry_msgs::Pose source, geometry_msgs::Pose &dest) {
    dest.position.x = source.position.x;
    dest.position.y = source.position.y;
    dest.position.z = source.position.z;

    dest.orientation.x = source.orientation.x;
    dest.orientation.y = source.orientation.y;
    dest.orientation.z = source.orientation.z;
    dest.orientation.w = source.orientation.w;
}

/*
	IN 			: Quaternion 1, Quaternion 2
	OUT 		: true if equal, false otherwise
	DESCRIPTION	: Compares the two quaternion for equality
*/
bool Utilities::compare(Eigen::Quaterniond quat1, Eigen::Quaterniond quat2)
{
    bool status = true;
    if( (quat1.x() != quat2.x()) || (quat1.y() != quat2.y()) || (quat1.z() != quat2.z()) || (quat1.w() != quat2.w()) )
        status = false;
    return status;
}

/*
	IN 			: start quaternion, end quaternion, number of interpolation points
	OUT 		: vector list of eigen quaternions
	DESCRIPTION	: Interpolates from start quaternion to the end quaternion
*/
std::vector<Eigen::Quaterniond> Utilities::interpolate_orientation(Eigen::Quaterniond start_quat, Eigen::Quaterniond end_quat, int num_of_interpolation_points){
    std::vector<Eigen::Quaterniond> orientation_list;
    if(false == compare(start_quat, end_quat) )
    {
        double p = 1.0/(num_of_interpolation_points+1);
        for(double i=0.0; i<=1.0; i+=p){
            orientation_list.push_back(start_quat.slerp(i, end_quat));
        }
    }
    else
    {
        for(int i=0; i<=num_of_interpolation_points+1;i++)
            orientation_list.push_back(end_quat);
    }
    return orientation_list;
}

/*
	IN 			: start vector, end vector, number of interpolation points
	OUT 		: vector list of eigen vectors
	DESCRIPTION	: Interpolates from start vector to the end vector
*/
std::vector<Eigen::Vector3d> Utilities::interpolate_position(Eigen::Vector3d start_vector, Eigen::Vector3d end_vector, int num_of_interpolation_points){
    std::vector<Eigen::Vector3d> vector_list;
    if( start_vector != end_vector)
    {
        double p = 1.0/(num_of_interpolation_points+1);
        for(double i=0.0; i<=1.0; i+=p){
            //Linear Interpolation
            Eigen::Vector3d pos = start_vector*(1-i) + end_vector*i;
            vector_list.push_back(pos);
        }
    }
    else
    {
        for(int i=0; i<=num_of_interpolation_points+1;i++)
            vector_list.push_back(start_vector);
    }
    return vector_list;
}

void angle_between_vector(){
    
}