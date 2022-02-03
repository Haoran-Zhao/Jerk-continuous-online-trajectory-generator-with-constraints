#include "UR3PathComputation.h"

double get_angle_betweem_two_vectors(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d n)
{
    a = a.normalized();
    b = b.normalized();
    double angle = acos(a.dot(b));
    Eigen::Vector3d cross = a.cross(b);
    if (n.dot(cross) < 0)
    {
        angle = -1 * angle;
    }
    return angle;
}

double radian_to_degree(double radian)
{
    return radian * 180 / M_PI;
}

double degree_to_radian(double degree)
{
    return degree * M_PI / 180;
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Constructor
*/
UR3PathComputation::UR3PathComputation()
{
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Destructor
*/
UR3PathComputation::~UR3PathComputation()
{
}

/*
	IN 			: Ros node handle, gazebo state object pointer, marker enabled flag
	OUT 		: None
	DESCRIPTION	: Initializes the node handle, gazebo state and some variables
*/
void UR3PathComputation::initialize(ros::NodeHandle *nodePtr, GazeboModelState *model_state, Eigen::Matrix4d current_end_effector_matrix)
{
    _node_ptr = nodePtr;
    _current_end_effector_matrix = current_end_effector_matrix;
    _model_state = model_state;
    _path_frame_created = false;
    _debug = false;
}

Eigen::Matrix4d UR3PathComputation::_rotate_end_effector_matrix(Eigen::Matrix4d end_effector_matrix)
{
    // Covert to pose
    geometry_msgs::Pose end_pose = Utilities::matrix_to_pose(end_effector_matrix);

    //Get Quaternion
    tf2::Quaternion end_quat(end_pose.orientation.x, end_pose.orientation.y, end_pose.orientation.z, end_pose.orientation.w);

    double current_rot_X, current_rot_Y, current_rot_Z;
    tf2::Matrix3x3(end_quat).getRPY(current_rot_X, current_rot_Y, current_rot_Z);

    end_quat.setEulerZYX(current_rot_Z - 1.5708, current_rot_Y, current_rot_X);

    //Convert Quaternion to orientation
    end_pose.orientation.x = end_quat.x();
    end_pose.orientation.y = end_quat.y();
    end_pose.orientation.z = end_quat.z();
    end_pose.orientation.w = end_quat.w();

    return Utilities::pose_to_matrix(end_pose);
}
