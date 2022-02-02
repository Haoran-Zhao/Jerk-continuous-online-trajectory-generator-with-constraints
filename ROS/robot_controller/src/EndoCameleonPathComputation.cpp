#include "EndoCameleonPathComputation.h"

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
EndoCameleonPathComputation::EndoCameleonPathComputation()
{
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Destructor
*/
EndoCameleonPathComputation::~EndoCameleonPathComputation()
{
}

/*
	IN 			: Ros node handle, gazebo state object pointer, marker enabled flag
	OUT 		: None
	DESCRIPTION	: Initializes the node handle, gazebo state and some variables
*/
void EndoCameleonPathComputation::initialize(ros::NodeHandle *nodePtr, GazeboModelState *model_state, Eigen::Matrix4d incision_matrix, Eigen::Matrix4d current_end_effector_matrix)
{
    _node_ptr = nodePtr;
    _current_end_effector_matrix = current_end_effector_matrix;
    _model_state = model_state;
    _incision_matrix = incision_matrix;
    _path_frame_created = false;
    _articulation_angle = 0.0; //0.523599
    _current_scope_tip_matrix = _get_scope_tip_matrix(current_end_effector_matrix);
    _debug = false;

    _scope_tip_publisher = _node_ptr->advertise<robot_controller::ScopeTip>("/scope_tip", 1);
}

Eigen::Matrix4d EndoCameleonPathComputation::_get_scope_tip_matrix(Eigen::Matrix4d end_effector_matrix)
{
    //Create the transformation matrix
    Eigen::Matrix4d transformation_to_scope_tip;
    transformation_to_scope_tip.block<4, 1>(0, 0) = Eigen::Vector4d(1, 0, 0, 0);
    transformation_to_scope_tip.block<4, 1>(0, 1) = Eigen::Vector4d(0, 1, 0, 0);
    transformation_to_scope_tip.block<4, 1>(0, 2) = Eigen::Vector4d(0, 0, 1, 0);
    transformation_to_scope_tip.block<4, 1>(0, 3) = Eigen::Vector4d(0, 0, 0.37, 1);

    return end_effector_matrix * transformation_to_scope_tip;
}

Eigen::Matrix4d EndoCameleonPathComputation::_rotate_end_effector_matrix(Eigen::Matrix4d end_effector_matrix)
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

/*
	IN 			: Reference to angulated angle, target matrix
	OUT 		: returns list of pose for the end effector to follow
	DESCRIPTION	: Computes the end effector way points
*/
std::vector<geometry_msgs::Pose> EndoCameleonPathComputation::compute_end_effector_waypoints(double &angulated_angle, double &rotation_angle, Eigen::Matrix4d target_mat)
{
    //Get the incision position (4th column of incision matrix)
    Eigen::Vector3d incision_vector = _incision_matrix.block<3, 1>(0, 3);

    robot_controller::ScopeTip message;
    message.scope_tip_target = Utilities::matrix_to_pose(target_mat);
    message.rcm = Utilities::matrix_to_pose(_incision_matrix);
    message.scope_length = std::to_string(0.37);
    _scope_tip_publisher.publish(message);

    std::vector<geometry_msgs::Pose> way;

    return way;
}