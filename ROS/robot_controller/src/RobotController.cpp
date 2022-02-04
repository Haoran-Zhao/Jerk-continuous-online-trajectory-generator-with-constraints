//
// Created by hmc on 11/18/20.
//

#include <robot_controller/RotationAngle.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <std_msgs/Float64.h>
#include "RobotController.h"
#include "Utilities.h"

RobotController::RobotController() {
    _move_group_ptr = NULL;
    _node_handle = NULL;
    _async_spinner = NULL;
    _marker_enabled = false;
    _model_state_enabled = true;
}

RobotController::~RobotController() {
    if (NULL != _move_group_ptr)
        delete _move_group_ptr;
    if (NULL != _node_handle)
        delete _node_handle;
    if (NULL != _async_spinner)
        delete _async_spinner;
    if (NULL != _loop_rate_ptr)
        delete _loop_rate_ptr;
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Initializes the node handle, async spinner, publishers and thread
*/
bool RobotController::initialize(int argc, char **argv) {
    bool status = true;
    try {
        //Initialize ros node
        ros::init(argc, argv, "robot_controller");

        //Create ros Node Handle
        _node_handle = new ros::NodeHandle("~");

        //Create and start AsyncSpinner
        _async_spinner = new ros::AsyncSpinner(1);
        if (NULL != _async_spinner)
            _async_spinner->start();
        else
            throw string("_async_spinner creation Failed");

        //Set the base frame, to be used by moveit
        if (NULL != _node_handle)
            _node_handle->param<std::string>("base_frame", _base_frame, "world"); // parameter name, string object reference, default value
        else
            throw string("_node_handle creation Failed");

        //Create a reference to the move group
        _move_group_ptr = new moveit::planning_interface::MoveGroupInterface("arm");

        if (NULL != _move_group_ptr) {
            //Move the robot to initial position
            _move_to_initial_position();

            //Set the base frame
            _move_group_ptr->setPoseReferenceFrame(_base_frame);

            //Set velocity scaling factor
            _move_group_ptr->setMaxVelocityScalingFactor(0.5);
        } else
            throw string("_move_group_ptr creation failed");

        _loop_rate_ptr = new ros::Rate(1);

        //Initialize the various gazebo models
        _initialize_gazebo_models();

        //Initialize the compute path object
        _compute_path->initialize(_node_handle, &_model_state, Utilities::pose_to_matrix(_move_group_ptr->getCurrentPose().pose));

        //Semaphore used to notify availablity of new cylinder pose to compute path
        _new_position_available_sem = new Semaphore(0, 0);

        //Thread that does robot movement
        //_robot_movement_thread = std::thread(&RobotController::_robot_movement_thread_func, this);

        _path_computation_thread = std::thread(&RobotController::_path_computation_thread_func, this);

        //ROS publisher to publish trajectory commands to the robot
        _joint_pub = _node_handle->advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);

        _twist_stamped_pub = _node_handle->advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 1 , true);

        // _joint_state_sub = _node_handle->subscribe("/joint_states", 1, &RobotController::jointCallback, this);
    }
    catch (...) {
        cout << "Exception : " << endl;
        status = false;
    }

    return status;
}

void RobotController::jointCallback(const sensor_msgs::JointState& state){
    auto header = state.header.stamp;
    auto& names = state.name;
    auto& positions = state.position;
    int size = std::min(names.size(), positions.size());
    joint_stream << (header.toSec()) << ",";
    for(int i=0; i < size; ++i){
        std::cout << names[i] << std::endl;
        joint_stream << positions[i] << ",";
    }
    joint_stream.seekp(-1, std::ios_base::end);
    joint_stream << "\n";
}

void RobotController::writeJointStatesToFile(){
    std::ofstream outFile;
    outFile.open("joint_data.csv");
//outFile << joint_stream.rdbuf();
    // std::cout << joint_stream.str() << std::endl;
}



/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Moves the robot to the initial position named arc, defined in the srdf file
*/
void RobotController::_move_to_initial_position() {
    _move_group_ptr->setNamedTarget("arc");
    _move_group_ptr->move();
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Initializes the gazebo model state object, and sets the pose of EndEffectorSphere and RCMSphere in the Gazebo environment
*/
void RobotController::_initialize_gazebo_models() {
    //Get the current end effector pose
    geometry_msgs::Pose current_robot_pose = _move_group_ptr->getCurrentPose().pose;

    //Intialize gazebo model state object
    _model_state.initialize(_node_handle);

    //Set the end effector sphere to robot pose
    _model_state.set_model_position_world("EndEffectorSphere", current_robot_pose);

    //Set the ScopeCylinder sphere
    geometry_msgs::Pose cylinder_pose = current_robot_pose;

    tf2::Quaternion cylinder_quat(cylinder_pose.orientation.x, cylinder_pose.orientation.y, cylinder_pose.orientation.z, cylinder_pose.orientation.w);

    _model_state.set_model_position_world("targetCylinder", cylinder_pose);
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Thread which continuously waits for new target pose, when new target pose is received the path is computed and asks the robot to move
*/
void RobotController::_robot_movement_thread_func() {
    double loop_rate = 0.008;
    ros::Rate cmd_rate(1 / loop_rate);
    while (ros::ok()) {
        //Wait for new cylinder position to be available
        _new_position_available_sem->wait(std::this_thread::get_id());

        //Compute the path, to get list of way points
        //this->compute_path_func();
        //_compute_path->compute_end_effector_waypoints(_articulation_angle, _rotation_angle, _target_matrix);

        cmd_rate.sleep();
    }
}

void RobotController::_path_computation_thread_func(){
  double smoothTime = 1;
  double max_linear_velocity = 0.03, max_linear_acceleration = 0.05, max_linear_jerk = 10;
  double max_angular_velocity = 0.5, max_angular_acceleration = 5, max_angular_jerk = 100;
  double eulerXVelocity = 0.0, eulerYVelocity = 0.0, eulerZVelocity = 0.0;
  double publish_period = 0.008;
  ros::Rate cmd_rate(1 / publish_period);
  geometry_msgs::TwistStamped twist;
  twist.header.frame_id = "world";

  geometry_msgs::Pose current_robot_pose = _move_group_ptr->getCurrentPose().pose;
  Eigen::Matrix4d current_end_effector_matrix = Utilities::pose_to_matrix(current_robot_pose);
  _target_matrix = Utilities::pose_to_matrix(_model_state.get_model_position_world("targetCylinder"));

  Eigen::Vector3d Xt; //target postion
  Eigen::Vector3d Xe = current_end_effector_matrix.block<3, 1>(0, 3);
  Eigen::Vector3d current_linear_velocity,current_linear_acceleration, current_angular_velocity,current_angular_acceleration;

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  while (ros::ok())
  {
    path_thread.lock();
    Eigen::Matrix4d target_matrix = _target_matrix;
    path_thread.unlock();
    current_robot_pose = _move_group_ptr->getCurrentPose().pose;
    _model_state.set_model_position_world("EndEffectorSphere", current_robot_pose);
    //Get the current end effector matrix
    current_end_effector_matrix = Utilities::pose_to_matrix(current_robot_pose);
    Xe = current_end_effector_matrix.block<3,1>(0,3);
    //Set Xt as the target position
    Xt = target_matrix.block<3, 1>(0, 3);
    //Get the damp position
    // Eigen::Vector3d damp_position = Utilities::SmoothDampVector(Xa, Xt, current_linear_velocity, smoothTime, max_linear_velocity, publish_period);
    Eigen::Vector3d damp_position = Utilities::RuckigCalculation(Xe, Xt, current_linear_velocity, current_linear_acceleration, max_linear_velocity, max_linear_acceleration, max_linear_jerk, publish_period);
    Eigen::Vector3d end_effector_linear_velocity = current_linear_velocity;

    //Calculate orientation
    geometry_msgs::Pose current_ee_pose = Utilities::matrix_to_pose(current_end_effector_matrix);
    tf2::Quaternion current_ee_quat(current_ee_pose.orientation.x, current_ee_pose.orientation.y, current_ee_pose.orientation.z, current_ee_pose.orientation.w);
    double current_x, current_y, current_z;
    tf2::Matrix3x3(current_ee_quat).getRPY(current_x, current_y, current_z);

    geometry_msgs::Pose target_pose = Utilities::matrix_to_pose(target_matrix);
    tf2::Quaternion target_quat(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
    double target_x, target_y, target_z;
    tf2::Matrix3x3(target_quat).getRPY(target_x, target_y, target_z);

    Utilities::correct_joint_range(current_x,current_y,current_z, target_x,target_y,target_z);
    printf("current: %f %f %f, target: %f %f %f\n", current_x, current_y, current_z, target_x, target_y,target_z);

    //Calculate Angular velocities and positions
    Eigen::Vector3d current_euler(current_x, current_y, current_z);
    Eigen::Vector3d target_euler(target_x, target_y, target_z);
    //Eigen::Vector3d damp_euler = Utilities::SmoothDampVector(current_euler, target_euler, current_angular_velocity, smoothTime, max_angular_velocity, publish_period);
    Eigen::Vector3d damp_euler = Utilities::RuckigCalculation(current_euler, target_euler, current_angular_velocity, current_angular_acceleration, max_angular_velocity, max_angular_acceleration, max_angular_jerk, publish_period);
    //Send velocity commands to moveit servo

    double x_cur = damp_euler.x();
    double y_cur = damp_euler.y();
    double z_cur = damp_euler.z();
    eulerXVelocity = current_angular_velocity.x();
    eulerYVelocity = current_angular_velocity.y();
    eulerZVelocity = current_angular_velocity.z();

    tf2::Quaternion quat_damp;
    quat_damp.setEulerZYX(z_cur, y_cur, x_cur);
    geometry_msgs::Pose damp_pose;
    damp_pose.orientation.x = quat_damp.x();
    damp_pose.orientation.y = quat_damp.y();
    damp_pose.orientation.z = quat_damp.z();
    damp_pose.orientation.w = quat_damp.w();
    damp_pose.position.x = damp_position.x();
    damp_pose.position.y = damp_position.y();
    damp_pose.position.z = damp_position.z();
    Eigen::Matrix4d damp_frame = Utilities::pose_to_matrix(damp_pose);

    Eigen::Matrix3d cur_rot = current_end_effector_matrix.block<3,3>(0,0);
    Eigen::Matrix3d target_rot = damp_frame.block<3,3>(0,0);
    Eigen::Matrix3d a = target_rot * cur_rot.transpose();
    double theta = acos((a.trace()-1)/2);
    Eigen::Matrix3d skew = 1/(2*publish_period) * theta/sin(theta) * (a-a.transpose());
    double angular_x = skew(2, 1);
    double angular_y = skew(0, 2);
    double angular_z = skew(1, 0);
    Eigen::Vector3d end_effector_angular_velocity(angular_x, angular_y, angular_z);

    printf("angular vel :%f %f %f\n", end_effector_angular_velocity.x(),end_effector_angular_velocity.y(),end_effector_angular_velocity.z());

    twist.header.stamp = ros::Time::now();
    twist.twist.linear.x = end_effector_linear_velocity.x();
    twist.twist.linear.y = end_effector_linear_velocity.y();
    twist.twist.linear.z = end_effector_linear_velocity.z();
    twist.twist.angular.x = end_effector_angular_velocity.x();
    twist.twist.angular.y = end_effector_angular_velocity.y();
    twist.twist.angular.z = end_effector_angular_velocity.z();
    _twist_stamped_pub.publish(twist);

    cmd_rate.sleep();
  }
}

void compute_derivative(){

}

/*
	IN 			: Delta Pose which is the difference between the previous and current pose of the target
	OUT 		: None
	DESCRIPTION	: Sets the delta pose to the class variable and move the cylinder to that pose
*/
void RobotController::set_delta_pose(geometry_msgs::Pose delta_pose) {
    Utilities::copy_geometry_pose(delta_pose, _delta_pose);

    if (_model_state_enabled)
        _update_cylinder();
}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Updates the cylinder position to the target pose (target pose = current cylinder pose + delta pose)
*/
void RobotController::_update_cylinder() {
    //Gets the current position of the model
    geometry_msgs::Pose current_pose = _model_state.get_model_position("targetCylinder");

    //Update the current position
    current_pose.position.x += _delta_pose.position.x;
    current_pose.position.y += _delta_pose.position.y;
    current_pose.position.z += _delta_pose.position.z;

    double current_rot_X, current_rot_Y, current_rot_Z, delta_rot_X, delta_rot_Y, delta_rot_Z;

    //Gets the current orientation of the model
    tf2::Quaternion current_quat(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);

    //Gets the delta orientation
    tf2::Quaternion delta_quat(_delta_pose.orientation.x, _delta_pose.orientation.y, _delta_pose.orientation.z, _delta_pose.orientation.w);

    //Get Current and delta Euler angles
    tf2::Matrix3x3(current_quat).getRPY(current_rot_X, current_rot_Y, current_rot_Z);
    tf2::Matrix3x3(delta_quat).getRPY(delta_rot_X, delta_rot_Y, delta_rot_Z);

    tf2::Quaternion target_quat;

    //Adds the delta rotation angles to the current rotation
    target_quat.setEulerZYX(current_rot_Z + delta_rot_Z, current_rot_Y + delta_rot_Y, current_rot_X + delta_rot_X);

    //Sets the orientation of the model
    current_pose.orientation.x = target_quat.x();
    current_pose.orientation.y = target_quat.y();
    current_pose.orientation.z = target_quat.z();
    current_pose.orientation.w = target_quat.w();

    //Update the model position in gazebo
    _model_state.set_model_position("targetCylinder", current_pose);

    //Set the target matrix
    geometry_msgs::Pose target_pose = _model_state.get_model_position_world("targetCylinder");
    path_thread.lock();
    _target_matrix = Utilities::pose_to_matrix(target_pose);
    path_thread.unlock();

    //Notify the semaphore of new cylinder position
    //_new_position_available_sem->notify(std::this_thread::get_id());
}
