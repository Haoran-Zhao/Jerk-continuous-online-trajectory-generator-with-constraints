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
    _path_replacement_enable = false;
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

        //Create ros publisher to publish image viewer rotation angle to the topic 'rotation angle', will be used by image viewer
        _angle_publisher = _node_handle->advertise<robot_controller::RotationAngle>("/rotation_angle", 1);

        _angulation_publisher = _node_handle->advertise<std_msgs::Float64>("/camera_controller/command", 1);

        //Initialize the various gazebo models
        _initialize_gazebo_models();

        //Initialize the compute path object
        _compute_path->initialize(_node_handle, &_model_state, Utilities::pose_to_matrix(_model_state.get_model_position_world("RCMSphere")), Utilities::pose_to_matrix(_move_group_ptr->getCurrentPose().pose));

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

    //Set the RCM sphere pose
    Eigen::Matrix4d rcm_matrix = Utilities::pose_to_matrix(current_robot_pose);
    rcm_matrix.block<3, 1>(0, 3) = rcm_matrix.block<3, 1>(0, 3) + ((_scope_length/2 + 0.045) * rcm_matrix.block<3, 1>(0, 2));
    _model_state.set_model_position_world("RCMSphere", Utilities::matrix_to_pose(rcm_matrix));
    incision_matrix = rcm_matrix;

    //Set the ScopeCylinder sphere
    Eigen::Matrix4d cylinder_matrix = Utilities::pose_to_matrix(current_robot_pose);
    cylinder_matrix.block<3, 1>(0, 3) = cylinder_matrix.block<3, 1>(0, 3) + ((_scope_length) * cylinder_matrix.block<3, 1>(0, 2));
    geometry_msgs::Pose cylinder_pose = Utilities::matrix_to_pose(cylinder_matrix);

    tf2::Quaternion cylinder_quat(cylinder_pose.orientation.x, cylinder_pose.orientation.y, cylinder_pose.orientation.z, cylinder_pose.orientation.w);
    double current_rot_X, current_rot_Y, current_rot_Z;
    tf2::Matrix3x3(cylinder_quat).getRPY(current_rot_X, current_rot_Y, current_rot_Z);
    cylinder_quat.setEulerZYX(current_rot_Z, current_rot_Y, current_rot_X - _cylinder_angle);
    cylinder_pose.orientation.x = cylinder_quat.x();
    cylinder_pose.orientation.y = cylinder_quat.y();
    cylinder_pose.orientation.z = cylinder_quat.z();
    cylinder_pose.orientation.w = cylinder_quat.w();

    _model_state.set_model_position_world("ScopeCylinder", cylinder_pose);
    _model_state.set_model_position_world("ScopeCylinder2", cylinder_pose);

    //Set the ScopeTipSphere sphere
    _model_state.set_model_position_world("ScopeTipSphere", cylinder_pose);
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

Eigen::Matrix4d getScopeTip(Eigen::Matrix4d end_effector_matrix, double angulation_radian)
{
  //Create the transformation matrix
  Eigen::Matrix4d transformation_to_scope_tip;
  transformation_to_scope_tip.setIdentity();
  transformation_to_scope_tip(2, 3) = 0.37;
  end_effector_matrix.block<3, 1>(0, 3) = end_effector_matrix.block<3, 1>(0, 3) + 0.37 * end_effector_matrix.block<3, 1>(0, 2);

  //Rotate end tip
  geometry_msgs::Pose end_pose = Utilities::matrix_to_pose(end_effector_matrix);
  tf2::Quaternion tip_quat(end_pose.orientation.x, end_pose.orientation.y, end_pose.orientation.z, end_pose.orientation.w);
  double current_rot_X, current_rot_Y, current_rot_Z;
  tf2::Matrix3x3(tip_quat).getRPY(current_rot_X, current_rot_Y, current_rot_Z);
  tip_quat.setEulerZYX(current_rot_Z, current_rot_Y, current_rot_X - angulation_radian);
  end_pose.orientation.x = tip_quat.x();
  end_pose.orientation.y = tip_quat.y();
  end_pose.orientation.z = tip_quat.z();
  end_pose.orientation.w = tip_quat.w();

  end_effector_matrix = Utilities::pose_to_matrix(end_pose);

  return end_effector_matrix;
}

void RobotController::_path_computation_thread_func(){
  double rotation_angle = 0.0, angulation_angle = 0.523599;
  double max_linear_velocity = 0.03, max_angular_velocity = 0.5, smoothTime = 1, max_linear_acceleration = 0.05,max_linear_jerk = 10;
  double current_software_velocity = 0.0, current_angulation_velocity = 0.0;
  double eulerXVelocity = 0.0, eulerYVelocity = 0.0, eulerZVelocity = 0.0;
  double publish_period = 0.008;
  ros::Rate cmd_rate(1 / publish_period);
  geometry_msgs::TwistStamped twist;
  twist.header.frame_id = "world";

  geometry_msgs::Pose current_robot_pose = _move_group_ptr->getCurrentPose().pose;
  Eigen::Matrix4d current_end_effector_matrix = Utilities::pose_to_matrix(current_robot_pose);
  Eigen::Matrix4d current_scope_tip_matrix = getScopeTip(current_end_effector_matrix, angulation_angle);
  _target_matrix = Utilities::pose_to_matrix(_model_state.get_model_position_world("ScopeCylinder"));

  Eigen::Vector3d Xe = current_end_effector_matrix.block<3, 1>(0, 3);
  Eigen::Vector3d Xa = current_scope_tip_matrix.block<3, 1>(0, 3);
  Eigen::Vector3d Xb, Xc = incision_matrix.block<3, 1>(0, 3);
  Eigen::Vector3d current_linear_velocity,current_linear_acceleration, current_angular_velocity;

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  while (ros::ok())
  {
    path_thread.lock();
    Eigen::Matrix4d target_scope_tip_matrix = _target_matrix;
    path_thread.unlock();

    //Get the current end effector matrix
    current_end_effector_matrix = Utilities::pose_to_matrix(_move_group_ptr->getCurrentPose().pose);

    //Get the scope tip matrix
    current_scope_tip_matrix = getScopeTip(current_end_effector_matrix, angulation_angle);

    //Set Xa as the current scope tip matrix
    Xa = current_scope_tip_matrix.block<3, 1>(0, 3);

    //Set Xb as the target position and Xc as the incision matrix
    Xb = target_scope_tip_matrix.block<3, 1>(0, 3);

    //Calculate target matrix
    Eigen::Matrix4d target_matrix;
    target_matrix.setIdentity();
    target_matrix.block<3,1>(0,0) = (target_scope_tip_matrix.block<3, 1>(0, 2).cross((Xb - Xc).normalized())).normalized();
    target_matrix.block<3,1>(0,2) = target_scope_tip_matrix.block<3, 1>(0, 2).normalized();
    target_matrix.block<3,1>(0,1) = (target_matrix.block<3,1>(0,2).cross(target_matrix.block<3,1>(0,0))).normalized();
    target_matrix.block<3,1>(0,3) = target_scope_tip_matrix.block<3, 1>(0, 3);

    //Get the damp position
    // Eigen::Vector3d damp_position = Utilities::SmoothDampVector(Xa, Xb, current_linear_velocity, smoothTime, max_linear_velocity, publish_period);
    Eigen::Vector3d damp_position = Utilities::RuckigCalculation(Xa, Xb, current_linear_velocity, current_linear_acceleration, max_linear_velocity, max_linear_acceleration, max_linear_jerk, publish_period);

    //Calculate orientation
    geometry_msgs::Pose current_scope_tip_pose = Utilities::matrix_to_pose(current_scope_tip_matrix);
    tf2::Quaternion current_scope_tip_quat(current_scope_tip_pose.orientation.x, current_scope_tip_pose.orientation.y, current_scope_tip_pose.orientation.z, current_scope_tip_pose.orientation.w);
    double current_x, current_y, current_z;
    tf2::Matrix3x3(current_scope_tip_quat).getRPY(current_x, current_y, current_z);

    geometry_msgs::Pose target_scope_tip_pose = Utilities::matrix_to_pose(target_matrix);
    tf2::Quaternion target_scope_tip_quat(target_scope_tip_pose.orientation.x, target_scope_tip_pose.orientation.y, target_scope_tip_pose.orientation.z, target_scope_tip_pose.orientation.w);
    double target_x, target_y, target_z;
    tf2::Matrix3x3(target_scope_tip_quat).getRPY(target_x, target_y, target_z);

    //Calculate Angular velocities and positions
    Eigen::Vector3d current_euler(current_x, current_y, current_z);
    Eigen::Vector3d target_euler(target_x, target_y, target_z);
    Eigen::Vector3d damp_euler = Utilities::SmoothDampVector(current_euler, target_euler, current_angular_velocity, smoothTime, max_angular_velocity, publish_period);

    double x_cur = damp_euler.x();
    double y_cur = damp_euler.y();
    double z_cur = damp_euler.z();
    eulerXVelocity = current_angular_velocity.x();
    eulerYVelocity = current_angular_velocity.y();
    eulerZVelocity = current_angular_velocity.z();

    //Create target scope tip pose
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

    //Calculate Linear Velocity
    Eigen::Vector3d Xp = damp_position;
    Eigen::Vector3d f = Xp - Xc;
    Eigen::Vector3d f_der = current_linear_velocity;  //this is not the velocity along z-axis?
    Eigen::Vector3d end_effector_linear_velocity = current_linear_velocity - 0.37 *
                        (f.norm()*f.norm()*f_der - (f.dot(f_der))*f)/(f.norm()*f.norm()*f.norm()); // what is 0.37? why?

    double Zx_cylinder = cos(z_cur)*sin(y_cur)*cos(x_cur) + sin(z_cur)*sin(x_cur);
    double der_Zx_cylinder = -cos(z_cur)*sin(y_cur)*sin(x_cur)*eulerXVelocity
                             +cos(z_cur)*cos(x_cur)*cos(y_cur)*eulerYVelocity
                             -sin(y_cur)*cos(x_cur)*sin(z_cur)*eulerZVelocity
                             +sin(z_cur)*cos(x_cur)*eulerXVelocity + sin(x_cur)*cos(z_cur)*eulerZVelocity;

    double Zy_cylinder = sin(z_cur)*sin(y_cur)*cos(x_cur) - cos(z_cur)*sin(x_cur);
    double der_Zy_cylinder = -sin(z_cur)*sin(y_cur)*sin(x_cur)*eulerXVelocity
                             +sin(z_cur)*cos(x_cur)*cos(y_cur)*eulerYVelocity
                             +sin(y_cur)*cos(x_cur)*cos(z_cur)*eulerZVelocity
                             -cos(z_cur)*cos(x_cur)*eulerXVelocity + sin(x_cur)*sin(z_cur)*eulerZVelocity;

    double Zz_cylinder = cos(y_cur)*cos(x_cur);
    double der_Zz_cylinder = -cos(y_cur)*sin(x_cur)*eulerXVelocity - cos(x_cur)*sin(y_cur)*eulerYVelocity;

    Eigen::Vector3d Zcylinder = Eigen::Vector3d(Zx_cylinder, Zy_cylinder, Zz_cylinder);
    Eigen::Vector3d der_Zcylinder = Eigen::Vector3d(der_Zx_cylinder, der_Zy_cylinder, der_Zz_cylinder);

    Eigen::Vector3d Zend_effector = (damp_frame.block<3, 1>(0, 3) - Xc).normalized();
    Eigen::Vector3d Xend_effector = (Zcylinder.cross(Zend_effector).normalized()).normalized();
    Eigen::Vector3d Yend_effector = (Zend_effector.cross(Xend_effector)).normalized();
    Eigen::Matrix4d end_effector_frame;
    end_effector_frame.setIdentity();
    end_effector_frame.block<3,1>(0,0) = Xend_effector;
    end_effector_frame.block<3,1>(0,1) = Yend_effector;
    end_effector_frame.block<3,1>(0,2) = Zend_effector;
    end_effector_frame.block<3,1>(0,3) = damp_position - 0.37 * (damp_position - Xc).normalized();

    //Derivative of Z axis
    Eigen::Vector3d zAxis_der = (f.norm()*f.norm()*f_der - (f.dot(f_der))*f)/(f.norm()*f.norm()*f.norm());

    //Derivative of X axis
    Eigen::Vector3d g = Zcylinder.cross(Zend_effector);
    Eigen::Vector3d g_der = Zcylinder.cross(zAxis_der) + der_Zcylinder.cross(Zend_effector);
    Eigen::Vector3d xAxis_der = (g.norm()*g.norm()*g_der - (g.dot(g_der))*g)/(g.norm()*g.norm()*g.norm());

    //Derivative of Y Axis
    Eigen::Vector3d yAxis_der = Zend_effector.cross(xAxis_der) + zAxis_der.cross(Xend_effector);

    Eigen::Matrix3d rot_matrix;
    rot_matrix.setIdentity();
    rot_matrix.block<3, 1>(0, 0) = Xend_effector;
    rot_matrix.block<3, 1>(0, 1) = Yend_effector;
    rot_matrix.block<3, 1>(0, 2) = Zend_effector;

    Eigen::Matrix3d der_matrix;
    der_matrix.setIdentity();
    der_matrix.block<3, 1>(0, 0) = xAxis_der;
    der_matrix.block<3, 1>(0, 1) = yAxis_der;
    der_matrix.block<3, 1>(0, 2) = zAxis_der;

    Eigen::Matrix3d skew = der_matrix * rot_matrix.transpose();

    double angular_x = skew(2, 1);
    double angular_y = skew(0, 2);
    double angular_z = skew(1, 0);
    Eigen::Vector3d end_effector_angular_velocity(angular_x, angular_y, angular_z);

    //Velocity that needs to be added to counter rcm drift
    Xe = current_end_effector_matrix.block<3, 1>(0, 3);
    Eigen::Vector3d Xc_ = Xe +
                        ((Xc - Xe).dot(current_end_effector_matrix.block<3, 1>(0, 2)))*current_end_effector_matrix.block<3, 1>(0, 2);
    Eigen::Vector3d correction_velocity = ((Xc - Xc_) / publish_period) * max_linear_velocity;

    //Send velocity commands to moveit servo
    end_effector_linear_velocity = end_effector_linear_velocity + correction_velocity;
    twist.header.stamp = ros::Time::now();
    twist.twist.linear.x = end_effector_linear_velocity.x();
    twist.twist.linear.y = end_effector_linear_velocity.y();
    twist.twist.linear.z = end_effector_linear_velocity.z();
    twist.twist.angular.x = end_effector_angular_velocity.x();
    twist.twist.angular.y = end_effector_angular_velocity.y();
    twist.twist.angular.z = end_effector_angular_velocity.z();
    _twist_stamped_pub.publish(twist);

    //Calculate damped angulation angle
    double target_angulation = Utilities::get_angle_betweem_vectors(target_scope_tip_matrix.block<3,1>(0,2), end_effector_frame.block<3,1>(0,2),
                end_effector_frame.block<3,1>(0,0));
    angulation_angle = Utilities::SmoothDampAngle(angulation_angle, target_angulation, current_angulation_velocity, smoothTime,
                100, publish_period);
    if(std::isnan(angulation_angle)){
        angulation_angle = 0.0;
    }
    std_msgs::Float64 value;
    value.data = angulation_angle - 0.525399;
    _angulation_publisher.publish(value);

    //Calculate damped software rotation
    double target_angle = Utilities::get_angle_betweem_vectors(end_effector_frame.block<3,1>(0,0), target_scope_tip_matrix.block<3,1>(0,0),
                target_scope_tip_matrix.block<3,1>(0,2))* (180/M_PI);
    rotation_angle = Utilities::SmoothDampAngle(rotation_angle, target_angle, current_software_velocity, smoothTime,
                1, publish_period);
    if(std::isnan(rotation_angle)){
        rotation_angle = 0.0;
    }
    robot_controller::RotationAngle rotation_messsage;
    rotation_messsage.angle = std::to_string(rotation_angle);
    rotation_messsage.time = 0.0;
    _angle_publisher.publish(rotation_messsage);
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
    geometry_msgs::Pose current_pose = _model_state.get_model_position("ScopeCylinder");

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
    _model_state.set_model_position("ScopeCylinder", current_pose);

    //Set the target matrix
    geometry_msgs::Pose target_pose = _model_state.get_model_position_world("ScopeCylinder");
    path_thread.lock();
    _target_matrix = Utilities::pose_to_matrix(target_pose);
    path_thread.unlock();

    //Notify the semaphore of new cylinder position
    //_new_position_available_sem->notify(std::this_thread::get_id());
}
