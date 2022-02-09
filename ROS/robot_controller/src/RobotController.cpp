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
    _Cartesian_compute=false;
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

        //ROS publisher to publish trajectory commands to the robot
        _joint_pub = _node_handle->advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);
        //Thread that does robot movement
        if (_Cartesian_compute)
        {
          //_robot_movement_thread = std::thread(&RobotController::_robot_movement_thread_func, this);
          _path_computation_thread = std::thread(&RobotController::_path_computation_thread_func, this);
          _twist_stamped_pub = _node_handle->advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 1 , true);
        }
        else{
          //Initialize kdl solver
          _kdl_initialize();
          _twist_stamped_joint_pub = _node_handle->advertise<control_msgs::JointJog>("/servo_server/delta_joint_cmds", 1 , true);
          //_joint_state_sub = _node_handle->subscribe("/joint_states", 1, &RobotController::_Joint_state_cb, this);
          _joint_computation_thread = std::thread(&RobotController::_joint_computation_thread_func, this);
        }
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


void RobotController::_kdl_initialize()
{
  KDL::Tree my_tree;
  string robot_desc_string;
  _node_handle->param("/robot_description", robot_desc_string, string());
  if(!kdl_parser::treeFromString(robot_desc_string, my_tree))
  {
    ROS_ERROR("Fail to Construct kdl tree");
  }
  else
  {
    ROS_INFO("Succesfully Construct kdl tree");
  }
  // std::cout << robot_desc_string << '\n';
  my_tree.getChain("base_link", "tool0", _my_chain);

  unsigned int numJoint = _my_chain.getNrOfJoints();
  _fk_solver = new KDL::ChainFkSolverPos_recursive(_my_chain);
  _ik_solver_pinv = new KDL::ChainIkSolverVel_pinv(_my_chain, 0.0001, 1000);
  _ik_solver = new KDL::ChainIkSolverPos_NR(_my_chain, *_fk_solver, *_ik_solver_pinv, 1000); //max 100 iterations and stop by an accuracy of 1e-6;

  if (_my_chain.getNrOfJoints() == 0)
  {
      ROS_INFO("Failed to initialize kinematic chain");
  }
  else
  {
      ROS_INFO("Num of joints in the chain: %u", _my_chain.getNrOfJoints());
  }
  _joint_state = KDL::JntArrayVel(numJoint);
  _joint_name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
}

void RobotController::_Joint_state_cb(const sensor_msgs::JointStateConstPtr& msg)
{
  _joint_state.q(0) = msg->position[2];
  _joint_state.q(1) = msg->position[1];
  _joint_state.q(2) = msg->position[0];
  _joint_state.q(3) = msg->position[3];
  _joint_state.q(4) = msg->position[4];
  _joint_state.q(5) = msg->position[5];

  _joint_state.qdot(0) = msg->velocity[2];
  _joint_state.qdot(1) = msg->velocity[1];
  _joint_state.qdot(2) = msg->velocity[0];
  _joint_state.qdot(3) = msg->velocity[3];
  _joint_state.qdot(4) = msg->velocity[4];
  _joint_state.qdot(5) = msg->velocity[5];
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
  Eigen::Vector3d last_target_pos;
  _target_matrix = Utilities::pose_to_matrix(_model_state.get_model_position_world("targetCylinder"));

  Eigen::Vector3d target_pos; //target postion
  Eigen::Vector3d current_pos = current_end_effector_matrix.block<3, 1>(0, 3);
  last_target_pos = current_pos;

  geometry_msgs::Pose current_ee_pose = Utilities::matrix_to_pose(current_end_effector_matrix);
  tf2::Quaternion current_quat(current_ee_pose.orientation.x, current_ee_pose.orientation.y, current_ee_pose.orientation.z, current_ee_pose.orientation.w);
  double current_x, current_y, current_z;
  tf2::Matrix3x3(current_quat).getRPY(current_x, current_y, current_z);
  Eigen::Vector3d last_target_euler(current_x, current_y, current_z);

  Eigen::Vector3d current_linear_velocity,current_linear_acceleration, current_angular_velocity,current_angular_acceleration;
  int idx_pos = 0, idx_euler=0;
  vector<vector<vector<double>>> profile_pos, profile_euler;
  Eigen::Vector3d damp_position, damp_euler;
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
    current_pos = current_end_effector_matrix.block<3,1>(0,3);
    //Set target_pos as the target position
    target_pos = target_matrix.block<3, 1>(0, 3);

    //Get the damp position
    //damp_position = Utilities::SmoothDampVector(Xa, target_pos, current_linear_velocity, smoothTime, max_linear_velocity, publish_period);
    //damp_position = Utilities::RuckigCalculation(current_pos, target_pos, current_linear_velocity, current_linear_acceleration, max_linear_velocity, max_linear_acceleration, max_linear_jerk, publish_period);
    damp_position = Utilities::OTGCalculation(current_pos, target_pos, last_target_pos, profile_pos, idx_pos, current_linear_velocity, current_linear_acceleration, max_linear_velocity, max_linear_acceleration,max_linear_jerk, publish_period);

    Eigen::Vector3d end_effector_linear_velocity = current_linear_velocity;

    //Calculate orientation
    current_ee_pose = Utilities::matrix_to_pose(current_end_effector_matrix);
    tf2::Quaternion current_ee_quat(current_ee_pose.orientation.x, current_ee_pose.orientation.y, current_ee_pose.orientation.z, current_ee_pose.orientation.w);
    tf2::Matrix3x3(current_ee_quat).getRPY(current_x, current_y, current_z);

    geometry_msgs::Pose target_pose = Utilities::matrix_to_pose(target_matrix);
    tf2::Quaternion target_quat(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
    double target_x, target_y, target_z;
    tf2::Matrix3x3(target_quat).getRPY(target_x, target_y, target_z);

    Utilities::correct_joint_range(current_x,current_y,current_z, target_x,target_y,target_z);
    //printf("current: %f %f %f, target: %f %f %f\n", current_x, current_y, current_z, target_x, target_y,target_z);

    //Calculate Angular velocities and positions
    Eigen::Vector3d current_euler(current_x, current_y, current_z);
    Eigen::Vector3d target_euler(target_x, target_y, target_z);
    //damp_euler = Utilities::SmoothDampVector(current_euler, target_euler, current_angular_velocity, smoothTime, max_angular_velocity, publish_period);
    damp_euler = Utilities::RuckigCalculation(current_euler, target_euler, current_angular_velocity, current_angular_acceleration, max_angular_velocity, max_angular_acceleration, max_angular_jerk, publish_period);
    //damp_euler = Utilities::OTGCalculation(current_euler, target_euler, last_target_euler, profile_euler, idx_euler, current_angular_velocity, current_angular_acceleration, max_angular_velocity, max_angular_acceleration, max_angular_jerk, publish_period);
/*
    if((last_target_euler-target_euler).norm()<=0.000001)
    {
      if((current_euler-target_euler).norm()<=0.000001 || idx_euler>=profile_euler[0].size()||profile_euler.empty())
      {
        printf("target: %f %f %f, current: %f %f %f\n", target_euler(0), target_euler(1),target_euler(2), current_euler(0), current_euler(1),current_euler(2));
        current_angular_velocity = {0.0,0.0,0.0};
        current_angular_acceleration = {0.0,0.0,0.0};
        damp_euler = current_euler;
        last_target_euler = target_euler;
        idx_euler = 0;
        profile_euler.clear();
        printf("....");
      }
      else
      {
        printf("moveing...%d %d %d %d\n", profile_euler[0].size(), profile_euler[1].size(), profile_euler[2].size(), idx_euler);
        printf("current: %f %f %f, expect: %f %f %f\n", current_euler(0), current_euler(1),current_euler(2), profile_euler[0][idx_euler][4], profile_euler[1][idx_euler][4],profile_euler[2][idx_euler][4]);
        vector<double> profile_x = profile_euler[0][idx_euler];
        vector<double> profile_y = profile_euler[1][idx_euler];
        vector<double> profile_z = profile_euler[2][idx_euler];
        printf("vel: %f %f %f\n", profile_x[3], profile_y[3], profile_z[3]);
        current_angular_velocity = {profile_x[3], profile_y[3], profile_z[3]};
        current_angular_acceleration = {profile_x[2], profile_y[2], profile_z[2]};
        if (idx_euler+1 < profile_euler[0].size())
        {
          damp_euler = {profile_euler[0][idx_euler+1][4],profile_euler[1][idx_euler+1][4],profile_euler[2][idx_euler+1][4]};
        }
        else{
          damp_euler = {profile_x[4],profile_y[4],profile_z[4]};
        }
        idx_euler+=1;
      }
    }
    else
    {
      idx_euler = 0;
      profile_euler.clear();
      last_target_euler = target_euler;
      profile_euler =Utilities::trajOTG(current_euler, target_euler, current_angular_velocity, current_angular_acceleration, max_angular_velocity,max_angular_acceleration, max_angular_jerk, 0.01, publish_period);
      vector<double> profile_x = profile_euler[0][idx_euler];
      vector<double> profile_y = profile_euler[1][idx_euler];
      vector<double> profile_z = profile_euler[2][idx_euler];
      printf("vel: %f %f %f\n", profile_x[3], profile_y[3], profile_z[3]);
      current_angular_velocity = {profile_x[3], profile_y[3], profile_z[3]};
      current_angular_acceleration = {profile_x[2],profile_y[2],profile_z[2]};
      damp_euler = {profile_euler[0][idx_euler+1][4],profile_euler[1][idx_euler+1][4],profile_euler[2][idx_euler+1][4]};
      printf("start...\n");
      printf("current: %f %f %f, expect: %f %f %f\n", current_euler(0), current_euler(1),current_euler(2), profile_euler[0][idx_euler][4], profile_euler[1][idx_euler][4],profile_euler[2][idx_euler][4]);
      idx_euler+=1;
    }
*/
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
//https://math.stackexchange.com/questions/668866/how-do-you-find-angular-velocity-given-a-pair-of-3x3-rotation-matrices
    Eigen::Matrix3d cur_rot = current_end_effector_matrix.block<3,3>(0,0);
    Eigen::Matrix3d target_rot = damp_frame.block<3,3>(0,0);
    Eigen::Matrix3d a = target_rot * cur_rot.transpose();
    double theta = acos((a.trace()-1)/2);
    Eigen::Matrix3d skew = 1/(2*publish_period) * theta/sin(theta) * (a-a.transpose());
    double angular_x = skew(2, 1);
    double angular_y = skew(0, 2);
    double angular_z = skew(1, 0);
    Eigen::Vector3d end_effector_angular_velocity(angular_x, angular_y, angular_z);

    //printf("angular vel :%f %f %f\n", end_effector_angular_velocity(0),end_effector_angular_velocity(1),end_effector_angular_velocity(2));

    twist.header.stamp = ros::Time::now();
    twist.twist.linear.x = end_effector_linear_velocity.x();
    twist.twist.linear.y = end_effector_linear_velocity.y();
    twist.twist.linear.z = end_effector_linear_velocity.z();
    twist.twist.angular.x = isnan(end_effector_angular_velocity.x())? 0 : end_effector_angular_velocity.x();
    twist.twist.angular.y = isnan(end_effector_angular_velocity.y())? 0 : end_effector_angular_velocity.y();
    twist.twist.angular.z = isnan(end_effector_angular_velocity.z())? 0 : end_effector_angular_velocity.z();
    //printf("linear vel: %f %f %f\n", twist.twist.linear.x,twist.twist.linear.y,twist.twist.linear.z);
    //printf("angular vel: %f %f %f\n", twist.twist.angular.x,twist.twist.angular.y,twist.twist.angular.z);

    _twist_stamped_pub.publish(twist);
    cmd_rate.sleep();
  }
}

void RobotController::_joint_computation_thread_func(){
  double smoothTime = 1;
  double max_angular_velocity = 0.5, max_angular_acceleration = 5, max_angular_jerk = 100;
  double publish_period = 0.008;
  ros::Rate cmd_rate(1 / publish_period);
  control_msgs::JointJog joint_deltas;
  joint_deltas.joint_names = _joint_name;
  joint_deltas.header.frame_id = "world";
  geometry_msgs::Pose current_robot_pose = _move_group_ptr->getCurrentPose().pose;
  vector<double> current_robot_joint = _move_group_ptr->getCurrentJointValues();
  Eigen::Matrix4d current_end_effector_matrix = Utilities::pose_to_matrix(current_robot_pose);
  _target_matrix = Utilities::pose_to_matrix(_model_state.get_model_position_world("targetCylinder"));
  vector<double> target_robot_joint, last_target_joint;
  target_robot_joint = current_robot_joint;
  last_target_joint = target_robot_joint;
  vector<double> current_angular_velocity(6),current_angular_acceleration(6);
  Eigen::Vector3d current_pos,target_pos; //target postion
  int idx_joint=0;
  vector<vector<vector<double>>> profile_joint;
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  while (ros::ok())
  {
    path_thread.lock();
    Eigen::Matrix4d target_matrix = _target_matrix;
    path_thread.unlock();
    current_robot_pose = _move_group_ptr->getCurrentPose().pose;
    current_robot_joint = _move_group_ptr->getCurrentJointValues();
    _model_state.set_model_position_world("EndEffectorSphere", current_robot_pose);

    //Get the current end effector matrix
    current_end_effector_matrix = Utilities::pose_to_matrix(current_robot_pose);
    current_pos = current_end_effector_matrix.block<3,1>(0,3);
    //Set target_pos as the target position
    target_pos = target_matrix.block<3, 1>(0, 3);
    Eigen::Matrix3d target_rot = target_matrix.block<3,3>(0,0);
    KDL::Rotation tcp_target(target_rot(0,0),target_rot(0,1),target_rot(0,2),target_rot(1,0),target_rot(1,1),target_rot(1,2),target_rot(2,0),target_rot(2,1),target_rot(2,2));

    KDL::JntArray q_init(_my_chain.getNrOfJoints());
    q_init(0) = current_robot_joint[0];
    q_init(1) = current_robot_joint[1];
    q_init(2) = current_robot_joint[2];
    q_init(3) = current_robot_joint[3];
    q_init(4) = current_robot_joint[4];
    q_init(5) = current_robot_joint[5];

    //Compute current tcp position
    KDL::Frame tcp_pos_start;
    _fk_solver->JntToCart(q_init, tcp_pos_start);
/*    ROS_INFO("Position read: %f %f %f", current_pos(0), current_pos(1), current_pos(2));
    ROS_INFO("Current tcp Position/Twist KDL:");
    ROS_INFO("Position: %f %f %f", tcp_pos_start.p(0), tcp_pos_start.p(1), tcp_pos_start.p(2));
    ROS_INFO("Orientation: %f %f %f", tcp_pos_start.M(0,0), tcp_pos_start.M(1,0), tcp_pos_start.M(2,0));
*/
    KDL::JntArray q_out(_my_chain.getNrOfJoints());
    KDL::Frame dest_frame(tcp_target, KDL::Vector(target_pos(0), target_pos(1), target_pos(2)));
    if (_ik_solver->CartToJnt(q_init, dest_frame, q_out) < 0) {
       ROS_ERROR( "Something really bad happened. You are in trouble now");
   } else {
       // parse output of ik_solver to the robot
       /*
       cout<<"current:"<<endl;
       for (unsigned int j = 0; j <_my_chain.getNrOfJoints(); j++) {
           std::cout << q_init(j) << "  ";
       }
       std::cout << std::endl;
       cout<<"target:"<<endl;
       for (unsigned int j = 0; j <_my_chain.getNrOfJoints(); j++) {
           std::cout << q_out(j) << "  ";
       }
       std::cout << std::endl;
       */
       target_robot_joint = {q_out(0),q_out(1),q_out(2),q_out(3),q_out(4),q_out(5)};
       //vector<double> damp_euler = Utilities::RuckigCalculation_Jnt(current_robot_joint, target_robot_joint, current_angular_velocity, current_angular_acceleration, max_angular_velocity, max_angular_acceleration, max_angular_jerk, publish_period);
       vector<double> damp_euler = Utilities::OTGCalculation_Jnt(current_robot_joint, target_robot_joint, last_target_joint, profile_joint, idx_joint, current_angular_velocity, current_angular_acceleration, max_angular_velocity, max_angular_acceleration, max_angular_jerk, publish_period);
       joint_deltas.velocities = {current_angular_velocity[0],current_angular_velocity[1],current_angular_velocity[2],current_angular_velocity[3],current_angular_velocity[4],current_angular_velocity[5]};
   }
   joint_deltas.header.stamp = ros::Time::now();
   _twist_stamped_joint_pub.publish(joint_deltas);
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
