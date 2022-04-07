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
    _Cartesian_compute=true;
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
        _async_spinner = new ros::AsyncSpinner(2);
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
        _rviz_marker.initialize(_node_handle, _move_group_ptr);
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
          //_joint_state_sub = _node_handle->subscribe("/joint_states", 1, &RobotController::jointCallback, this);
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
    if(_Cartesian_compute)
    {
      outFile.open("cart_data.csv");
    }
    else
    {
      outFile.open("joint_data.csv");
    }
    outFile << joint_stream.rdbuf();
    std::cout << joint_stream.str() << std::endl;
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
    geometry_msgs::Pose rcm_pose = Utilities::matrix_to_pose(rcm_matrix);
    _model_state.set_model_position_world("RCMSphere", rcm_pose);
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

    _model_state.set_model_position_world("targetCylinder", cylinder_pose);
    _rviz_marker._update_target_position(cylinder_pose);
    _rviz_marker._rcm_pose = rcm_pose;
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
  unsigned int nj = _my_chain.getNrOfJoints();
  for(unsigned int i=0; i<nj; i++)
	{
		ROS_INFO("joint_name[%d]: %s", i, _my_chain.getSegment(i).getJoint().getName().c_str());
	}
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
  _joint_name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint","wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
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
  double angulation_angle = 0.523599;
  double smoothTime = 1;
  double max_linear_velocity = 0.03, max_linear_acceleration = 0.05, max_linear_jerk = 10;
  double max_angular_velocity = 0.5, max_angular_acceleration = 5, max_angular_jerk = 100;
  double alpha=0.00001;
  double eulerXVelocity = 0.0, eulerYVelocity = 0.0, eulerZVelocity = 0.0;
  double publish_period = 0.03;
  ros::Rate cmd_rate(1 / publish_period);
  geometry_msgs::TwistStamped twist;
  twist.header.frame_id = "world";

  geometry_msgs::Pose current_robot_pose = _move_group_ptr->getCurrentPose().pose;
  Eigen::Matrix4d current_end_effector_matrix = Utilities::pose_to_matrix(current_robot_pose);
  Eigen::Matrix4d current_scope_tip_matrix = getScopeTip(current_end_effector_matrix, angulation_angle);
  _target_matrix = Utilities::pose_to_matrix(_model_state.get_model_position_world("targetCylinder"));

  Eigen::Vector3d Xe = current_end_effector_matrix.block<3, 1>(0, 3);
  Eigen::Vector3d Xa = current_scope_tip_matrix.block<3, 1>(0, 3);
  Eigen::Vector3d Xb, Xc = incision_matrix.block<3, 1>(0, 3);
  Eigen::Vector3d last_target_pos = Xa;

  geometry_msgs::Pose current_ee_pose = Utilities::matrix_to_pose(current_end_effector_matrix);
  tf2::Quaternion current_quat(current_ee_pose.orientation.x, current_ee_pose.orientation.y, current_ee_pose.orientation.z, current_ee_pose.orientation.w);
  double current_x, current_y, current_z;
  tf2::Matrix3x3(current_quat).getRPY(current_x, current_y, current_z);
  Eigen::Vector3d last_target_euler(current_x, current_y, current_z);

  Eigen::Vector3d current_linear_velocity,current_linear_acceleration, current_angular_velocity,current_angular_acceleration;
  TrigonometricOTG* trajOTG_pos_ptr;
  TrigonometricOTG* trajOTG_euler_ptr;
  int idx_pos = 0, idx_euler=0;
  vector<vector<vector<double>>> profile_pos, profile_euler;
  Eigen::Vector3d damp_position, damp_euler;
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  while (ros::ok())
  {
    ///std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
    //damp_position = Utilities::SmoothDampVector(Xa, Xb, current_linear_velocity, smoothTime, max_linear_velocity, publish_period);
    //damp_position = Utilities::RuckigCalculation(Xa, Xb, current_linear_velocity, current_linear_acceleration, max_linear_velocity, max_linear_acceleration, max_linear_jerk, publish_period);
    damp_position = Utilities::OTGCalculationS(Xa, Xb, last_target_pos, trajOTG_pos_ptr, current_linear_velocity, current_linear_acceleration, max_linear_velocity, max_linear_acceleration, max_linear_jerk, alpha, publish_period);

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
    //damp_euler = Utilities::SmoothDampVector(current_euler, target_euler, current_angular_velocity, smoothTime, max_angular_velocity, publish_period);
    //damp_euler = Utilities::RuckigCalculation(current_euler, target_euler, current_angular_velocity, current_angular_acceleration, max_angular_velocity, max_angular_acceleration, max_angular_jerk, publish_period);
    damp_euler = Utilities::OTGCalculationS(current_euler, target_euler, last_target_euler, trajOTG_euler_ptr, current_angular_velocity, current_angular_acceleration, max_angular_velocity, max_angular_acceleration,max_angular_jerk,alpha, publish_period);

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
    Eigen::Vector3d f_der = current_linear_velocity;
    Eigen::Vector3d end_effector_linear_velocity = current_linear_velocity - 0.37 *
                        (f.norm()*f.norm()*f_der - (f.dot(f_der))*f)/(f.norm()*f.norm()*f.norm());

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
    
    printf("target x: %f y: %f z: %f\n", Xb(0), Xb(1), Xb(2));
    printf("current x: %f y: %f z: %f\n", Xa(0), Xa(1), Xa(2));

    printf("linear x: %f y: %f z: %f\n", twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z);
    printf("angular x: %f y: %f z: %f\n", twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z);

    _twist_stamped_pub.publish(twist);
    cmd_rate.sleep();
  }
}

void RobotController::_joint_computation_thread_func(){
  double smoothTime = 1;
  double max_angular_velocity = 1, max_angular_acceleration = 1, max_angular_jerk = 10;
  double publish_period = 0.03;
  double alpha = 0.00001;
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
  TrigonometricOTG* trajOTG_ptr;

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  while (ros::ok())
  {
    //auto start = chrono::high_resolution_clock::now();
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
    //printf("joint 0 %f, joint 1 %f, joint 2 %f\n",q_init(0), q_init(1), q_init(2) );
    //Compute current tcp position
    //KDL::Frame tcp_pos_start;
    //_fk_solver->JntToCart(q_init, tcp_pos_start);
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
       //vector<double> damp_euler = Utilities::OTGCalculation_Jnt(current_robot_joint, target_robot_joint, last_target_joint, profile_joint, idx_joint, current_angular_velocity, current_angular_acceleration, max_angular_velocity, max_angular_acceleration, max_angular_jerk, publish_period);
       vector<double> damp_euler = Utilities::OTGCalculation_JntS(current_robot_joint, target_robot_joint, last_target_joint, trajOTG_ptr, current_angular_velocity, current_angular_acceleration, max_angular_velocity, max_angular_acceleration, max_angular_jerk, alpha, 0.012);
       joint_stream << target_robot_joint[0] << ","<< target_robot_joint[1] << ","<< target_robot_joint[2] << ","<< target_robot_joint[3] << ","<< target_robot_joint[4] << ","<< target_robot_joint[5] << ","<<damp_euler[0] << ","<< damp_euler[1] << ","<< damp_euler[2] << ","<< damp_euler[3] << ","<< damp_euler[4] << ","<< damp_euler[5] << ","<< current_robot_joint[0] << ","<< current_robot_joint[1] << ","<< current_robot_joint[2] << ","<< current_robot_joint[3] << ","<< current_robot_joint[4] << ","<< current_robot_joint[5] << ",";
       joint_stream.seekp(-1, std::ios_base::end);
       joint_stream << "\n";
       joint_deltas.velocities = {current_angular_velocity[0],current_angular_velocity[1],current_angular_velocity[2],current_angular_velocity[3],current_angular_velocity[4],current_angular_velocity[5]};
   }
   joint_deltas.header.stamp = ros::Time::now();
   _twist_stamped_joint_pub.publish(joint_deltas);
/*
   auto stop = chrono::high_resolution_clock::now();
   auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
   if(duration.count()>15000)
   {
     cout << "Time taken by function: "
              << duration.count() << " microseconds" << endl;
    }
*/
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
    geometry_msgs::Pose current_pose;// = _model_state.get_model_position("targetCylinder");
    current_pose.orientation.w = 1;
    //printf("position:%f %f %f\n", current_pose.position.x,current_pose.position.y,current_pose.position.z);
    //printf("orientation:%f %f %f %f\n", current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z, current_pose.orientation.w);
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
    //geometry_msgs::Pose target_pose = _model_state.get_model_position_world("targetCylinder");

    Eigen::Matrix4d temp_matrix = Utilities::pose_to_matrix(_rviz_marker._target_pose);
    Eigen::Matrix4d temp_matrix1 = Utilities::pose_to_matrix(current_pose);
    Eigen::Matrix4d new_matrix = temp_matrix*temp_matrix1;
    geometry_msgs::Pose target_pose = Utilities::matrix_to_pose(new_matrix);

    _rviz_marker._update_target_position(target_pose);


    //printf("gazebo:%f %f %f\n", target_pose.position.x,target_pose.position.y,target_pose.position.z);
    //printf("rviz:%f %f %f\n", _rviz_marker._target_pose.position.x,_rviz_marker._target_pose.position.y,_rviz_marker._target_pose.position.z);
    path_thread.lock();
    _target_matrix = Utilities::pose_to_matrix(target_pose);
    path_thread.unlock();

    //Notify the semaphore of new cylinder position
    //_new_position_available_sem->notify(std::this_thread::get_id());
}
