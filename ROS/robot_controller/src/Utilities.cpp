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

void Utilities::correct_joint_range(double& current_x, double& current_y, double& current_z, double& target_x, double& target_y, double& target_z){
  if(abs(target_x-current_x)> M_PI)
  {
    current_x = fmod(current_x+2*M_PI,2*M_PI);
    target_x = fmod(target_x+2*M_PI,2*M_PI);
  }
  if(abs(target_y-current_y)> M_PI)
  {
    current_y = fmod(current_y+2*M_PI,2*M_PI);
    target_y = fmod(target_y+2*M_PI,2*M_PI);
  }
  if(abs(target_z-current_z)> M_PI)
  {
    current_z = fmod(current_z+2*M_PI,2*M_PI);
    target_z = fmod(target_z+2*M_PI,2*M_PI);
  }
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


double Utilities::get_angle_betweem_two_vectors(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d n)
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

Eigen::Vector3d Utilities::RuckigCalculation(Eigen::Vector3d current, Eigen::Vector3d target, Eigen::Vector3d &currentVelocity, Eigen::Vector3d &currentAcceleration, double maxVel, double maxAccel, double maxJerk, double deltaTime)
{

  //test ruckig
  ruckig::Ruckig<3> otg {deltaTime};  // control cycle
  ruckig::InputParameter<3> input;
  ruckig::OutputParameter<3> output;
  // Set input parameters
  input.current_position = {current(0), current(1), current(2)};
  input.current_velocity = {currentVelocity(0), currentVelocity(1), currentVelocity(2)};
  input.current_acceleration = {currentAcceleration(0), currentAcceleration(1), currentAcceleration(2)};

  input.target_position = {target(0), target(1), target(2)};
  input.target_velocity = {0.0, 0.0, 0.0};
  input.target_acceleration = {0.0, 0.0, 0.0};

  input.max_velocity = {maxVel, maxVel, maxVel};
  input.max_acceleration = {maxAccel, maxAccel, maxAccel};
  input.max_jerk = {maxJerk, maxJerk, maxJerk};
  otg.update(input, output);
  auto& vel = output.new_velocity;
  auto& accel = output.new_acceleration;
  Eigen::Vector3d otg_vel(vel[0],vel[1],vel[2]);
  Eigen::Vector3d otg_accel(accel[0],accel[1],accel[2]);
  // std::cout << "t | p1 | p2 | p3" << std::endl;
  // std::cout << output.time << " " << pos[0] << " " << pos[1] << " " << pos[2] << " "<< vel[0] << " " << vel[1] << " " << vel[2] << " " << accel[0] << " " << accel[1] << " " << accel[2] << " " << std::endl;
  // std::cout << output.time << " " << damp_position[0] << " " << damp_position[1] << " " << damp_position[2] << " "<< currentVelocity[0] << " " << currentVelocity[1] << " " << currentVelocity[2] << " " << std::endl;
  // std::cout << "Calculation duration: " << output.calculation_duration << " [micros]." << std::endl;
  currentVelocity = otg_vel;
  currentAcceleration = otg_accel;

  auto& pos = output.new_position;
  Eigen::Vector3d otg_pos(pos[0],pos[1],pos[2]);

  return otg_pos;
}

vector<double> Utilities::RuckigCalculation_Jnt(vector<double> current, vector<double> target, vector<double>& currentVelocity, vector<double> &currentAcceleration, double maxVel, double maxAccel, double maxJerk, double deltaTime)
{

  //test ruckig
  ruckig::Ruckig<6> otg {deltaTime};  // control cycle
  ruckig::InputParameter<6> input;
  ruckig::OutputParameter<6> output;
  // Set input parameters
  input.current_position = {current[0], current[1], current[2], current[3], current[4], current[5]};
  input.current_velocity = {currentVelocity[0], currentVelocity[1], currentVelocity[2], currentVelocity[3], currentVelocity[4], currentVelocity[5]};
  input.current_acceleration = {currentAcceleration[0], currentAcceleration[1], currentAcceleration[2],currentAcceleration[3], currentAcceleration[4], currentAcceleration[5]};
  input.target_position = {target[0], target[1], target[2],target[3], target[4], target[5]};
  input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input.max_velocity = {maxVel, maxVel, maxVel, maxVel, maxVel, maxVel};
  input.max_acceleration = {maxAccel, maxAccel, maxAccel, maxAccel, maxAccel, maxAccel};
  input.max_jerk = {maxJerk, maxJerk, maxJerk, maxJerk, maxJerk, maxJerk};
  otg.update(input, output);
  auto& pos = output.new_position;
  auto& vel = output.new_velocity;
  auto& accel = output.new_acceleration;
  vector<double> otg_pos(pos.begin(),pos.end());
  vector<double> otg_vel(vel.begin(),vel.end());
  vector<double> otg_accel(accel.begin(),accel.end());
  // std::cout << "t | p1 | p2 | p3" << std::endl;
  // std::cout << output.time << " " << pos[0] << " " << pos[1] << " " << pos[2] << " "<< vel[0] << " " << vel[1] << " " << vel[2] << " " << accel[0] << " " << accel[1] << " " << accel[2] << " " << std::endl;
  // std::cout << output.time << " " << damp_position[0] << " " << damp_position[1] << " " << damp_position[2] << " "<< currentVelocity[0] << " " << currentVelocity[1] << " " << currentVelocity[2] << " " << std::endl;
  // std::cout << "Calculation duration: " << output.calculation_duration << " [micros]." << std::endl;
  currentVelocity = otg_vel;
  currentAcceleration = otg_accel;
  return otg_pos;
}

vector<vector<vector<double>>> Utilities::trajOTG(Eigen::Vector3d current, Eigen::Vector3d target, Eigen::Vector3d currentVelocity, Eigen::Vector3d currentAcceleration, double maxVel, double maxAccel, double maxJerk, double alpha, double deltaTime){
  int num_dof = 3;
  vector<double> current_accel = {currentAcceleration(0), currentAcceleration(1),currentAcceleration(2)};
  vector<double> current_vel = {currentVelocity(0),currentVelocity(1),currentVelocity(2)};
  vector<double> current_pos = {current(0),current(1),current(2)};
  vector<double> target_pos = {target(0), target(1), target(2)};
  TrigonometricOTG trajectoryOTG(num_dof, maxJerk, maxAccel, maxVel, current_accel, current_vel, current_pos, target_pos, alpha, deltaTime);
  vector<vector<vector<double>>> profile = trajectoryOTG.trajGenerator();

  return profile;
}

TrigonometricOTG* Utilities::trajOTG_ptr(Eigen::Vector3d current, Eigen::Vector3d target, Eigen::Vector3d currentVelocity, Eigen::Vector3d currentAcceleration, double maxVel, double maxAccel, double maxJerk, double alpha, double deltaTime){
  int num_dof = 3;
  vector<double> current_accel = {currentAcceleration(0), currentAcceleration(1),currentAcceleration(2)};
  vector<double> current_vel = {currentVelocity(0),currentVelocity(1),currentVelocity(2)};
  vector<double> current_pos = {current(0),current(1),current(2)};
  vector<double> target_pos = {target(0), target(1), target(2)};
  TrigonometricOTG* trajectoryOTG = new TrigonometricOTG(num_dof, maxJerk, maxAccel, maxVel, current_accel, current_vel, current_pos, target_pos, alpha, deltaTime);
  //printf("trajOTG_ptr\n");
  return trajectoryOTG;
}

Eigen::Vector3d Utilities::OTGCalculation(Eigen::Vector3d current, Eigen::Vector3d target, Eigen::Vector3d& last_target, vector<vector<vector<double>>>& profile, int& idx, Eigen::Vector3d& currentVelocity, Eigen::Vector3d& currentAcceleration, double maxVel, double maxAccel, double maxJerk, double publish_period)
{
  Eigen::Vector3d damp_position;
  if((last_target-target).norm()<=0.000001)
  {
    if((current-target).norm()<=0.000001 || profile.empty() || idx >= profile[0].size())
    {
      //printf("target size: %d\n", target.size());
      //printf("target: %f %f %f, current: %f %f %f\n", target(0), target(1),target(2), current(0), current(1),current(2));
      currentVelocity = {0.0,0.0,0.0};
      currentAcceleration = {0.0,0.0,0.0};
      damp_position = current;
      idx = 0;
      profile.clear();
    }
    else
    {
      //printf("moveing...%d %d %d %d\n", profile[0].size(), profile[1].size(), profile[2].size(), idx);
      //printf("current: %f %f %f, expect: %f %f %f\n", current(0), current(1),current(2), profile[0][idx][4], profile[1][idx][4],profile[2][idx][4]);
      vector<double> profile_x = profile[0][idx];
      vector<double> profile_y = profile[1][idx];
      vector<double> profile_z = profile[2][idx];
      //printf("vel: %f %f %f\n", profile_x[3], profile_y[3], profile_z[3]);
      currentVelocity = {profile_x[3], profile_y[3], profile_z[3]};
      currentAcceleration = {profile_x[2], profile_y[2], profile_z[2]};
      if (idx+1 < profile[0].size())
      {
        damp_position = {profile[0][idx+1][4],profile[1][idx+1][4],profile[2][idx+1][4]};
      }
      else{
        damp_position = {profile_x[4],profile_y[4],profile_z[4]};
      }
      idx+=1;
    }
  }
  else
  {
    idx = 0;
    profile.clear();
    last_target = target;
    profile =Utilities::trajOTG(current, target, currentVelocity, currentAcceleration, maxVel,maxAccel, maxJerk, 0.01, publish_period);
    vector<double> profile_x = profile[0][idx];
    vector<double> profile_y = profile[1][idx];
    vector<double> profile_z = profile[2][idx];
    currentVelocity = {profile_x[3], profile_y[3], profile_z[3]};
    currentAcceleration = {profile_x[2],profile_y[2],profile_z[2]};
    damp_position = {profile[0][idx+1][4],profile[1][idx+1][4],profile[2][idx+1][4]};
    //printf("start...\n");
    //printf("target: %f %f %f, compute: %f %f %f\n", target(0), target(1),target(2), profile[0].back()[4], profile[1].back()[4],profile[2].back()[4]);
    idx+=1;
  }

  return damp_position;
}

Eigen::Vector3d Utilities::OTGCalculationS(Eigen::Vector3d current, Eigen::Vector3d target, Eigen::Vector3d& last_target,TrigonometricOTG*& otg_ptr, Eigen::Vector3d& currentVelocity, Eigen::Vector3d& currentAcceleration, double maxVel, double maxAccel, double maxJerk, double alpha,double publish_period)
{
  //printf("OTGCalculationS\n");
  Eigen::Vector3d damp_position;
  if((last_target-target).norm()<=0.000001)
  {
    int n;
    if(otg_ptr)
    {
      n= ceil(otg_ptr->minT_/otg_ptr->rate_);
    }
    //printf("COMPUTED SIZE\n");

    if((current-target).norm()<=0.000001 ||otg_ptr==nullptr || otg_ptr->idx_ >= n)
    {
      //printf("target size: %d\n", target.size());
      //printf("target: %f %f %f, current: %f %f %f\n", target(0), target(1),target(2), current(0), current(1),current(2));
      currentVelocity = {0.0,0.0,0.0};
      currentAcceleration = {0.0,0.0,0.0};
      damp_position = current;
      otg_ptr=nullptr;
    }
    else
    {
      //printf("CASE2\n");
      vector<vector<double>> profile = otg_ptr->trajGeneratorS();
      //printf("moveing...%f %d\n", ceil(otg_ptr->minT_/otg_ptr->rate_), otg_ptr->idx_);
      //printf("current: %f %f %f, expect: %f %f %f\n", current(0), current(1),current(2), profile[0][idx][4], profile[1][idx][4],profile[2][idx][4]);
      vector<double> profile_x = profile[0];
      vector<double> profile_y = profile[1];
      vector<double> profile_z = profile[2];
      currentVelocity = {profile_x[3], profile_y[3], profile_z[3]};
      currentAcceleration = {profile_x[2],profile_y[2],profile_z[2]};
      damp_position = {profile_x[4],profile_y[4],profile_z[4]};
      //printf("current: %f %f %f, expect: %f %f %f, target: %f %f %f\n", current(0), current(1),current(2), profile_x[4], profile_y[4],profile_z[4], target(0),target(1),target(2));
      //printf("vel: %f %f %f\n", currentVelocity(0), currentVelocity(1),currentVelocity(2));

      //printf("start...\n");
      otg_ptr->idx_+=1;
      //printf("vel: %f %f %f\n", profile_x[3], profile_y[3], profile_z[3]);
    }
  }
  else
  {
    //printf("CASE1\n");
    last_target = target;
    otg_ptr=nullptr;
    otg_ptr = Utilities::trajOTG_ptr(current, target, currentVelocity, currentAcceleration, maxVel,maxAccel, maxJerk, alpha, publish_period);
    otg_ptr->minimumTime();
    otg_ptr->idx_+=1;
    //printf("minT: %f, idx: %d\n", otg_ptr->minT_, otg_ptr->idx_);
    vector<vector<double>> profile = otg_ptr->trajGeneratorS();
    vector<double> profile_x = profile[0];
    vector<double> profile_y = profile[1];
    vector<double> profile_z = profile[2];
    currentVelocity = {profile_x[3], profile_y[3], profile_z[3]};
    currentAcceleration = {profile_x[2],profile_y[2],profile_z[2]};
    damp_position = {profile_x[4],profile_y[4],profile_z[4]};
    //printf("start...\n");
    //printf("current: %f %f %f, expect: %f %f %f, target: %f %f %f\n", current(0), current(1),current(2), profile_x[4], profile_y[4],profile_z[4], target(0),target(1),target(2));
    //printf("vel: %f %f %f\n", currentVelocity(0), currentVelocity(1),currentVelocity(2));
    otg_ptr->idx_+=1;
  }

  return damp_position;
}


vector<vector<vector<double>>> Utilities::trajOTG_Jnt(vector<double> current, vector<double> target, vector<double> currentVelocity, vector<double> currentAcceleration, double maxVel, double maxAccel, double maxJerk, double alpha, double deltaTime){
  int num_dof = 6;
  TrigonometricOTG trajectoryOTG(num_dof, maxJerk, maxAccel, maxVel, currentAcceleration, currentVelocity, current, target, alpha, deltaTime);
  vector<vector<vector<double>>> profile = trajectoryOTG.trajGenerator();

  return profile;
}

TrigonometricOTG* Utilities::trajOTG_Jnt_ptr(vector<double> current, vector<double> target, vector<double> currentVelocity, vector<double> currentAcceleration, double maxVel, double maxAccel, double maxJerk, double alpha, double deltaTime) {
    int num_dof = 6;
    TrigonometricOTG* trajectoryOTG = new TrigonometricOTG(num_dof, maxJerk, maxAccel, maxVel, currentAcceleration, currentVelocity, current, target, alpha, deltaTime);

    return trajectoryOTG;
}

vector<double> Utilities::OTGCalculation_Jnt(vector<double> current, vector<double> target, vector<double>& last_target, vector<vector<vector<double>>>& profile, int& idx, vector<double>& currentVelocity, vector<double>& currentAcceleration, double maxVel, double maxAccel, double maxJerk, double publish_period)
{
  vector<double> damp_position;
  if(norm_between_vectors(last_target,target)<=0.000001)
  {
    if(norm_between_vectors(current,target)<=0.000001 || profile.empty() || idx >= profile[0].size())
    {
      //printf("target size: %d\n", target.size());
      //printf("target: %f %f %f, current: %f %f %f\n", target(0), target(1),target(2), current(0), current(1),current(2));
      currentVelocity = {0.0,0.0,0.0,0.0,0.0,0.0};
      currentAcceleration = {0.0,0.0,0.0,0.0,0.0,0.0};
      damp_position = current;
      idx = 0;
      profile.clear();
    }
    else
    {
      currentVelocity = {profile[0][idx][3], profile[1][idx][3], profile[2][idx][3],profile[3][idx][3], profile[4][idx][3], profile[5][idx][3]};
      currentAcceleration = {profile[0][idx][2], profile[1][idx][2], profile[2][idx][2],profile[3][idx][2], profile[4][idx][2], profile[5][idx][2]};
      if (idx+1 < profile[0].size())
      {
        damp_position = {profile[0][idx+1][4],profile[1][idx+1][4],profile[2][idx+1][4],profile[3][idx+1][4],profile[4][idx+1][4],profile[5][idx+1][4]};
      }
      else{
        damp_position = {profile[0][idx][4],profile[1][idx][4],profile[2][idx][4],profile[3][idx][4],profile[4][idx][4],profile[5][idx][4]};
      }
      idx+=1;
    }
  }
  else
  {
    idx = 0;
    profile.clear();
    last_target = target;
    profile =Utilities::trajOTG_Jnt(current, target, currentVelocity, currentAcceleration, maxVel,maxAccel, maxJerk, 0.01, publish_period);
    //printf("vel: %f %f %f\n", profile_x[3], profile_y[3], profile_z[3]);
    currentVelocity = {profile[0][idx][3], profile[1][idx][3], profile[2][idx][3],profile[3][idx][3], profile[4][idx][3], profile[5][idx][3]};
    currentAcceleration = {profile[0][idx][2], profile[1][idx][2], profile[2][idx][2],profile[3][idx][2], profile[4][idx][2], profile[5][idx][2]};
    damp_position = {profile[0][idx+1][4],profile[1][idx+1][4],profile[2][idx+1][4],profile[3][idx+1][4],profile[4][idx+1][4],profile[5][idx+1][4]};
    //printf("start...\n");
    //printf("current: %f %f %f, expect: %f %f %f\n", current(0), current(1),current(2), profile[0][idx][4], profile[1][idx][4],profile[2][idx][4]);
    idx+=1;
  }

  return damp_position;
}

vector<double> Utilities::OTGCalculation_JntS(vector<double> current, vector<double> target, vector<double>& last_target, TrigonometricOTG*& otg_ptr, vector<double>& currentVelocity, vector<double>& currentAcceleration, double maxVel, double maxAccel, double maxJerk, double alpha, double publish_period)
{
    //printf("OTGCalculationS\n");
    vector<double> damp_position;
    if (norm_between_vectors(last_target, target) <= 0.000001)
    {
        int n;
        if (otg_ptr) { n = ceil(otg_ptr->minT_ / otg_ptr->rate_); }
        //printf("COMPUTED SIZE\n");

        if (norm_between_vectors(current, target) <= 0.000001 || otg_ptr == nullptr || otg_ptr->idx_ >= n)
        {
            //printf("target size: %d\n", target.size());
            //printf("target: %f %f %f, current: %f %f %f\n", target(0), target(1),target(2), current(0), current(1),current(2));
            currentVelocity = { 0.0,0.0,0.0,0.0,0.0,0.0 };
            currentAcceleration = { 0.0,0.0,0.0,0.0,0.0,0.0 };
            damp_position = current;
            otg_ptr = nullptr;
        }
        else
        {
            //printf("CASE2\n");
            vector<vector<double>> profile = otg_ptr->trajGeneratorS();
            //printf("moveing...%f %d\n", ceil(otg_ptr->minT_/otg_ptr->rate_), otg_ptr->idx_);
            //printf("current: %f %f %f, expect: %f %f %f\n", current(0), current(1),current(2), profile[0][4], profile[1][4],profile[2][4]);
            vector<double> profile_1 = profile[0];
            vector<double> profile_2 = profile[1];
            vector<double> profile_3 = profile[2];
            vector<double> profile_4 = profile[3];
            vector<double> profile_5 = profile[4];
            vector<double> profile_6 = profile[5];
            currentVelocity = { profile_1[3], profile_2[3], profile_3[3],profile_4[3], profile_5[3], profile_6[3] };
            currentAcceleration = { profile_1[2],profile_2[2],profile_3[2],profile_4[2], profile_5[2], profile_6[2] };
            damp_position = { profile_1[4],profile_2[4],profile_3[4],profile_4[4], profile_5[4], profile_6[4] };
            //printf("current: %f %f %f %f %f %f, expect: %f %f %f %f %f %f, target: %f %f %f %f %f %f\n", current[0], current[1],current[2],current[3], current[4],current[5], profile_1[4], profile_2[4],profile_3[4],profile_4[4], profile_5[4],profile_6[4], target[0],target[1],target[2], target[3],target[4],target[5]);

            //printf("start...\n");
            otg_ptr->idx_ += 1;
            //printf("vel: %f %f %f %f %f %f\n", profile_1[3], profile_2[3], profile_3[3],profile_4[3], profile_5[3], profile_6[3]);
        }
    }
    else
    {
        //printf("CASE1\n");
        last_target = target;
        otg_ptr = nullptr;
        //printf("creating new ptr\n");
        otg_ptr = Utilities::trajOTG_Jnt_ptr(current, target, currentVelocity, currentAcceleration, maxVel, maxAccel, maxJerk, alpha, publish_period);
        //printf("created new ptr\n");
        otg_ptr->minimumTime();
        otg_ptr->idx_+=1;
        //printf("minimumTime: %f\n", otg_ptr->minT_);
        vector<vector<double>> profile = otg_ptr->trajGeneratorS();
        vector<double> profile_1 = profile[0];
        vector<double> profile_2 = profile[1];
        vector<double> profile_3 = profile[2];
        vector<double> profile_4 = profile[3];
        vector<double> profile_5 = profile[4];
        vector<double> profile_6 = profile[5];
        currentVelocity = { profile_1[3], profile_2[3], profile_3[3],profile_4[3], profile_5[3], profile_6[3] };
        currentAcceleration = { profile_1[2],profile_2[2],profile_3[2],profile_4[2], profile_5[2], profile_6[2] };
        damp_position = { profile_1[4],profile_2[4],profile_3[4],profile_4[4], profile_5[4], profile_6[4] };
        //printf("current: %f %f %f %f %f %f, expect: %f %f %f %f %f %f, target: %f %f %f %f %f %f\n", current[0], current[1],current[2],current[3], current[4],current[5], profile_1[4], profile_2[4],profile_3[4],profile_4[4], profile_5[4],profile_6[4], target[0],target[1],target[2], target[3],target[4],target[5]);
        //printf("vel: %f %f %f %f %f %f\n", profile_1[3], profile_2[3], profile_3[3],profile_4[3], profile_5[3], profile_6[3]);

        //printf("start...\n");
        otg_ptr->idx_ += 1;
    }

    return damp_position;
}


Eigen::Vector3d Utilities::SmoothDampVector(Eigen::Vector3d current, Eigen::Vector3d target, Eigen::Vector3d &currentVelocity, double smoothTime,
                                 double maxSpeed, double deltaTime)
{
  double output_x = 0.0;
  double output_y = 0.0;
  double output_z = 0.0;

  // Based on Game Programming Gems 4 Chapter 1.10
  smoothTime = std::max(0.0001, smoothTime);
  double omega = 2.0 / smoothTime;

  double x = omega * deltaTime;
  double exp = 1.0 / (1 + x + 0.48 * x * x + 0.235 * x * x * x);

  double change_x = current.x() - target.x();
  double change_y = current.y() - target.y();
  double change_z = current.z() - target.z();
  Eigen::Vector3d originalTo = target;

  // Clamp maximum speed
  double maxChange = maxSpeed * smoothTime;

  double maxChangeSq = maxChange * maxChange;
  double sqrmag = change_x * change_x + change_y * change_y + change_z * change_z;
  if (sqrmag > maxChangeSq)
  {
    double mag = std::sqrt(sqrmag);
    change_x = change_x / mag * maxChange;
    change_y = change_y / mag * maxChange;
    change_z = change_z / mag * maxChange;
  }

  target(0) = current.x() - change_x;
  target(1) = current.y() - change_y;
  target(2) = current.z() - change_z;

  double temp_x = (currentVelocity.x() + omega * change_x) * deltaTime;
  double temp_y = (currentVelocity.y() + omega * change_y) * deltaTime;
  double temp_z = (currentVelocity.z() + omega * change_z) * deltaTime;

  currentVelocity(0) = (currentVelocity.x() - omega * temp_x) * exp;
  currentVelocity(1) = (currentVelocity.y() - omega * temp_y) * exp;
  currentVelocity(2) = (currentVelocity.z() - omega * temp_z) * exp;

  output_x = target.x() + (change_x + temp_x) * exp;
  output_y = target.y() + (change_y + temp_y) * exp;
  output_z = target.z() + (change_z + temp_z) * exp;

  // Prevent overshooting
  double origMinusCurrent_x = originalTo.x() - current.x();
  double origMinusCurrent_y = originalTo.y() - current.y();
  double origMinusCurrent_z = originalTo.z() - current.z();
  double outMinusOrig_x = output_x - originalTo.x();
  double outMinusOrig_y = output_y - originalTo.y();
  double outMinusOrig_z = output_z - originalTo.z();

  if (origMinusCurrent_x * outMinusOrig_x + origMinusCurrent_y * outMinusOrig_y + origMinusCurrent_z * outMinusOrig_z > 0)
  {
    output_x = originalTo.x();
    output_y = originalTo.y();
    output_z = originalTo.z();

    currentVelocity(0) = (output_x - originalTo.x()) / deltaTime;
    currentVelocity(1) = (output_y - originalTo.y()) / deltaTime;
    currentVelocity(2) = (output_z - originalTo.z()) / deltaTime;
  }
  //return currentVelocity;
  return Eigen::Vector3d(output_x, output_y, output_z);
}

double Utilities::Repeat(double t, double length){
  return Utilities::clamp(t -std::floor(t / length) * length, 0.0, length);
}

double Utilities::DeltaAngle(double current, double target){
  double delta = Utilities::Repeat((target - current), 360.0);
  if (delta > 180.0)
    delta -= 360.0;
  return delta;
}

double Utilities::SmoothDamp(double current, double target , double &currentVelocity, double smoothTime,
                  double maxSpeed, double deltaTime )
{
  //std:: cout << current << ", " << target << "," << currentVelocity << std::endl;
  smoothTime = std::max(0.0001, smoothTime);
  double omega = 2.0 / smoothTime;

  double x = omega * deltaTime;
  double exp = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x);
  double change = current - target;
  double originalTo = target;

  // Clamp maximum speed
  double maxChange = maxSpeed * smoothTime;
  change = clamp(change, -maxChange, maxChange);
  target = current - change;

  double temp = (currentVelocity + omega * change) * deltaTime;
  currentVelocity = (currentVelocity - omega * temp) * exp;
  double output = target + (change + temp) * exp;

  // Prevent overshooting
  if (originalTo - current > 0.0 == output > originalTo)
  {
    output = originalTo;
    currentVelocity = (output - originalTo) / deltaTime;
  }

  return output;
}

double Utilities::SmoothDampAngle(double current, double target,  double &currentVelocity, double smoothTime,  double maxSpeed,  double deltaTime){
  target = current + DeltaAngle(current, target);
  return SmoothDamp(current, target, currentVelocity, smoothTime, maxSpeed, deltaTime);
}

double Utilities::clamp(double n, double lower, double upper)
{
  return std::max(lower, std::min(n, upper));
}

double Utilities::norm_between_vectors(vector<double> v1, vector<double> v2)
{
  vector<double> temp;
  for(int i=0; i<v1.size();i++)
  {
    temp.push_back(v1[i]-v2[i]);
  }
  double s;
  for(int i=0; i<v1.size();i++)
  {
    s += temp[i]*temp[i];
  }

  return sqrt(s);
}

double Utilities::get_angle_betweem_vectors(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d n)
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
