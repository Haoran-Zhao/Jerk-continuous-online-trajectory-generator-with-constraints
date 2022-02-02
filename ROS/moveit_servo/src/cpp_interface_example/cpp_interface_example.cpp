/*******************************************************************************
 *      Title     : cpp_interface_example.cpp
 *      Project   : moveit_servo
 *      Created   : 11/20/2019
 *      Author    : Andy Zelenak
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <std_msgs/Int8.h>

#include <moveit_servo/servo.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>

#include <moveit_servo/ScopeTip.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <algorithm>

#include "../custom/GazeboModelState.h"
#include "../custom/Utilities.h"


static const std::string LOGNAME = "cpp_interface_example";

moveit_servo::Servo *servo;
//GazeboModelState model_state;

//Rviz Class
class RvizMarker{
public:
  RvizMarker(ros::NodeHandle node){
    _node = &node;
    marker_pub = _node->advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  }

  void drawSphere(geometry_msgs::Pose pose){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.id = _id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose = pose;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker_pub.publish(marker);
    _id += 1;
  }

private:
  ros::NodeHandle *_node;
  ros::Publisher marker_pub;
  int _id = 0 ;
};
RvizMarker *rviz;


// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
  StatusMonitor(ros::NodeHandle &nh, const std::string &topic)
  {
    sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
  }

private:
  void statusCB(const std_msgs::Int8ConstPtr &msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto &status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status: " << status_str);
    }
  }
  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  ros::Subscriber sub_;
};

ros::Publisher twist_stamped_pub;
ros::Publisher velocity_test_pub;

Eigen::Matrix4d start_tip;
Eigen::Matrix4d end_tip;
Eigen::Matrix4d end_effector_start;
Eigen::Matrix4d end_effector_end;

Eigen::Matrix4d current_scope_tip_matrix;
Eigen::Matrix4d target_scope_tip_matrix;

Eigen::Matrix4d incision_matrix;

std::mutex thread_mutex;

double scope_length;

Eigen::Quaterniond euler2Quaternion( double roll, double pitch, double yaw )
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

double calculate_line_to_point_distance(Eigen::Vector3d p, Eigen::Vector3d a, Eigen::Vector3d b)
{
  double lamb = ((p - a).dot(b - a)) / (b - a).dot(b - a);
  Eigen::Vector3d p1 = a + lamb * (b - a);
  return (p - p1).norm();
}

double radian_to_degree(double radian)
{
  return radian * 180 / M_PI;
}

double degree_to_radian(double degree)
{
  return degree * M_PI / 180;
}

Eigen::Vector3d rotate(Eigen::Vector3d p, double theta, Eigen::Vector3d p1, Eigen::Vector3d p2)
{
  Eigen::Vector3d s = (p2 - p1) / (p2 - p1).norm();
  double x, y, z, a, b, c, u, v, w;
  x = p.x();
  y = p.y();
  z = p.z();
  a = p1.x();
  b = p1.y();
  c = p1.z();
  u = s.x();
  v = s.y();
  w = s.z();

  Eigen::Vector3d point(((a * (v * v + w * w) - u * (b * v + c * w - u * x - v * y - w * z)) * (1 - cos(theta)) + x * cos(theta) + (-c * v + b * w - w * y + v * z) * sin(theta)),
                        ((b * (u * u + w * w) - v * (a * u + c * w - u * x - v * y - w * z)) * (1 - cos(theta)) + y * cos(theta) + (c * u - a * w + w * x - u * z) * sin(theta)),
                        ((c * (u * u + v * v) - w * (a * u + b * v - u * x - v * y - w * z)) * (1 - cos(theta)) + z * cos(theta) + (-b * u + a * v - v * x + u * y) * sin(theta)));

  return point; //=new value for x axis, normalize and set it equal to new x axis
                //Y = Z x X
}

Eigen::Matrix4d getScopeTip(Eigen::Matrix4d end_effector_matrix)
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
  tip_quat.setEulerZYX(current_rot_Z, current_rot_Y, current_rot_X - 0.523599);
  end_pose.orientation.x = tip_quat.x();
  end_pose.orientation.y = tip_quat.y();
  end_pose.orientation.z = tip_quat.z();
  end_pose.orientation.w = tip_quat.w();

  end_effector_matrix = Utilities::pose_to_matrix(end_pose);

  return end_effector_matrix;
}

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

Eigen::Vector3d compute_linear_velocity(Eigen::Vector3d Xa, Eigen::Vector3d Xb, Eigen::Vector3d Xc, double lamda, Eigen::Matrix4d start_matrix, Eigen::Matrix4d end_matrix)
{
  Eigen::Vector3d f_lamda = Xc - Xa - lamda * (Xb - Xa);
  Eigen::Vector3d f_der_lamda = Xa - Xb;
  double f_lamda_mod = f_lamda.norm();
  double dot_prod = f_lamda.dot(f_der_lamda);

  Eigen::Vector3d linear_vel = Xb - Xa + 0.37 * ((f_lamda_mod * f_lamda_mod * f_der_lamda) - (dot_prod * f_lamda)) / (f_lamda_mod * f_lamda_mod * f_lamda_mod);

  return linear_vel;
}

Eigen::Vector3d compute_angular_velocity(Eigen::Vector3d Xa, Eigen::Vector3d Xb, Eigen::Vector3d Xc, double lamda, Eigen::Matrix4d start_matrix, Eigen::Matrix4d end_matrix)
{
  Eigen::Vector3d X = Xa + lamda * (Xb - Xa) + 0.37 * (Xc - Xa - lamda * (Xb - Xa)) / (Xc - Xa - lamda * (Xb - Xa)).norm();
  Eigen::Vector3d zAxis = (Xa - Xc + lamda * (Xb - Xa)).normalized();

  Eigen::Vector3d n = ((Xa - Xc).cross(Xb - Xc)).normalized();
  Eigen::Vector3d n2_start = (Xa - Xc).normalized();
  Eigen::Vector3d n2_end = (Xb - Xc).normalized();

  double thetaA = get_angle_betweem_two_vectors((n.cross(n2_start)), (start_matrix.block<3, 1>(0, 1)).normalized(), n2_start);
  double thetaB = get_angle_betweem_two_vectors((n.cross(n2_end)), (end_matrix.block<3, 1>(0, 1)).normalized(), n2_end);

  double thetaLamda = thetaA + lamda * (thetaB - thetaA);

  Eigen::Vector3d yAxis = cos(thetaLamda) * (n.cross(zAxis)) + sin(thetaLamda) * n;
  Eigen::Vector3d xAxis = -sin(thetaLamda) * (n.cross(zAxis)) + cos(thetaLamda) * n;

  Eigen::Matrix3d rot_matrix;
  rot_matrix.setIdentity();
  rot_matrix.block<3, 1>(0, 0) = xAxis;
  rot_matrix.block<3, 1>(0, 1) = yAxis;
  rot_matrix.block<3, 1>(0, 2) = zAxis;

  //Calculate derivate of Z axis
  Eigen::Vector3d f1_lamda = Xa - Xc + lamda * (Xb - Xa);
  Eigen::Vector3d f1_der_lamda = Xb - Xa;
  double f1_lamda_mod = f1_lamda.norm();
  double dot_prod = f1_lamda.dot(f1_der_lamda);

  Eigen::Vector3d zAxis_der = ((f1_lamda_mod * f1_lamda_mod * f1_der_lamda) - (dot_prod * f1_lamda)) / (f1_lamda_mod * f1_lamda_mod * f1_lamda_mod);

  Eigen::Vector3d yAxis_der = -1 * (thetaB - thetaA) * sin(thetaA + lamda * (thetaB - thetaA)) * n.cross(zAxis) +
                              cos(thetaA + lamda * (thetaB - thetaA)) * (n.cross(zAxis_der)) +
                              (thetaB - thetaA) * cos(thetaA + lamda * (thetaB - thetaA)) * n;

  Eigen::Vector3d xAxis_der = -1 * (thetaB - thetaA) * cos(thetaA + lamda * (thetaB - thetaA)) * n.cross(zAxis) +
                              -1 * sin(thetaA + lamda * (thetaB - thetaA)) * n.cross(zAxis_der) +
                              -1 * (thetaB - thetaA) * sin(thetaA + lamda * (thetaB - thetaA)) * n;

  Eigen::Matrix3d der_matrix;
  der_matrix.setIdentity();
  der_matrix.block<3, 1>(0, 0) = xAxis_der;
  der_matrix.block<3, 1>(0, 1) = yAxis_der;
  der_matrix.block<3, 1>(0, 2) = zAxis_der;

  // std::cout << "Transpose multiply matrix" << std::endl;
  // std::cout << der_matrix * rot_matrix.transpose() << std::endl;
  Eigen::Matrix3d skew = der_matrix * rot_matrix.transpose();

  double angular_x = skew(2, 1);
  double angular_y = skew(0, 2);
  double angular_z = skew(1, 0);

  return Eigen::Vector3d(angular_x, angular_y, angular_z);
}

Eigen::Vector3d compute_linear_velocity_combination(Eigen::Vector3d X0, Eigen::Vector3d X1, Eigen::Vector3d X2, Eigen::Vector3d Xc, double lamda)
{
  Eigen::Vector3d f_lamda = ((1 - lamda) * (1 - lamda) * X0 + 2 * (1 - lamda) * lamda * X1 + lamda * lamda * X2) - Xc;
  double f_lamda_mod = f_lamda.norm();
  Eigen::Vector3d f_der_lamda = 2 * X0 * (lamda - 1) + 2 * X1 * (1 - 2 * lamda) + 2 * lamda * X2;

  Eigen::Vector3d linear_vel = -1 * 2 * (1 - lamda) * X0 + (1 - 2 * lamda) * 2 * X1 + 2 * lamda * X2 - 0.37 * ((f_lamda_mod * f_lamda_mod * f_der_lamda) - (f_lamda.dot(f_der_lamda) * f_lamda)) / (f_lamda_mod * f_lamda_mod * f_lamda_mod);

  //End Effector Position
  Eigen::Vector3d end = f_lamda + 0.37 * (Xc - f_lamda).normalized();
  return end;
}

tf2::Quaternion compute_angular_combination(Eigen::Vector3d X0, Eigen::Vector3d X1, Eigen::Vector3d X2, Eigen::Vector3d Xc, double lamda, Eigen::Matrix4d start_matrix, Eigen::Matrix4d end_matrix)
{
  Eigen::Vector3d Xp = (1 - lamda) * (1 - lamda) * X0 + 2 * (1 - lamda) * lamda * X1 + lamda * lamda * X2;
  Eigen::Vector3d zAxis = (Xp - Xc).normalized();

  Eigen::Vector3d tang = (2 * X0 * (lamda - 1) + 2 * X1 * (1 - 2 * lamda) + 2 * lamda * X2).normalized();

  Eigen::Vector3d n = zAxis.cross(tang).normalized();

  Eigen::Vector3d nCrossZAxis_start = (((X0 - Xc).normalized()).cross((X1 - X0).normalized())).cross((X1 - X0).normalized());
  Eigen::Vector3d nCrossZAxis_end = (((X2 - Xc).normalized()).cross((X2 - X1).normalized())).cross((X2 - X1).normalized());

  double thetaA = get_angle_betweem_two_vectors(nCrossZAxis_start, (start_matrix.block<3, 1>(0, 1)).normalized(), (X0 - Xc).normalized());
  double thetaB = get_angle_betweem_two_vectors(nCrossZAxis_end, (end_matrix.block<3, 1>(0, 1)).normalized(), (X2 - Xc).normalized());
  double thetaLamda = thetaA + lamda * (thetaB - thetaA);

  Eigen::Vector3d yAxis = (cos(thetaLamda) * (n.cross(zAxis)) + sin(thetaLamda) * n).normalized();
  Eigen::Vector3d xAxis = -(-sin(thetaLamda) * (n.cross(zAxis)) + cos(thetaLamda) * n).normalized();

  Eigen::Matrix3d rot_matrix;
  rot_matrix.setIdentity();
  rot_matrix.block<3, 1>(0, 0) = xAxis;
  rot_matrix.block<3, 1>(0, 1) = yAxis;
  rot_matrix.block<3, 1>(0, 2) = zAxis;

  //Calculate derivate of Z axis
  Eigen::Vector3d f1_lamda = Xp - Xc;
  Eigen::Vector3d f1_der_lamda = 2 * X0 * (lamda - 1) + 2 * X1 * (1 - 2 * lamda) + 2 * lamda * X2;
  double f1_lamda_mod = f1_lamda.norm();
  double dot_prod = f1_lamda.dot(f1_der_lamda);

  Eigen::Vector3d zAxis_der = ((f1_lamda_mod * f1_lamda_mod * f1_der_lamda) - (dot_prod * f1_lamda)) / (f1_lamda_mod * f1_lamda_mod * f1_lamda_mod);

  Eigen::Vector3d yAxis_der = -1 * (thetaB - thetaA) * sin(thetaA + lamda * (thetaB - thetaA)) * n.cross(zAxis) +
                              cos(thetaA + lamda * (thetaB - thetaA)) * (n.cross(zAxis_der)) +
                              (thetaB - thetaA) * cos(thetaA + lamda * (thetaB - thetaA)) * n;

  Eigen::Vector3d xAxis_der = -1 * (thetaB - thetaA) * cos(thetaA + lamda * (thetaB - thetaA)) * n.cross(zAxis) +
                              -1 * sin(thetaA + lamda * (thetaB - thetaA)) * n.cross(zAxis_der) +
                              -1 * (thetaB - thetaA) * sin(thetaA + lamda * (thetaB - thetaA)) * n;

  Eigen::Matrix3d der_matrix;
  der_matrix.setIdentity();
  der_matrix.block<3, 1>(0, 0) = xAxis_der;
  der_matrix.block<3, 1>(0, 1) = yAxis_der;
  der_matrix.block<3, 1>(0, 2) = zAxis_der;

  // std::cout << "Transpose multiply matrix" << std::endl;
  // std::cout << der_matrix * rot_matrix.transpose() << std::endl;
  Eigen::Matrix3d skew = der_matrix * rot_matrix.transpose();

  double angular_x = skew(2, 1);
  double angular_y = skew(0, 2);
  double angular_z = skew(1, 0);

  //Computing End Effector Orientation
  Eigen::Matrix4d frame;
  frame.setIdentity();
  frame.block<3, 3>(0, 0) = rot_matrix;
  geometry_msgs::Pose rot_pose = Utilities::matrix_to_pose(frame);
  tf2::Quaternion quat(rot_pose.orientation.x, rot_pose.orientation.y, rot_pose.orientation.z, rot_pose.orientation.w);

  //return Eigen::Vector3d(angular_x, angular_y, angular_z);
  return quat;
}

void scopeTipCallback(const moveit_servo::ScopeTip &msg)
{
  //Gets the current end effector position
  Eigen::Isometry3d end_effector;
  servo->getEEFrameTransform(end_effector);
  current_scope_tip_matrix = getScopeTip(end_effector.matrix());

  //Stores the matrix for target scope tip
  target_scope_tip_matrix = Utilities::pose_to_matrix(msg.scope_tip_target);

  //Stores rcm matrix
  incision_matrix = Utilities::pose_to_matrix(msg.rcm);

  //Stores scope length
  scope_length = std::stod(msg.scope_length);

  double publish_period = 0.008;
  ros::Rate cmd_rate(1 / publish_period);
  auto twist = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();
  twist->header.frame_id = "world";

  auto twist_old = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();
  twist_old->header.frame_id = "world";
  twist->header.stamp = ros::Time::now();
  twist->twist.linear.x = 0;
  twist->twist.linear.y = 0;
  twist->twist.linear.z = 0;
  twist->twist.angular.x = 0;
  twist->twist.angular.y = 0;
  twist->twist.angular.z = 0;
  velocity_test_pub.publish(twist);
  twist_stamped_pub.publish(twist);
  cmd_rate.sleep();
  bool movement_completed = true;
  double time_to_reach_target = 3.5;
  double time = 0.0;
  double lamda = 0.0;

  Eigen::Vector3d Xa = current_scope_tip_matrix.block<3, 1>(0, 3);
  Eigen::Vector3d Xb = target_scope_tip_matrix.block<3, 1>(0, 3);
  Eigen::Vector3d Xc = incision_matrix.block<3, 1>(0, 3);
  const double euler = std::exp(1.0);

  //Trapezoidal
  double L0 = 0, L1 = 1;
  double V = 1.5;
  double tf = 1;
  double tb = (L0 - L1 + V * tf) / V;
  double a = V / tb;
  for (double time = 0.0; time < time_to_reach_target; time += publish_period)
  {
    double lamda_old = time / time_to_reach_target;
    Eigen::Vector3d linear_old = compute_linear_velocity(Xa, Xb, Xc, lamda_old, current_scope_tip_matrix, target_scope_tip_matrix) * 1 / time_to_reach_target;
    Eigen::Vector3d rot_old = compute_angular_velocity(Xa, Xb, Xc, lamda_old, current_scope_tip_matrix, target_scope_tip_matrix) * 1 / time_to_reach_target;

    //Trapezoidal
    double lamda = 0, dlamda = 0, gamma = 0, dgamma = 0;
    gamma = time / time_to_reach_target;
    dgamma = 1 / time_to_reach_target;

    if (gamma < tb)
    {
      lamda = L0 + (a * gamma * gamma) / 2.0;
      dlamda = a * gamma;
    }
    else if (gamma >= tb && gamma < tf - tb)
    {
      lamda = ((L1 + L0 - V * tf) / 2) + V * gamma;
      dlamda = V;
    }
    else if (gamma > tf - tb && gamma <= tf)
    {
      lamda = L1 - ((a * tf * tf) / 2) + (a * tf * gamma) - ((a / 2) * gamma * gamma);
      dlamda = a * (tf - gamma);
    }
    else
    {
      std::cout << "ERROR " << std::endl;
    }

    //Cubic
    // double gamma = time/time_to_reach_target;
    // double dgamma = 1/time_to_reach_target;
    // double lamda = 3*gamma*gamma - 2*gamma*gamma*gamma;
    // double dlamda = 6*gamma - 6*gamma*gamma;

    Eigen::Vector3d linear_trap = compute_linear_velocity(Xa, Xb, Xc, lamda, current_scope_tip_matrix, target_scope_tip_matrix) * dgamma * dlamda;
    Eigen::Vector3d rot_trap = compute_angular_velocity(Xa, Xb, Xc, lamda, current_scope_tip_matrix, target_scope_tip_matrix) * dgamma * dlamda;

    //Sigmoid
    // double k1 = 17;
    // double gamma = time / time_to_reach_target;
    // double dgamma = 1/time_to_reach_target;

    // double lamda = 1 / (1+pow(euler, -1 * k1 * (gamma - 0.5)));
    // double dlamda = (k1* pow(euler, -1*k1*(gamma-0.5)))/pow((1+pow(euler,-1*k1*(gamma-0.5))),2);

    // Eigen::Vector3d linear_sig = compute_linear_velocity(Xa, Xb, Xc, lamda, current_scope_tip_matrix, target_scope_tip_matrix) * dgamma * dlamda;
    // Eigen::Vector3d rot_sig = compute_angular_velocity(Xa, Xb, Xc, lamda, current_scope_tip_matrix, target_scope_tip_matrix) * dgamma * dlamda;

    //Send velocity commands to moveit servo
    twist->header.stamp = ros::Time::now();
    twist->twist.linear.x = linear_trap.x();
    twist->twist.linear.y = linear_trap.y();
    twist->twist.linear.z = linear_trap.z();
    twist->twist.angular.x = rot_trap.x();
    twist->twist.angular.y = rot_trap.y();
    twist->twist.angular.z = rot_trap.z();

    //Send velocity commands to moveit servo
    twist_old->header.stamp = ros::Time::now();
    twist_old->twist.linear.x = linear_old.x();
    twist_old->twist.linear.y = linear_old.y();
    twist_old->twist.linear.z = linear_old.z();
    twist_old->twist.angular.x = rot_old.x();
    twist_old->twist.angular.y = rot_old.y();
    twist_old->twist.angular.z = rot_old.z();

    velocity_test_pub.publish(twist_old);
    twist_stamped_pub.publish(twist);

    cmd_rate.sleep();
  }
  cmd_rate.sleep();

  //Send velocity command to stop the robot
  twist->header.stamp = ros::Time::now();
  twist->twist.linear.x = 0;
  twist->twist.linear.y = 0;
  twist->twist.linear.z = 0;
  twist->twist.angular.x = 0;
  twist->twist.angular.y = 0;
  twist->twist.angular.z = 0;
  //twist_stamped_pub.publish(twist);
  velocity_test_pub.publish(twist);
  cmd_rate.sleep();
}

void circle()
{
  Eigen::Isometry3d end_effector_start;
  servo->getEEFrameTransform(end_effector_start);
  double publish_period = 0.008;
  ros::Rate cmd_rate(1 / publish_period);
  double time_to_reach_target = 10;
  double radius = 0.07;
  int counter = 0;
  Eigen::Isometry3d mid_point;
  Eigen::Vector3d current_end_effector = end_effector_start.translation();
  Eigen::Vector3d center = current_end_effector - radius * Eigen::Vector3d(0, 0, 1);

  auto msg = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();
  for (double time = 0.0; time <= time_to_reach_target; time += publish_period, counter++)
  {
    double x_current = center.x() + radius * cos(time * 2 * M_PI / time_to_reach_target);
    double z_current = center.z() + radius * sin(time * 2 * M_PI / time_to_reach_target);

    double x_next = center.x() + radius * cos((time + 0.008) * 2 * M_PI / time_to_reach_target);
    double z_next = center.z() + radius * sin((time + 0.008) * 2 * M_PI / time_to_reach_target);

    msg->header.frame_id = "world";
    msg->header.stamp = ros::Time::now();
    // msg->twist.linear.x = x_next - x_current;
    // msg->twist.linear.y = 0;
    // msg->twist.linear.z = z_next - z_current;
    msg->twist.linear.x = radius * -sin((M_PI / 180) * time * 360 / time_to_reach_target) * ((M_PI / 180) * 360 / time_to_reach_target);
    msg->twist.linear.y = 0;
    msg->twist.linear.z = radius * cos((M_PI / 180) * time * 360 / time_to_reach_target) * ((M_PI / 180) * 360 / time_to_reach_target);

    Eigen::Vector3d velocity(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    std::cout << velocity.norm() << std::endl;
    twist_stamped_pub.publish(msg);
    cmd_rate.sleep();
  }
  msg->header.stamp = ros::Time::now();
  msg->twist.linear.x = 0;
  msg->twist.linear.y = 0;
  msg->twist.linear.z = 0;
  msg->twist.angular.x = 0;
  msg->twist.angular.y = 0;
  msg->twist.angular.z = 0;
  twist_stamped_pub.publish(msg);
}

void line()
{
  Eigen::Isometry3d end_effector_start;
  servo->getEEFrameTransform(end_effector_start);
  double publish_period = 0.008;
  ros::Rate cmd_rate(1 / publish_period);
  double time_to_reach_target = 20;
  double distance_to_move = 0.1;
  Eigen::Vector3d start = end_effector_start.translation();
  Eigen::Vector3d target = end_effector_start.translation() + Eigen::Vector3d(distance_to_move, 0, 0);
  int counter = 0;
  auto msg = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();

  //Eigen::Vector3d X_current = start;
  for (double time = 0.0; time <= time_to_reach_target; time += publish_period, counter++)
  {
    msg->header.frame_id = "world";
    msg->header.stamp = ros::Time::now();

    Eigen::Vector3d current = start + (target - start) * time / time_to_reach_target;
    Eigen::Vector3d next = start + (target - start) * (time + 0.008) / time_to_reach_target;
    Eigen::Vector3d delta = current - next;

    msg->twist.linear.x = delta.x();
    msg->twist.linear.y = delta.y();
    msg->twist.linear.z = delta.z();

    // msg->twist.linear.x = (target.x() - start.x())/time_to_reach_target;
    // msg->twist.linear.y = (target.y() - start.y() )/time_to_reach_target;
    // msg->twist.linear.z = (target.z() - start.z())/time_to_reach_target;
    twist_stamped_pub.publish(msg);
    cmd_rate.sleep();
    std::cout << "counter = " << counter << std::endl;
  }
}

void callback(const moveit_servo::ScopeTip &msg)
{
  thread_mutex.lock();
  //Stores the matrix for target scope tip
  target_scope_tip_matrix = Utilities::pose_to_matrix(msg.scope_tip_target);

  //Stores rcm matrix
  incision_matrix = Utilities::pose_to_matrix(msg.rcm);

  //Stores scope length
  scope_length = std::stod(msg.scope_length);
  thread_mutex.unlock();
}

double rampFunction(double distance)
{
  double velocity = 0, velocity_max = 0.005;
  double L1 = velocity_max, L0 = 0, d0 = 0.005;
  if (distance < d0)
    velocity = distance; //L0 + 3 * (L1 - L0) * (distance / d0) * (distance / d0) - 2 * (L1 - L0) * (distance / d0) * (distance / d0) * (distance / d0);
  else
    velocity = velocity_max;

  return velocity;
}

void write_to_csv(std::string filename, std::vector<double> current_distance, std::vector<double> current_velocity, std::vector<double> rec_velocity)
{
  std::ofstream file;
  file.open(filename);

  for (int i = 0; i < current_distance.size(); i++)
  {
    file << current_distance.at(i) << "," << current_velocity.at(i) << "," << rec_velocity.at(i) << "\n";
  }
  file.close();
  return;
}

double clamp(double n, double lower, double upper)
{
  return std::max(lower, std::min(n, upper));
}

Eigen::Vector3d SmoothDampVector(Eigen::Vector3d current, Eigen::Vector3d target, Eigen::Vector3d &currentVelocity, double smoothTime,
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

double Repeat(double t, double length){
  return clamp(t -std::floor(t / length) * length, 0.0, length);
}

double DeltaAngle(double current, double target){
  double delta = Repeat((target - current), 360.0);
  if (delta > 180.0)
    delta -= 360.0;
  return delta;
}

double SmoothDamp(double current, double target , double &currentVelocity, double smoothTime,
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

double SmoothDampAngle(double current, double target,  double &currentVelocity, double smoothTime,  double maxSpeed,  double deltaTime){
  target = current + DeltaAngle(current, target);
  return SmoothDamp(current, target, currentVelocity, smoothTime, maxSpeed, deltaTime);
}

void movement_damp_thread()
{
  Eigen::Isometry3d end_effector;
  servo->getEEFrameTransform(end_effector);
  Eigen::Matrix4d current_scope_tip_matrix = getScopeTip(end_effector.matrix());
  target_scope_tip_matrix = current_scope_tip_matrix;

  double publish_period = 0.024;
  ros::Rate cmd_rate(1 / publish_period);
  auto twist = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();
  twist->header.frame_id = "world";
  twist->header.stamp = ros::Time::now();
  twist->twist.linear.x = 0;
  twist->twist.linear.y = 0;
  twist->twist.linear.z = 0;
  twist->twist.angular.x = 0;
  twist->twist.angular.y = 0;
  twist->twist.angular.z = 0;
  Eigen::Vector3d Xe = end_effector.matrix().block<3, 1>(0, 3);
  Eigen::Vector3d Xa = current_scope_tip_matrix.block<3, 1>(0, 3);
  Eigen::Vector3d Xb, Xc, Xa_, Xc_, correction_velocity;

  double velocity_max = 0.03;
  Eigen::Vector3d current_velocity;
  double eulerXVelocity = 0.0, eulerYVelocity = 0.0, eulerZVelocity = 0.0;
  std::cout << "current x, current y, current z, target x, target y, target z, xVel, yVel, zVel, xPos, yPos, zPos" << std::endl;
  int counter;
  while (true)
  {
    //Get the current end effector matrix
    servo->getEEFrameTransform(end_effector);

    //Get the scope tip matrix
    current_scope_tip_matrix = getScopeTip(end_effector.matrix());

    //Set Xa as the current scope tip matrix
    Eigen::Vector3d Xa = current_scope_tip_matrix.block<3, 1>(0, 3);

    //Get End effector position
    Xe = end_effector.matrix().block<3, 1>(0, 3);

    //Set Xb as the target position and Xc as the incision matrix
    thread_mutex.lock();
    Xb = target_scope_tip_matrix.block<3, 1>(0, 3);
    Xc = incision_matrix.block<3, 1>(0, 3);
    thread_mutex.unlock();

    //If distance between current scope tip and target is less than 0.00005m stop the robot and rcm distance difference < 0.00001
    if ((Xa - Xb).norm() <= 0.0001)
    {
      twist->header.stamp = ros::Time::now();
      twist->twist.linear.x = 0;
      twist->twist.linear.y = 0;
      twist->twist.linear.z = 0;
      twist->twist.angular.x = 0;
      twist->twist.angular.y = 0;
      twist->twist.angular.z = 0;
      twist_stamped_pub.publish(twist);
      cmd_rate.sleep();
      continue;
    }

    Xc_ = Xe + ((Xc - Xe).dot(end_effector.matrix().block<3, 1>(0, 2))) * end_effector.matrix().block<3, 1>(0, 2);
    correction_velocity = ((Xc - Xc_) / publish_period) * velocity_max;

    Eigen::Vector3d current_position = SmoothDampVector(Xa, Xb, current_velocity, 1, velocity_max, publish_period);

    // //Calculate the linear and rotational velocities
    double lamda = (current_velocity.norm() * publish_period) / (Xa - Xb).norm();
    Eigen::Vector3d linear = compute_linear_velocity(Xa, Xb, Xc, lamda, current_scope_tip_matrix, target_scope_tip_matrix) * current_velocity.norm()/ (Xa - Xb).norm();
    Eigen::Vector3d rot = compute_angular_velocity(Xa, Xb, Xc, lamda, current_scope_tip_matrix, target_scope_tip_matrix) * current_velocity.norm() / ((Xa - Xb).norm());
    linear = correction_velocity + linear;

    // //Send velocity commands to moveit servo
    twist->header.stamp = ros::Time::now();
    twist->twist.linear.x = linear.x();
    twist->twist.linear.y = linear.y();
    twist->twist.linear.z = linear.z();
    twist->twist.angular.x = rot.x();
    twist->twist.angular.y = rot.y();
    twist->twist.angular.z = rot.z();
    twist_stamped_pub.publish(twist);
    cmd_rate.sleep();
  }
}

void movement_thread2()
{
  Eigen::Isometry3d end_effector;
  servo->getEEFrameTransform(end_effector);
  Eigen::Matrix4d current_scope_tip_matrix = getScopeTip(end_effector.matrix());
  target_scope_tip_matrix = current_scope_tip_matrix;

  double publish_period = 0.024;
  ros::Rate cmd_rate(1 / publish_period);
  auto twist = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();
  twist->header.frame_id = "world";
  twist->header.stamp = ros::Time::now();
  twist->twist.linear.x = 0;
  twist->twist.linear.y = 0;
  twist->twist.linear.z = 0;
  twist->twist.angular.x = 0;
  twist->twist.angular.y = 0;
  twist->twist.angular.z = 0;
  Eigen::Vector3d Xe = end_effector.matrix().block<3, 1>(0, 3);
  Eigen::Vector3d Xa = current_scope_tip_matrix.block<3, 1>(0, 3);
  Eigen::Vector3d Xb, Xc, Xa_, Xc_, correction_velocity;

  double velocity_previous = 0.0, distance_previous = 0.0, velocity_max = 0.03;
  double counter = 0;
  std::cout << "distance_current,velocity_current,velocity_recommended,Xa - Xb" << std::endl;
  while (true)
  {
    // if(counter >= 1000)
    //   break;

    //Get the current end effector matrix
    servo->getEEFrameTransform(end_effector);

    //Get the scope tip matrix
    current_scope_tip_matrix = getScopeTip(end_effector.matrix());

    //Set Xa as the current scope tip matrix
    Eigen::Vector3d Xa = current_scope_tip_matrix.block<3, 1>(0, 3);

    //Get End effector position
    Xe = end_effector.matrix().block<3, 1>(0, 3);

    //Set Xb as the target position and Xc as the incision matrix
    thread_mutex.lock();
    Xb = target_scope_tip_matrix.block<3, 1>(0, 3);
    Xc = incision_matrix.block<3, 1>(0, 3);
    thread_mutex.unlock();

    //If distance between current scope tip and target is less than 0.00005m stop the robot and rcm distance difference < 0.00001
    if ((Xa - Xb).norm() <= 0.0005)
    {
      twist->header.stamp = ros::Time::now();
      twist->twist.linear.x = 0;
      twist->twist.linear.y = 0;
      twist->twist.linear.z = 0;
      twist->twist.angular.x = 0;
      twist->twist.angular.y = 0;
      twist->twist.angular.z = 0;
      twist_stamped_pub.publish(twist);
      cmd_rate.sleep();
      counter = 0;
      continue;
    }

    Xc_ = Xe + ((Xc - Xe).dot(end_effector.matrix().block<3, 1>(0, 2))) * end_effector.matrix().block<3, 1>(0, 2);
    correction_velocity = ((Xc - Xc_) / publish_period) * velocity_max;

    double velocity_recommended = rampFunction((Xa - Xb).norm());
    int rampDirection = 0;
    if (velocity_previous < velocity_recommended)
      rampDirection = +1;
    else if (velocity_previous > velocity_recommended)
      rampDirection = -1;
    else
      rampDirection = 0;

    // Logic to start the movement
    if (distance_previous < 0.00005 && rampDirection == +1)
      velocity_previous = 0.1;

    // Logic to stop the movement
    if (distance_previous < 0.00005 && rampDirection == -1)
    {
      velocity_previous = 0;
      distance_previous = 0;
    }

    double distance_current = distance_previous + rampDirection * velocity_previous * publish_period;
    double velocity_current = rampFunction(distance_current);

    //Calculate the linear and rotational velocities
    double lamda = (velocity_current * publish_period) / (Xa - Xb).norm();
    Eigen::Vector3d linear = compute_linear_velocity(Xa, Xb, Xc, lamda, current_scope_tip_matrix, target_scope_tip_matrix) * velocity_current / ((Xa - Xb).norm());
    Eigen::Vector3d rot = compute_angular_velocity(Xa, Xb, Xc, lamda, current_scope_tip_matrix, target_scope_tip_matrix) * velocity_current / ((Xa - Xb).norm());

    //linear = correction_velocity + linear;

    //Xa = Xa + lamda*(Xb-Xa);

    //Send velocity commands to moveit servo
    twist->header.stamp = ros::Time::now();
    twist->twist.linear.x = linear.x();
    twist->twist.linear.y = linear.y();
    twist->twist.linear.z = linear.z();
    twist->twist.angular.x = rot.x();
    twist->twist.angular.y = rot.y();
    twist->twist.angular.z = rot.z();
    twist_stamped_pub.publish(twist);
    cmd_rate.sleep();

    Xa_ = Xa + lamda * (Xb - Xa);
    velocity_previous = velocity_current;
    distance_previous = distance_current;
    counter += 1;
  }
}

void cylinder_follow_thread()
{
  Eigen::Isometry3d end_effector;
  servo->getEEFrameTransform(end_effector);
  Eigen::Matrix4d current_end_effector_matrix = end_effector.matrix();

  Eigen::Matrix4d current_scope_tip_matrix = getScopeTip(end_effector.matrix());
  target_scope_tip_matrix = current_scope_tip_matrix;

  double publish_period = 0.016;
  ros::Rate cmd_rate(1 / publish_period);
  auto twist = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();
  twist->header.frame_id = "world";
  twist->header.stamp = ros::Time::now();
  twist->twist.linear.x  = 0;
  twist->twist.linear.y  = 0;
  twist->twist.linear.z  = 0;
  twist->twist.angular.x = 0;
  twist->twist.angular.y = 0;
  twist->twist.angular.z = 0;
  Eigen::Vector3d Xe = end_effector.matrix().block<3, 1>(0, 3);
  Eigen::Vector3d Xa = current_scope_tip_matrix.block<3, 1>(0, 3);
  Eigen::Vector3d Xb, Xc;

  double max_linear_velocity = 0.03, max_angular_velocity = 10, smoothTime = 1;
  double eulerXVelocity =0.0, eulerYVelocity = 0.0, eulerZVelocity = 0.0;
  Eigen::Vector3d current_linear_velocity;
  while (ros::ok())
  {
    //Get the current end effector matrix
    servo->getEEFrameTransform(end_effector);
    current_end_effector_matrix = end_effector.matrix();

    //Get the scope tip matrix
    current_scope_tip_matrix = getScopeTip(current_end_effector_matrix);
    //current_scope_tip_matrix = Utilities::pose_to_matrix(model_state.get_model_position_world("ScopeCylinder2"));

    //Set Xa as the current scope tip matrix
    Eigen::Vector3d Xa = current_scope_tip_matrix.block<3, 1>(0, 3);

    //Get End effector position
    Xe = end_effector.matrix().block<3, 1>(0, 3);

    //Set Xb as the target position and Xc as the incision matrix
    thread_mutex.lock();
    Xb = target_scope_tip_matrix.block<3, 1>(0, 3);
    Xc = incision_matrix.block<3, 1>(0, 3);
    thread_mutex.unlock();

    //Get the damp position
    Eigen::Vector3d target_position = SmoothDampVector(Xa, Xb, current_linear_velocity, smoothTime, max_linear_velocity, publish_period);

    //Calculate orientation
    geometry_msgs::Pose current_scope_tip_pose = Utilities::matrix_to_pose(current_scope_tip_matrix);
    tf2::Quaternion current_scope_tip_quat(current_scope_tip_pose.orientation.x, current_scope_tip_pose.orientation.y, current_scope_tip_pose.orientation.z, current_scope_tip_pose.orientation.w);
    double current_x, current_y, current_z;
    tf2::Matrix3x3(current_scope_tip_quat).getRPY(current_x, current_y, current_z);

    geometry_msgs::Pose target_scope_tip_pose = Utilities::matrix_to_pose(target_scope_tip_matrix);
    tf2::Quaternion target_scope_tip_quat(target_scope_tip_pose.orientation.x, target_scope_tip_pose.orientation.y, target_scope_tip_pose.orientation.z, target_scope_tip_pose.orientation.w);
    Eigen::Vector3d target_euler;
    double target_x, target_y, target_z;
    tf2::Matrix3x3(target_scope_tip_quat).getRPY(target_x, target_y, target_z);

    //Calculate Angular velocities and positions
    double x_cur = SmoothDampAngle(current_x, target_x,  eulerXVelocity, smoothTime,  max_angular_velocity,  publish_period);
    double y_cur = SmoothDampAngle(current_y, target_y,  eulerYVelocity, smoothTime,  max_angular_velocity,  publish_period);
    double z_cur = SmoothDampAngle(current_z, target_z,  eulerZVelocity, smoothTime,  max_angular_velocity,  publish_period);

    //Create target scope tip pose
    tf2::Quaternion quat_damp;
    quat_damp.setEulerZYX(z_cur, y_cur, x_cur);
    geometry_msgs::Pose damp_pose;
    damp_pose.orientation.x = quat_damp.x();
    damp_pose.orientation.y = quat_damp.y();
    damp_pose.orientation.z = quat_damp.z();
    damp_pose.orientation.w = quat_damp.w();
    damp_pose.position.x = target_position.x();
    damp_pose.position.y = target_position.y();
    damp_pose.position.z = target_position.z();
   // model_state.set_model_position_world("ScopeCylinder2", damp_pose); 
    Eigen::Matrix4d damp_frame = Utilities::pose_to_matrix(damp_pose);

    // Eigen::Quaterniond current_quat(current_end_effector_matrix.block<3, 3>(0, 0));
    // Eigen::Quaterniond target_quat(end_frame.block<3, 3>(0, 0));
    // Eigen::AngleAxisd rot_axis((target_quat*current_quat.inverse()).normalized());
    // Eigen::Vector3d angularDisplacement = rot_axis.axis() * rot_axis.angle();    

    //Calculate Linear Velocity
    Eigen::Vector3d Xp = target_position;
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
    Eigen::Vector3d Xend_effector = Zcylinder.cross(Zend_effector).normalized();
    Eigen::Vector3d Yend_effector = (Zend_effector.cross(Xend_effector)).normalized();
    Eigen::Matrix4d end_effector_frame;
    end_effector_frame.setIdentity();
    end_effector_frame.block<3,1>(0,0) = Xend_effector;
    end_effector_frame.block<3,1>(0,1) = Yend_effector;
    end_effector_frame.block<3,1>(0,2) = Zend_effector;
    end_effector_frame.block<3,1>(0,3) = target_position - 0.37 * (target_position - Xc).normalized();

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

    //Send velocity commands to moveit servo
    twist->header.stamp = ros::Time::now();
    twist->twist.linear.x = end_effector_linear_velocity.x();
    twist->twist.linear.y = end_effector_linear_velocity.y();
    twist->twist.linear.z = end_effector_linear_velocity.z();
    twist->twist.angular.x = end_effector_angular_velocity.x();
    twist->twist.angular.y = end_effector_angular_velocity.y();
    twist->twist.angular.z = end_effector_angular_velocity.z();

   //If distance between current scope tip and target is less than 0.00005m stop the robot and rcm distance difference < 0.00001
    if ((Xa - Xb).norm() <= 0.000001 
      && (std::abs(current_x-target_x)<0.0001) && (std::abs(current_y-target_y<0.0001)) && (std::abs(current_z-target_z<0.0001)))
    {
      twist->header.stamp = ros::Time::now();
      twist->twist.linear.x = 0;
      twist->twist.linear.y = 0;
      twist->twist.linear.z = 0;
      twist->twist.angular.x = 0;
      twist->twist.angular.y = 0;
      twist->twist.angular.z = 0;
      twist_stamped_pub.publish(twist);
      cmd_rate.sleep();
      continue;
    }

    twist_stamped_pub.publish(twist);
    cmd_rate.sleep();
  }
}

void test_thread(){
  double publish_period = 0.008;
  ros::Rate cmd_rate(1 / publish_period);
  geometry_msgs::TwistStamped twist;
  twist.header.frame_id = "SCOPE";
  while (ros::ok())
  {    
    twist.header.stamp = ros::Time::now();
    twist.twist.linear.x = 0;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = 0;
    twist.twist.angular.x = 0;
    twist.twist.angular.y = 0;
    twist.twist.angular.z = 1;
    twist_stamped_pub.publish(twist);
    cmd_rate.sleep();
  }
}

/**
 * Instantiate the C++ servo node interface.
 * Send some Cartesian commands, then some joint commands.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, LOGNAME);
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(100);
  spinner.start();

  rviz = new RvizMarker(nh);
  //model_state.initialize(&nh);

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();

  // Run the servo node C++ interface in a new timer to ensure a constant outgoing message rate.
  servo = new moveit_servo::Servo(nh, planning_scene_monitor);
  servo->start();

  // Subscribe  to servo status (and log it when it changes)
  StatusMonitor status_monitor(nh, servo->getParameters().status_topic);

  // Create publishers to send servo commands
  twist_stamped_pub = nh.advertise<geometry_msgs::TwistStamped>(servo->getParameters().cartesian_command_in_topic, 1);
  velocity_test_pub = nh.advertise<geometry_msgs::TwistStamped>("/test_velocity", 1);

  auto joint_servo_pub = nh.advertise<control_msgs::JointJog>(servo->getParameters().joint_command_in_topic, 1);

  ros::Subscriber sub = nh.subscribe("/scope_tip", 1, callback);

  std::thread t1(test_thread);//ylinder_follow_thread //movement_damp_thread
  t1.join();
  ros::waitForShutdown();

  servo->setPaused(true);
  return 0;
}
