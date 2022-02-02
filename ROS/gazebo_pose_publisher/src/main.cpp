#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_pose_publisher/custom_pose.h>
#include <iostream>
#include <vector>
ros::Publisher pub;
bool airborne;

int seq = 0;

// Forces callback function
void poseCb(ConstPosesStampedPtr &posesStamped){
    //std::cout << posesStamped->DebugString();

    long sec = posesStamped->time().sec();
    long nsec = posesStamped->time().nsec();

    gazebo_pose_publisher::custom_pose message;

    std::vector<geometry_msgs::Pose> pose_list;
    std::vector<std::string> name_list;
    std_msgs::Header header;
    header.seq = seq;
    header.stamp.sec = posesStamped->time().sec();
    header.stamp.nsec = posesStamped->time().nsec();
    header.frame_id = "frame";

    for (int i =0; i < posesStamped->pose_size(); ++i){
        const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);

        name_list.push_back(pose.name());
        geometry_msgs::Pose p;
        p.position.x = pose.position().x();
        p.position.y = pose.position().y();
        p.position.z = pose.position().z();

        p.orientation.x = pose.orientation().x();
        p.orientation.y = pose.orientation().y();
        p.orientation.z = pose.orientation().z();
        p.orientation.w = pose.orientation().w();

        pose_list.push_back(p);
    }
    seq += 1;

    message.header = header;
    message.names = name_list;
    message.poses = pose_list;

    pub.publish(message);
}

int main(int _argc, char **_argv){

    // Load Gazebo & ROS
    gazebo::client::setup(_argc, _argv);
    ros::init(_argc, _argv, "pose_publisher");

    // Create Gazebo node and init
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Create ROS node and init
    ros::NodeHandle n;
    pub = n.advertise<gazebo_pose_publisher::custom_pose>("/custom/poses", 1000);

    // Listen to Gazebo contacts topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/pose/info", poseCb);

    // Busy wait loop...replace with your own code as needed.
    // Busy wait loop...replace with your own code as needed.
    while (true)
    {
        gazebo::common::Time::MSleep(20);

        // Spin ROS (needed for publisher) // (nope its actually for subscribers-calling callbacks ;-) )
        ros::spinOnce();
    }
    gazebo::client::shutdown();
}
