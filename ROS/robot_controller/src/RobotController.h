#pragma once
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include "kdl_conversions/kdl_msg.h"
#include "tf_conversions/tf_kdl.h"
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "GazeboMarker.h"
#include "GazeboModelState.h"
#include "Semaphore.h"
#include "PathComputation.h"
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include "control_msgs/JointJog.h"
#include <cmath>
#include<TrigonometricOTG.h>

class RobotController {
public:
    /**
    * Constructor
    * Creates RobotController object
    */
    RobotController();

    /**
    * Destructor
    * Creates RobotController object
    */
    virtual ~RobotController();

    /**
    * Moves the cylinder to currentCylinderPose + deltaPose
    * @param refTargetPose Delta position the needs to be applied to the cylinder
    */
    virtual void set_delta_pose(geometry_msgs::Pose delta_pose);

    /**
    * Moves the robot through the defined way points
    */
    virtual void move_robot(){}

    /**
    * Initializes RobotController object
    */
    virtual bool initialize(int argc, char **argv);

    /**
    * Resets the cylinder position and orientation to match scope tip
    */
    virtual void reset_cylinder() = 0;

    ros::NodeHandle* getNodeHandle(){
        return _node_handle;
    }

    GazeboModelState* get_model_state(){
        return &_model_state;
    }

    void writeJointStatesToFile();

protected:
    ros::Publisher _twist_stamped_pub;
    ros::Publisher _twist_stamped_joint_pub;
    /**
    * RObot chain for kdl solver
    */
    KDL::Chain _my_chain;
    /**
    *   joint names
    */
    std::vector<string> _joint_name;
    /**
    *   kdl forward kinematic solver
    */
    KDL::ChainFkSolverPos_recursive* _fk_solver;
    /**
    /**
    *   kdl forward kinematic solver
    */
    KDL::ChainIkSolverVel_pinv* _ik_solver_pinv;

    KDL::ChainIkSolverPos_NR* _ik_solver;
    /**
    /**
    *   joint states
    */
    KDL::JntArrayVel _joint_state;
    bool _Cartesian_compute;
    /**
    /* To record joint states*/
    ros::Subscriber  _joint_state_sub;

    /**
    * Joint state subscirber callback
    */
    void _Joint_state_cb(const sensor_msgs::JointStateConstPtr& msg);

    void jointCallback(const sensor_msgs::JointState& state);
    std::stringstream joint_stream;

    std::mutex path_thread;

    /**
    * Pointer to NodeHandle Object
    */
    ros::NodeHandle *_node_handle;

    /**
    * Pointer to move group object
    */
    moveit::planning_interface::MoveGroupInterface *_move_group_ptr;

    /**
    * Path Computation Object
    */
    PathComputation *_compute_path;

    /**
    * GazeboMarker object to create markers for rendering in gazebo environment
    */
    GazeboMarker _marker;

    /**
    * Set to true to enable markers, false otherwise
    */
    bool _marker_enabled;

    /**
    * GazeboModelState object to set and receive model information from gazebo environment
    */
    GazeboModelState _model_state;

    /**
    * Set to true to enable model state, false otherwise
    */
    bool _model_state_enabled;

    /**
    * Holds the reference frame
    */
    string _base_frame;

    /**
    * Pointer to AsyncSpinner to run the main thread in background
    */
    ros::AsyncSpinner *_async_spinner;

    /**
    *   Pointer to loop rate
    */
    ros::Rate *_loop_rate_ptr;

    /**
    * Holds the list of waypoints through which the robot endeffector moves, returned by _compute_path object
    */
    std::vector<geometry_msgs::Pose> _end_effector_pose_list;

    /**
    * Holds the delta pose that is applied to the cylinder
    */
    geometry_msgs::Pose _delta_pose;

    /**
    * Stores the target matrix
    */
    Eigen::Matrix4d _target_matrix;

    /**
    * Thread which computes the path and moves the robot
    */
    std::thread _robot_movement_thread;

    std::thread _path_computation_thread;
    std::thread _joint_computation_thread;
    /**
    * Points to semaphore object which notifies if cylinder position is updated
    */
    Semaphore *_new_position_available_sem;

    /**
    * Ros Publisher object to send the trajectory information to the trajectory topic
    */
    ros::Publisher _joint_pub;

    /**
    * Initializes the gazebo models
    */
    virtual void _initialize_gazebo_models();

    /**
    * Moves the robot to the initial named position specified in the moveit setup file
    */
    void _move_to_initial_position();
    void _kdl_initialize();
    /**
    * Updates the cylinder position to the deltapose value
    */
    void _update_cylinder();

    /**
    * Thread function called by _robot_movement_thread to perform compute the path and move the robot
    */
    void _robot_movement_thread_func();

    void _path_computation_thread_func();
    void _joint_computation_thread_func();
   /**
   * Moves the robot through a list of specified waypoints moveit trajectory planner
   * @param wayPoints List of waypoints
   * @returns True if successful, false otherwise
   */
    bool virtual _move_robot_to_pose(vector<geometry_msgs::Pose> wayPoints){};
};
