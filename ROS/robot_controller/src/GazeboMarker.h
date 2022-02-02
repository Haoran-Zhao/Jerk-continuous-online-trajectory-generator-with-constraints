#pragma once

#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <iostream>

using namespace std;

/**
* Responsible for drawing and clearing markers (Spherical, line) in Gazebo
*/
class GazeboMarker {
public:
    /**
	 * Constructor
	 * Creates a GazeboMarker object
	 */
    GazeboMarker();

    /**
	 * Destructor
	 * Called when GazeboMarker object get destroyed
	 */
    ~GazeboMarker();

    /**
	*	Renders a sphere marker at the specified location
	*	@param pose Location where the marker should be placed
	*/
    void draw_sphere_marker(geometry_msgs::Pose pose);

    /**
     *	Renders a line segment marker between the specified two points
     *	@param start_pose Starting point of the line segment marker
     *	@param end_pose Ending point of the line segment marker
     *	@param clear set to true to clear all previous marker, false otherwise
     */
    void draw_line_marker(geometry_msgs::Pose start_pose, geometry_msgs::Pose end_pose, bool clear);

    /**
    *	Renders a line segment marker between the specified two points
    *	@param start_vect Starting point of the line segment marker
    *	@param end_vect Ending point of the line segment marker
    *	@param clear set to true to clear all previous marker, false otherwise
    */
    void draw_line_marker(Eigen::Vector3d start_vect, Eigen::Vector3d end_vect, bool clear);

    /**
    *	Clears all the markers that are rendered in Gazebo
    */
    void erase_markers();

private:

    /**
    *	Object that sends marker messages to gazebo
    */
    ignition::transport::Node _node;

    /**
    *	Defines marker object
    */
    ignition::msgs::Marker _marker_msg;

    /**
    *	Defines market type
    */
    ignition::msgs::Material *_mat_msg;

    /**
    *	Enum to simplify definition of marker type
    */
    enum MessageID {
        NONE = -1,
        SPHERE = 0,
        LINE = 1
    };
};