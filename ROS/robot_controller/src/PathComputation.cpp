

#include "PathComputation.h"

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Constructor
*/
PathComputation::PathComputation()
{

}

/*
	IN 			: None
	OUT 		: None
	DESCRIPTION	: Destructor
*/
PathComputation::~PathComputation()
{

}

/*
	IN 			: P, theta, p1, p2
	OUT 		: Rotated vector
	DESCRIPTION	: Rotates a point p along an angle theta
*/
Eigen::Vector3d PathComputation::rotate(Eigen::Vector3d p, double theta, Eigen::Vector3d p1, Eigen::Vector3d p2){
    Eigen::Vector3d s = (p2 - p1)/(p2-p1).norm();
    double x, y, z, a, b, c, u, v, w;
    x = p.x(); y = p.y(); z = p.z();
    a = p1.x(); b = p1.y(); c = p1.z();
    u = s.x(); v = s.y(); w = s.z();

    Eigen::Vector3d point(((a*(v*v + w*w) - u*(b*v + c*w - u*x - v*y - w*z))*(1 - cos(theta)) + x*cos(theta) + (-c*v + b*w - w*y + v*z)*sin(theta)),
                          ((b*(u*u + w*w) - v*(a*u + c*w - u*x - v*y - w*z))*(1 - cos(theta)) + y*cos(theta) + (c*u - a*w + w*x - u*z)*sin(theta)),
                          ((c*(u*u + v*v) - w*(a*u + b*v - u*x - v*y - w*z))*(1 - cos(theta)) + z*cos(theta) + (-b*u + a*v - v*x + u*y)*sin(theta))
    );

    return point; //=new value for x axis, normalize and set it equal to new x axis
    //Y = Z x X
}