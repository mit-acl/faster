#ifndef RP_HPP_
#define RP_HPP_

#define SCREEN_PRINT_RATE	0.5

// ROS includes
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

// custom messages
#include "acl_system/ViconState.h"
#include "acl_system/QuadCmd.h"

// Global includes
#include <stdio.h>
#include <math.h>
#include <vector>
#include <algorithm>

class REACT
{
public:
	REACT();

	ros::Publisher filtered_scan_pub;

	void scanCB(const sensor_msgs::LaserScan& msg);
	void stateCB(const acl_system::ViconState& msg);

private:

	double inf;

	std::ostringstream errorMsg, warnMsg;

	tf::Vector3 pose;
	tf::Quaternion att;

	//## Logging and Debugging Functions
	void screenPrint();

	void paritionScan();
	
};

#endif /* RP_HPP_ */