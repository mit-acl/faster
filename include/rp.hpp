#ifndef RP_HPP_
#define RP_HPP_

#define SCREEN_PRINT_RATE	0.5

// ROS includes
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "tf/transform_datatypes.h"

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

	ros::Publisher partitioned_scan_pub, goal_pub;

	void scanCB(const sensor_msgs::LaserScan& msg);
	void stateCB(const acl_system::ViconState& msg);
	void partition_scan(const sensor_msgs::LaserScan& msg);

	// ROS timed functions
	void sendGoal(const ros::TimerEvent&);

private:

	double inf; // Infinity definition
	double thresh ;

	std::ostringstream errorMsg, warnMsg;

	tf::Vector3 pose;
	tf::Quaternion att;
	geometry_msgs::PointStamped goal;
 	sensor_msgs::LaserScan partitioned_scan;

	//## Logging and Debugging Functions
	void screenPrint();
	void find_free_space();
	void paritionScan();
	
};

#endif /* RP_HPP_ */