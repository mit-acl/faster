#ifndef PI
#define PI 3.14159265359 ///< The number PI
#endif

#define SCREEN_PRINT_RATE	0.5

#ifndef GPF_HPP_
#define GPF_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

// custom messages
#include "acl_system/ViconState.h"

#include <stdio.h>
#include <math.h>
#include <vector>
#include <algorithm>

class FilterGP
{
public:
	FilterGP();

	ros::Publisher filtered_scan_pub;

	void scanCB(const sensor_msgs::LaserScan& msg);
	void stateCB(const acl_system::ViconState& msg);
	

private:

	double max_range, alt;
	double ang_min, ang_max;
	double min_idx, max_idx;
	double increment, num_samples;
	double inf;
	double points_removed;
	double nx, ny, nz;
	double angle1, angle2;

	tf::Quaternion att;

	std::ostringstream errorMsg, warnMsg;

	//## Logging and Debugging Functions
	void screenPrint();

	void findAngles(tf::Quaternion q, double &min_idx, double &max_idx);

	double rangeCheck(int idx);
	
};

#endif /* GPF_HPP_ */