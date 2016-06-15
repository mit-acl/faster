#ifndef PI
#define PI 3.14159265359 ///< The number PI
#endif

#define SCREEN_PRINT_RATE	0.5

#ifndef GPF_HPP_
#define GPF_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/Path.h"

// custom messages
#include "acl_system/ViconState.h"

#include <stdio.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <numeric>

class FilterGP
{
public:
	FilterGP();

	ros::Publisher filtered_scan_pub, point_array_pub;

	void scanCB(const sensor_msgs::LaserScan& msg);
	void stateCB(const acl_system::ViconState& msg);
	

private:

	double max_range, alt;
	double ang_min, ang_max;
	double increment, num_samples;
	double inf;
	double points_removed;
	double x, y, z;
	double ground_range, filter_thresh;

	nav_msgs::Path scan_points;
	tf::Vector3 pose;

	tf::Quaternion q;

	std::ostringstream errorMsg, warnMsg;

	std::vector<double> new_goal_vec;
	std::vector<double> last_goal_vec;

	//## Logging and Debugging Functions
	void screenPrint();
	
};

#endif /* GPF_HPP_ */