#ifndef RP_HPP_
#define RP_HPP_

#define SCREEN_PRINT_RATE	0.5
#define PI 3.14159

// ROS includes
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Path.h"

// custom messages
#include "acl_system/ViconState.h"
#include "acl_system/QuadCmd.h"

// Global includes
#include <stdio.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <queue>

class REACT
{
public:
	REACT();

	ros::Publisher partitioned_scan_pub, pub_clean_scan, goal_pub, new_goal_pub, int_goal_pub, last_goal_pub, corridor_scan_pub;

	void scanCB(const sensor_msgs::LaserScan& msg);
	void stateCB(const acl_system::ViconState& msg);
	void partition_scan(const sensor_msgs::LaserScan& msg);
	void vis_better_scan(const sensor_msgs::LaserScan& msg);
	void check_goal(const sensor_msgs::LaserScan& msg);

	// ROS timed functions
	void sendGoal(const ros::TimerEvent&);

private:

	double inf; // Infinity definition
	double thresh, yaw, dist_2_goal, angle_2_goal, angle_check, msg_received, cost, cost_i, angle_diff, safe_distance, min_cost, buffer;
	double num_samples, angle_max, angle_min, angle_increment;
	double angle_diff_last, angle_seg_inc;
	bool debug, can_reach_goal, corridor_free;
	int down_sample, num_of_partitions, goal_index, collision_counter, collision_counter_corridor, num_of_points;

	std::ostringstream errorMsg, warnMsg;

	tf::Vector3 pose, next_goal_v, last_goal_v;
	geometry_msgs::PointStamped goal, new_goal, last_goal;
	sensor_msgs::LaserScan filtered_scan;
	// geometry_msgs::PoseArray goal_points;
	nav_msgs::Path goal_points;

	//## Logging and Debugging Functions
	void screenPrint();
	void find_inter_goal();
	
};

#endif /* RP_HPP_ */