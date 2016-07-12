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
#include "acl_system/QuadGoal.h"
#include "acl_system/QuadState.h"
#include "acl_system/QuadFlightEvent.h"

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

	ros::Publisher partitioned_scan_pub, pub_clean_scan, goal_pub, new_goal_pub, int_goal_pub, last_goal_pub, corridor_scan_pub, quad_goal_pub;;

	void scanCB(const sensor_msgs::LaserScan& msg);
	void stateCB(const acl_system::ViconState& msg);
	void partition_scan(const sensor_msgs::LaserScan& msg);
	void vis_better_scan(const sensor_msgs::LaserScan& msg);
	void check_goal(const sensor_msgs::LaserScan& msg);

	void eventCB(const acl_system::QuadFlightEvent& msg);

	// ROS timed functions
	void sendGoal(const ros::TimerEvent&);

private:

	double inf_; // Infinity definition
	double thresh_, yaw_, dist_2_goal_, angle_2_goal_, angle_check_, msg_received_, cost_, cost_i_, angle_diff_, safe_distance_, min_cost_, buffer_;
	double num_samples_, angle_max_, angle_min_, angle_increment_;
	double angle_diff_last_, angle_seg_inc_;
	bool debug_, can_reach_goal_, corridor_free_;
	int down_sample_, num_of_partitions_, goal_index_, collision_counter_, collision_counter_corridor_;

	double spinup_time_;
	int quad_status_;
	acl_system::QuadState state_;
	acl_system::QuadFlightEvent quad_event_;


	std::ostringstream errorMsg, warnMsg;

	tf::Vector3 pose_, next_goal_v_, last_goal_v_;
	geometry_msgs::PointStamped goal_, new_goal_, last_goal_;
	sensor_msgs::LaserScan filtered_scan_;
	// geometry_msgs::PoseArray goal_points;
	nav_msgs::Path goal_points_;

	acl_system::QuadGoal quad_goal_;

	//## Logging and Debugging Functions
	void screenPrint();
	void find_inter_goal();
	void saturate(double &var, double min, double max);
	void takeoff();
	void land();
	
};

#endif /* RP_HPP_ */