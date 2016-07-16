#include "rp.hpp"


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "reactive_planner", ros::init_options::NoSigintHandler);

	ros::NodeHandle n("~");

	REACT rp;

	// initialize listener callback for state estimate
	ros::Subscriber vic_pose = n.subscribe("vicon", 1, &REACT::stateCB, &rp);
	
	// initialize listener callback for laser scan
	ros::Subscriber sub_scan = n.subscribe("scan", 1, &REACT::scanCB, &rp);

	// initialize listener callback for event
	ros::Subscriber event_sub = n.subscribe("event", 1, &REACT::eventCB, &rp);

	// initialize listener callback for global goal
	ros::Subscriber global_goal_sub = n.subscribe("global_goal", 1, &REACT::global_goalCB, &rp);
	

	// SendCmd timer
	ros::Timer sendGoalTimer = n.createTimer(ros::Duration(0.01), &REACT::sendGoal, &rp);


	rp.pub_clean_scan = n.advertise<sensor_msgs::LaserScan>("clean_scan", 1);
	// rp.goal_pub = n.advertise<geometry_msgs::PointStamped>("global_goal", 1);
	rp.new_goal_pub = n.advertise<geometry_msgs::PointStamped>("new_global_goal", 1);
	rp.int_goal_pub = n.advertise<geometry_msgs::PoseArray>("int_goal", 1);
	rp.last_goal_pub = n.advertise<geometry_msgs::PointStamped>("last_global_goal",1);

	rp.quad_goal_pub = n.advertise<acl_system::QuadGoal>("goal",1);

	// run the code
	// start asynchronous spinner
	ros::AsyncSpinner spinner(2); // Use 2 threads
	spinner.start();
	ros::waitForShutdown();

	return 0;
}
