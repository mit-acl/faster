#include "tip.hpp"


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "reactive_planner", ros::init_options::NoSigintHandler);

	ros::NodeHandle n("~");

	TIP tip;

	// initialize listener callback for state estimate
	ros::Subscriber vic_pose = n.subscribe("state", 1, &TIP::stateCB, &tip);
	
	// initialize listener callback for pointcloud
	ros::Subscriber sub_pcl = n.subscribe("camera/cloud", 1, &TIP::pclCB, &tip);

	// initialize listener callback for event
	ros::Subscriber event_sub = n.subscribe("event", 1, &TIP::eventCB, &tip);

	// initialize listener callback for global goal
	ros::Subscriber global_goal_sub = n.subscribe("global_goal", 1, &TIP::global_goalCB, &tip);

	// initialize listener callback for system mode
	ros::Subscriber mode_sub = n.subscribe("mode", 1, &TIP::modeCB, &tip);	

	// SendCmd timer
	ros::Timer sendGoalTimer = n.createTimer(ros::Duration(tip.plan_eval_time_), &TIP::sendGoal, &tip);

	tip.traj_pub = n.advertise<nav_msgs::Path>("traj",1);
	tip.tipData_pub = n.advertise<tip::TIP>("tip_data",1);
	tip.quad_goal_pub = n.advertise<acl_msgs::QuadGoal>("goal",1);
	tip.clouds_pub = n.advertise<sensor_msgs::PointCloud2>("clouds",1);
	tip.bl_pub = n.advertise<acl_msgs::FloatStamped>("boundary_layer",1);

	// run the code
	// start asynchronous spinner
	ros::AsyncSpinner spinner(2); // Use 2 threads
	spinner.start();
	ros::waitForShutdown();

	return 0;
}
