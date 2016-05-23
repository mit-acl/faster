#include "rp.hpp"


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "reactive_planner");

	ros::NodeHandle n;

	REACT rp;

	// initialize listener callback for state estimate
	ros::Subscriber vic_pose = n.subscribe("vicon", 1, &REACT::stateCB, &rp);
	
	// initialize listener callback for laser scan
	ros::Subscriber sub_scan = n.subscribe("scan", 1, &REACT::scanCB, &rp);


	// rp.filtered_scan_pub = n.advertise<sensor_msgs::LaserScan>("/filtered_scan", 1);


	// run the code
	// start asynchronous spinner
	ros::AsyncSpinner spinner(2); // Use 2 threads
	spinner.start();
	ros::waitForShutdown();

	return 0;
}
