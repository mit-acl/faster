#include "gpf.hpp"


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "scan_filter", ros::init_options::NoSigintHandler);

	ros::NodeHandle n("~");

	FilterGP fgp;

	// initialize listener callback for state estimate
	ros::Subscriber vic_pose = n.subscribe("vicon", 1, &FilterGP::stateCB, &fgp);
	
	// initialize listener callback for laser scan
	ros::Subscriber sub_scan = n.subscribe("scan", 1, &FilterGP::scanCB, &fgp);


	fgp.filtered_scan_pub = n.advertise<sensor_msgs::LaserScan>("filtered_scan", 1);


	// run the code
	// start asynchronous spinner
	ros::AsyncSpinner spinner(2); // Use 4 threads
	spinner.start();
	ros::waitForShutdown();

	return 0;
}
