#include "rp.hpp"

REACT::REACT(){

	pose.setZero();
	inf = std::numeric_limits<double>::max();

	ROS_INFO("Initialized.");

}

void REACT::stateCB(const acl_system::ViconState& msg)
{
	// TODO time check.
	if (msg.has_pose) {
		pose.setX(msg.pose.position.x);
		pose.setY(msg.pose.position.y);
		pose.setZ(msg.pose.position.z);	

		tf::quaternionMsgToTF(msg.pose.orientation,att);
	} 
	// if (msg.has_twist) velCallback(msg.twist);
}

void REACT::scanCB(const sensor_msgs::LaserScan& msg)
 {

 	std::cout << "Received scan" << std::endl;

 	screenPrint();

    // for (int i=min_idx; i < max_idx; i++){
    // 	double r = rangeCheck(i-min_idx);
    	
    // 	if (msg.ranges[i] > 0.8*r){
    // 		msg_filtered.ranges[i] = inf;
    // 		points_removed++;
    // 	}
    // }
	
    // filtered_scan_pub.publish(msg_filtered);
 }


void REACT::screenPrint()
{
	if (not errorMsg.str().empty())
		ROS_ERROR_STREAM_THROTTLE(SCREEN_PRINT_RATE, errorMsg.str());

	if (not warnMsg.str().empty())
		ROS_WARN_STREAM_THROTTLE(SCREEN_PRINT_RATE, warnMsg.str());

	std::ostringstream msg;
	msg.setf(std::ios::fixed); // give all the doubles the same precision
	msg.setf(std::ios::showpos); // show +/- signs always
	msg << std::setprecision(4) << std::endl; // set precision to 4 decimal places

	// double r, p, y;
	// tf::Matrix3x3(att).getRPY(r, p, y);
	// msg << "Height:           " << alt << std::endl;
	// msg << "Points Removed:   " << points_removed << std::endl;
	// msg << "Attitude:	  r: " << r << "  p: " << p << "  y: " << y << std::endl;

	ROS_INFO_STREAM_THROTTLE(SCREEN_PRINT_RATE, msg.str());
	// Print at 1/0.5 Hz = 2 Hz
}
