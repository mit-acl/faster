#include "rp.hpp"

REACT::REACT(){

	pose.setZero();

	// Should be read as param
	thresh = 0.5; 
	goal.header.stamp = ros::Time::now();
	goal.header.frame_id = "vicon";
	goal.point.x =  0;
	goal.point.y = -5;
	goal.point.z = 0;

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

void REACT::sendGoal(const ros::TimerEvent& e)
{
	goal_pub.publish(goal);
}

void REACT::scanCB(const sensor_msgs::LaserScan& msg)
 {
 	partition_scan(msg);
 	find_free_space();
 }


void REACT::partition_scan(const sensor_msgs::LaserScan& msg){
	std::cout << "Received scan" << std::endl;

 	partitioned_scan = msg;

 	double num_samples = (msg.angle_max - msg.angle_min) / msg.angle_increment;

 	// screenPrint();

 	int j = 0;

    for (int i=0; i < num_samples; i++){
    	
    	if (std::abs(msg.ranges[i+1]-msg.ranges[i]) < thresh){
    		partitioned_scan.ranges[i] = inf;
    	}
    	else{
    		if (isinf(msg.ranges[i])){
    			partitioned_scan.ranges[i] = msg.ranges[i+1];
    		}
    		else {
    			partitioned_scan.ranges[i] = msg.ranges[i];
    		}
    	}

    }

    partitioned_scan.ranges[0] = 0.99*msg.range_max;
    partitioned_scan.ranges[num_samples] = 0.99*msg.range_max;
	
    partitioned_scan_pub.publish(partitioned_scan);
}

void REACT::find_free_space()
{

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
