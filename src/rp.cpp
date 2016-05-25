#include "rp.hpp"

REACT::REACT(){

	pose.setZero();

	// Should be read as param
	thresh = 0.5; 
	debug = 1;
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

		yaw = tf::getYaw(msg.pose.orientation);
	} 
	// if (msg.has_twist) velCallback(msg.twist);
}

void REACT::sendGoal(const ros::TimerEvent& e)
{
	goal_pub.publish(goal);
}

void REACT::scanCB(const sensor_msgs::LaserScan& msg)
 {
 	if (debug){
 		vis_better_scan(msg);
 	}
 	partition_scan(msg);
 	find_free_space();
 }

void REACT::vis_better_scan(const sensor_msgs::LaserScan& msg)
 {
 	sensor_msgs::LaserScan clean_scan;
 	clean_scan = msg;
 	clean_scan.range_max = 1.1*msg.range_max;
 	double num_samples = (msg.angle_max - msg.angle_min) / msg.angle_increment + 1;
    for (int i=0; i < num_samples; i++){
    	if(isinf(clean_scan.ranges[i])){
    		clean_scan.ranges[i] = msg.range_max;
    	}
    }
    pub_clean_scan.publish(clean_scan);
 }

void REACT::partition_scan(const sensor_msgs::LaserScan& msg){
	std::cout << "Received scan" << std::endl;

	geometry_msgs::PoseArray goal_points;

	goal_points.header.stamp = ros::Time::now();
	goal_points.header.frame_id = "vicon";

 	partitioned_scan = msg;
 	partitioned_scan.range_max = 6;

 	double num_samples = (msg.angle_max - msg.angle_min) / msg.angle_increment + 1;

 	// screenPrint();

 	int j = 0;
 	double sum = 0;
 	double r = 0;
 	double angle = 0;

 	sensor_msgs::LaserScan filtered_scan;

 	filtered_scan = msg;

 	geometry_msgs::Pose temp;

    for (int i=0; i < num_samples; i++){
    	if (isinf(filtered_scan.ranges[i]) || isnan(filtered_scan.ranges[i])){
    		filtered_scan.ranges[i] = 0.99*filtered_scan.range_max;
    	}

    	if (isinf(filtered_scan.ranges[i+1]) || isnan(filtered_scan.ranges[i+1])){
    		filtered_scan.ranges[i+1] = 0.99*filtered_scan.range_max;
    	}

    	if (std::abs(filtered_scan.ranges[i+1]-filtered_scan.ranges[i]) < thresh){
    		// Do nothing
    		if (isinf(filtered_scan.ranges[i]) || isnan(filtered_scan.ranges[i])){
    			sum += filtered_scan.range_max;
    		}
    		else{
    			sum += filtered_scan.ranges[i];
    		}
    		// std::cout << sum << std::endl;
    		// std::cout << filtered_scan.ranges[i] << std::endl;
    	}
    	else{
    		std::cout << std::abs(filtered_scan.ranges[i+1]-filtered_scan.ranges[i]) << std::endl;
    		std::cout << filtered_scan.ranges[i+1] << std::endl;
    		std::cout << filtered_scan.ranges[i] << std::endl;

    		r = sum/(i-j+1);
    		std::cout << "i: " << i << " sum: " << sum << " r: " << r << std::endl;
    		angle = filtered_scan.angle_min + filtered_scan.angle_increment*(i+j)/2 + yaw;
    		temp.position.x = r*cos(angle);
    		temp.position.y = r*sin(angle);
    		temp.position.z = 0;
    		goal_points.poses.push_back(temp);
    		sum = 0;
    		j = i;
    	}
    	
    	// if (std::abs(msg.ranges[i+1]-msg.ranges[i]) < thresh){
    	// 	partitioned_scan.ranges[i] = inf;
    	// }
    	// else{
    	// 	if (isinf(msg.ranges[i])){
    	// 		partitioned_scan.ranges[i] = msg.ranges[i+1];
    	// 	}
    	// 	else {
    	// 		partitioned_scan.ranges[i] = msg.ranges[i];
    	// 	}

	    if (isinf(partitioned_scan.ranges[0])){
	 	   partitioned_scan.ranges[0] = msg.range_max;
		}
		if (isinf(partitioned_scan.ranges[num_samples])){
	    	partitioned_scan.ranges[num_samples] = msg.range_max;
		}
	    partitioned_scan_pub.publish(partitioned_scan);
		
		int_goal_pub.publish(goal_points);
	}
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
