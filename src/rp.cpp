#include "rp.hpp"

REACT::REACT(){

	pose.setZero();

	// Should be read as param
	thresh = 0.5; 
	debug = 1;
	angle_check = 40*3.14159/180; // deg
	safe_distance = 2;

	goal.header.stamp = ros::Time::now();
	goal.header.frame_id = "vicon";
	goal.point.x =  0;
	goal.point.y = -5;
	goal.point.z = 0.5;

	num_of_partitions = 0;
	can_reach_goal = false;
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
 	msg_received = ros::Time::now().toSec();
 	check_goal(msg);
 	if (!can_reach_goal){
 		partition_scan(msg);
 		find_inter_goal();
 	}
 	else{
 		new_goal.header.stamp = ros::Time::now();
	 	new_goal.header.frame_id = "vicon";
	 	new_goal.point.x = goal.point.x;
	 	new_goal.point.y = goal.point.y;
	 	new_goal_pub.publish(new_goal);
 	}
 	
 }

 void REACT::find_inter_goal(){
 	// Re-initialize cost
 	cost = inf;
 	for (int i=0; i < num_of_partitions ; i++){
 		double r_i = sqrt(pow(pose.getX() - goal_points.poses[i].pose.position.x, 2) + pow( pose.getY() - goal_points.poses[i].pose.position.y, 2));
 		double angle_i = atan2 ( goal_points.poses[i].pose.position.y - pose.getY(), goal_points.poses[i].pose.position.x - pose.getX() ) - yaw;
 		angle_diff  =  angle_i  - angle_2_goal;
 		// cost_i = pow(angle_diff,2) + pow(1/r_i,4);
 		cost_i = pow(angle_diff,2) ;


 		std::cout << "i: " << i << " cost_i: " << cost_i << std::endl;
 		std::cout << "r_i: " << r_i << " angle_i: " << angle_i << " angle_diff: " << angle_diff << std::endl;

 		if (cost_i < cost){
 			cost = cost_i;
 			goal_index = i;
 		}
 	}
 	new_goal.header.stamp = ros::Time::now();
 	new_goal.header.frame_id = "vicon";
 	new_goal.point.x = goal_points.poses[goal_index].pose.position.x;
 	new_goal.point.y = goal_points.poses[goal_index].pose.position.y;
 	new_goal.point.z = goal.point.z;

 	new_goal_pub.publish(new_goal);

 }

 void REACT::check_goal(const sensor_msgs::LaserScan& msg)
{
	std::cout << "Received scan" << std::endl;

	// Distance to goal
	dist_2_goal = sqrt( pow(goal.point.x-pose.getX(),2) + pow(goal.point.y-pose.getY(),2));
	// Angle to goal in body frame
	angle_2_goal = atan2( goal.point.y - pose.getY(), goal.point.x - pose.getX() ) - yaw; 
	std::cout << "Distance: " << dist_2_goal << " Angle: " << angle_2_goal << std::endl;

	collision_counter = 0;

	double num_samples = (msg.angle_max - msg.angle_min) / msg.angle_increment + 1;
	double sum = 0;
	double temp_range = 0;

	int j = (int) ((angle_2_goal - msg.angle_min)/msg.angle_increment);
	int delta = (int) (angle_check/msg.angle_increment) ;

	std::cout << "j: " <<  j << std::endl;
	std::cout << "delta: " << delta << std::endl;

	for (int i=j-delta; i < j+delta; i++)
	{
		if(isinf(msg.ranges[i]) || isnan(msg.ranges[i])){
    		temp_range = msg.range_max;
    	}
    	else{
    		temp_range = msg.ranges[i];
    	}
    	// std::cout << temp_range << std::endl;
		sum += temp_range;
		if (dist_2_goal > temp_range) collision_counter+=1;
	}

	double r = sum/(2*delta);


	std::cout << "r: " << r <<std::endl;
	std::cout << "collision counter: " << collision_counter << std::endl;

	if (r > dist_2_goal && collision_counter < 10){
		can_reach_goal = true;
	}
	else{
		can_reach_goal = false;
	}

	std::cout << "Can reach goal: " << can_reach_goal << std::endl;
}


void REACT::partition_scan(const sensor_msgs::LaserScan& msg){
	std::cout << "Partioning scan" << std::endl;

	// geometry_msgs::PoseArray goal_points;

	goal_points.header.stamp = ros::Time::now();
	goal_points.header.frame_id = "vicon";

	goal_points.poses.clear();

 	double num_samples = (msg.angle_max - msg.angle_min) / msg.angle_increment + 1;

 	// screenPrint();

 	int j = 0;
 	double sum = 0;

 	std::vector<double> r;
 	std::vector<double> r_temp;
 	std::vector<double> angle;
 	std::vector<double> angle_temp;
 
 	num_of_partitions = 0;

 	sensor_msgs::LaserScan filtered_scan;

 	filtered_scan = msg;
 	filtered_scan.range_max = 1.1*msg.range_max;

 	geometry_msgs::PoseStamped temp;

    for (int i=0; i < num_samples; i++){
    	if (isinf(filtered_scan.ranges[i]) || isnan(filtered_scan.ranges[i])){
    		filtered_scan.ranges[i] = msg.range_max;
    	}

    	if (isinf(filtered_scan.ranges[i+1]) || isnan(filtered_scan.ranges[i+1])){
    		filtered_scan.ranges[i+1] = msg.range_max;
    	}

    	if (std::abs(filtered_scan.ranges[i+1]-filtered_scan.ranges[i]) < thresh){
    			sum += filtered_scan.ranges[i+1];  
    	}
    	else{
    		if ((i-j)>10){

    			// if (sum/(i-j) > *std::max_element(r.begin(),r.end())) r.clear();

	    		r_temp.push_back(sum/(i-j));
	    		// std::cout << "i: " << i << " j: " << j << " sum: " << sum << " r: " << r << std::endl;
	    		angle_temp.push_back(filtered_scan.angle_min + filtered_scan.angle_increment*(i+j)/2 + yaw);
	    		sum = 0;
	    		j = i+1;
    		}
    	}		
	}

	double r_max = *std::max_element(r_temp.begin(),r_temp.end());

	for (int i = 0; i < r_temp.size(); i++){
		if (r_temp[i] > safe_distance){
			r.push_back(r_temp[i]);
			angle.push_back(angle_temp[i]);
		}
	}
	
	for(int i = 0; i < r.size(); i++)
	{
		temp.pose.position.x = r[i]*cos(angle[i]) + pose.getX();
		temp.pose.position.y = r[i]*sin(angle[i]) + pose.getY();
		temp.pose.position.z = goal.point.z;
		temp.header.seq = num_of_partitions;
		goal_points.poses.push_back(temp);
		num_of_partitions+=1;
	}

	int_goal_pub.publish(goal_points);
	std::cout << "Latency: " << ros::Time::now().toSec() - msg_received << std::endl;
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
