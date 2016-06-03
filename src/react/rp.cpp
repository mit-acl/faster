#include "rp.hpp"

REACT::REACT(){

	pose.setZero();

	// Should be read as param
	ros::param::get("~thresh",thresh);
	ros::param::get("~debug",debug);
	ros::param::get("~angle_check",angle_check);
	ros::param::get("~safe_distance",safe_distance);
	ros::param::get("~buffer",buffer);

	goal.header.stamp = ros::Time::now();
	goal.header.frame_id = "vicon";

	last_goal.header.stamp = ros::Time::now();
	last_goal.header.frame_id = "vicon";
	last_goal.point.x = 0;
	last_goal.point.y = 0;
	last_goal.point.z = 0;

	ros::param::get("~goal_x",goal.point.x);
	ros::param::get("~goal_y",goal.point.y);
	ros::param::get("~goal_z",goal.point.z);

	num_of_points = 2;

	num_of_partitions = 0;
	collision_counter_corridor = 0;
	collision_counter = 0;
	can_reach_goal = false;
	corridor_free = false;
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

void REACT::check_goal(const sensor_msgs::LaserScan& msg)
{
	std::cout << "Received scan" << std::endl;

	// Distance to goal
	dist_2_goal = sqrt( pow(goal.point.x-pose.getX(),2) + pow(goal.point.y-pose.getY(),2));
	// Angle to goal in body frame
	angle_2_goal = atan2( goal.point.y - pose.getY(), goal.point.x - pose.getX() ) - yaw; 
	std::cout << "Distance: " << dist_2_goal << " Angle: " << angle_2_goal << std::endl;

	collision_counter = 0;

	angle_max = msg.angle_max;
	angle_min = msg.angle_min;
	angle_increment = msg.angle_increment;

	num_samples = (angle_max - angle_min) / angle_increment + 1;
	double sum = 0;
	double temp_range = 0;

	int j = (int) ((angle_2_goal - angle_min)/angle_increment);
	int delta = (int) (angle_check/angle_increment) ;

	// std::cout << "j: " <<  j << std::endl;
	// std::cout << "delta: " << delta << std::endl;

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


void REACT::find_inter_goal(){
 	// Re-initialize cost
 	std::vector<double> cost_v;
 	std::vector<double> angles;
 	std::vector<double> ranges;

 	corridor_free = false;

	std::priority_queue<double, std::vector<double>, std::greater<double> > cost_queue;

 	for (int i=0; i < num_of_partitions ; i++){
 		double r_i = sqrt(pow(pose.getX() - goal_points.poses[i].pose.position.x, 2) + pow( pose.getY() - goal_points.poses[i].pose.position.y, 2));
 		double angle_i = atan2 ( goal_points.poses[i].pose.position.y - pose.getY(), goal_points.poses[i].pose.position.x - pose.getX() ) - yaw;
 		angle_diff  =  std::abs(angle_i)  - std::abs(angle_2_goal);
 		cost_i = pow(angle_diff,2) ;

 		// if(i < 2){
 		// 	std::cout <<  goal_points.poses[i].pose.position.x << " " <<  goal_points.poses[i].pose.position.y << std::endl;
 		// }

 		std::cout << "i: " << i << " cost_i: " << cost_i << std::endl;
 		// std::cout << "r_i: " << r_i << " angle_i: " << angle_i << " angle_diff: " << angle_diff << std::endl;

 		cost_queue.push(cost_i);
 		cost_v.push_back(cost_i);
 		angles.push_back(angle_i);
 		ranges.push_back(r_i);

 	}

 	int goal_counter = 0;
 	// Collision check
 	while (!corridor_free){
 		collision_counter_corridor = 0;

 		min_cost = cost_queue.top();

 		std::vector<double>::iterator it;
 		it = std::find(cost_v.begin(),cost_v.end(),min_cost);
 		goal_index = it - cost_v.begin();

 		double current_part_angle = angles[goal_index];
 		double current_part_range = ranges[goal_index];

		int j = (int) ((current_part_angle - angle_min)/angle_increment);
		int delta = (int) (PI/2/angle_increment) ;
		int delta_1 = (int) (PI/2/angle_increment);
		int delta_2 = (int) (PI/2/angle_increment);

		// Check we're within scan bounds
		if (j-delta < 0){
			delta_1 = j;
		}
		else if (j+delta > num_samples){
			delta_2 = num_samples-j;
		}

		std::cout << "j: " << j << " delta: " << delta << " goal index: " << goal_index  << " goal_counter: " << goal_counter <<  std::endl;

		std::cout << "range: " << current_part_range << std::endl;
		// r and theta used to check predicted ranges
		double r_temp ;
		double theta = angle_increment*(delta-delta_1) ;
		// double theta = 0;

		std::cout << "delta_1: " << delta_1 << " delta_2: " << delta_2 << std::endl;

		// Check scan ccw
		for (int i=j-delta_1; i < j+delta_2; i++){
			r_temp = std::abs(buffer/std::cos(theta));

			r_temp = std::min(r_temp,safe_distance);

			if (r_temp > filtered_scan.ranges[i]){
				// std::cout << i << std::endl;
				// std::cout << filtered_scan.ranges[i] << std::endl;
				// std::cout << r_temp << std::endl;
				// std::cout << theta << std::endl;
				collision_counter_corridor+=1;
			}

			if (collision_counter_corridor>9) break;

			theta+=angle_increment;		

		}

		if (collision_counter_corridor<10) corridor_free=true; 

		std::cout << "corridor collision counter: " << collision_counter_corridor << std::endl;


 		// Erase current elements from cost vector
 		if (!corridor_free){
 			cost_queue.pop();
 			goal_counter+=1;

 			if(cost_v.empty()){
 				std::cout << "Need to stop!!!!!!" << std::endl;
 				break;
 			}
 		}
		std::cout << "cost size: " << cost_v.size() << std::endl;
 	}

 	std::cout << "min_cost: " << min_cost << " goal index: " << goal_index <<  std::endl;

 	new_goal.header.stamp = ros::Time::now();
 	new_goal.header.frame_id = "vicon";
 	new_goal.point.x = goal_points.poses[goal_index].pose.position.x;
 	new_goal.point.y = goal_points.poses[goal_index].pose.position.y;
 	new_goal.point.z = goal.point.z;

 	new_goal_pub.publish(new_goal);
 	last_goal_pub.publish(last_goal);

 	last_goal.header.stamp = ros::Time::now();
 	last_goal.header.frame_id = "vicon";
 	last_goal.point = new_goal.point;

	std::cout << "Latency: " << ros::Time::now().toSec() - msg_received << std::endl;
 }


void REACT::partition_scan(const sensor_msgs::LaserScan& msg){
	std::cout << "Partioning scan" << std::endl;

	// geometry_msgs::PoseArray goal_points;

	goal_points.header.stamp = ros::Time::now();
	goal_points.header.frame_id = "vicon";

	goal_points.poses.clear();

 	// screenPrint();

 	int j = 0;
 	double sum = 0;

 	std::vector<double> r;
 	std::vector<double> r_temp;
 	std::vector<double> angle;
 	std::vector<double> angle_temp;
 
 	num_of_partitions = 0;

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

    	if (std::abs(filtered_scan.ranges[i+1]-filtered_scan.ranges[j]) < thresh){
    			sum += filtered_scan.ranges[i+1];  
    	}
    	else{
    		if ((i-j)>10){
    			// Number of points 
    			for (int k=0; k<num_of_points+1;k++){
    				r_temp.push_back(sum/(i-j));
		    		// std::cout << "i: " << i << " j: " << j << " sum: " << sum << " r: " << r << std::endl; 
		    		angle_temp.push_back(filtered_scan.angle_min + filtered_scan.angle_increment*(i+j)/2 + (k-1)*filtered_scan.angle_increment*(i-j)/(2*num_of_points) + yaw);
    			}
    		}

    		sum = 0;
    		j = i+1;
    	}		
	}

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
