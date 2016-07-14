#include "rp.hpp"

REACT::REACT(){

	pose_.setZero();

	// Should be read as param
	ros::param::get("~thresh",thresh_);
	ros::param::get("~debug",debug_);
	ros::param::get("~angle_check",angle_check_);
	ros::param::get("~safe_distance",safe_distance_);
	ros::param::get("~buffer",buffer_);

	goal_.header.stamp = ros::Time::now();
	goal_.header.frame_id = "vicon";

	ros::param::get("~goal_x",goal_.point.x);
	ros::param::get("~goal_y",goal_.point.y);
	ros::param::get("~goal_z",goal_.point.z);

	ros::param::get("cntrl/spinup_time",spinup_time_);

	ros::param::get("~heading",heading_);

	last_goal_.header.stamp = ros::Time::now();
	last_goal_.header.frame_id = "vicon";
	last_goal_.point.x = goal_.point.x;
	last_goal_.point.y = goal_.point.y;
	last_goal_.point.z = goal_.point.z;

	// Should be params
	j_max_ = 30;
	a_max_ = 5;

	angle_seg_inc_ = 10*PI/180;

	num_of_partitions_ = 0;
	collision_counter_corridor_ = 0;
	collision_counter_ = 0;
	can_reach_goal_ = false;
	corridor_free_ = false;
	inf_ = std::numeric_limits<double>::max();

	quad_status_ = state_.NOT_FLYING;

	quad_goal_.cut_power = true;

	ROS_INFO("Planner initialized.");

}

void REACT::global_goalCB(const geometry_msgs::PointStamped& msg){
	goal_ = msg;
}

void REACT::stateCB(const acl_system::ViconState& msg)
{
	// TODO time check.
	if (msg.has_pose) {
		pose_.setX(msg.pose.position.x);
		pose_.setY(msg.pose.position.y);
		pose_.setZ(msg.pose.position.z);	

		yaw_ = tf::getYaw(msg.pose.orientation);
	} 
	// if (msg.has_twist) velCallback(msg.twist);
}

void REACT::sendGoal(const ros::TimerEvent& e)
{
	if (quad_status_== state_.TAKEOFF){
		takeoff();
		if (quad_goal_.pos.z == goal_.point.z){
			quad_status_ = state_.FLYING;
		}
	}

	else if (quad_status_== state_.LAND){
			land();
			if (quad_goal_.pos.z == -0.1){
				quad_status_ = state_.NOT_FLYING;
				quad_goal_.cut_power = true;
			}
		}

	else if (quad_status_ == state_.GO){
		t_ = ros::Time::now().toSec() - t0_;
		eval_trajectory(quad_goal_,X0_,Y0_,t_x_,t_y_,t_);
	}

	quad_goal_.header.stamp = ros::Time::now();
	quad_goal_.header.frame_id = "vicon";
	quad_goal_pub.publish(quad_goal_);
}


void REACT::takeoff(){
	quad_goal_.pos.z+=0.003;
	saturate(quad_goal_.pos.z,-0.1,goal_.point.z);
}


void REACT::land(){
	if (quad_goal_.pos.z > 0.4){
		quad_goal_.pos.z-=0.003;
		saturate(quad_goal_.pos.z,-0.1,goal_.point.z);
	}
	else{
		quad_goal_.pos.z-=0.001;
		saturate(quad_goal_.pos.z,-0.1,goal_.point.z);
	}
}

void REACT::eventCB(const acl_system::QuadFlightEvent& msg)
{
	// Takeoff
	if (msg.mode == msg.TAKEOFF && quad_status_== state_.NOT_FLYING){

		ROS_INFO("Waiting for spinup");
		ros::Duration(spinup_time_).sleep();
		ROS_INFO("Taking off");


		quad_status_ = state_.TAKEOFF; 
		quad_goal_.pos.x = pose_.getX();
		quad_goal_.pos.y = pose_.getY();
		quad_goal_.pos.z = pose_.getZ();

		quad_goal_.vel.x = 0;
		quad_goal_.vel.y = 0;
		quad_goal_.vel.z = 0;

		quad_goal_.yaw = yaw_;
		quad_goal_.dyaw = 0;

		quad_goal_.cut_power = false;

	}
	// Emergency kill
	else if (msg.mode == msg.KILL && quad_status_ != state_.NOT_FLYING){
		quad_status_ = state_.NOT_FLYING;
		quad_goal_.cut_power = true;
		ROS_ERROR("Killing");
	}
	// Landing
	else if (msg.mode == msg.LAND && quad_status_ == state_.FLYING){
		quad_status_ = state_.LAND;
		ROS_INFO("Landing");
	}
	// Initializing
	else if (msg.mode == msg.INIT && quad_status_ == state_.FLYING){
		quad_goal_.yaw = heading_;
		ROS_INFO("Initialized");
	}
	// GO!!!!
	else if (msg.mode == msg.START && quad_status_ == state_.FLYING){
		quad_status_ = state_.GO;
		ROS_INFO("Starting");
		// Set speed to desired speed
	}
	// STOP!!!
	else if (msg.mode == msg.ESTOP && quad_status_ == state_.GO){
		ROS_WARN("Stopping");
		// Stay in go command but set speed to zero
	}


}

void REACT::find_times(std::vector<double>& t, Eigen::Matrix4d& X0, std::vector<double> x, double vf){
	double j_temp = copysign(j_max_,vf-x[1]);
	double vfp = x[1] + pow(x[2],2)/(2*j_temp);

	if (std::abs(vfp-vf) < 0.05){
		j_[0] = -j_temp;
		// No 2nd and 3rd stage
		j_[1] = 0;
		j_[2] = 0;

		t[0] = -x[2]/j_[0];
		// No 2nd and 3rd stage
		t[1] = 0;
		t[2] = 0;

		v0_[0] = x[1];
		// No 2nd and 3rd stage
		v0_[1] = 0;
		v0_[2] = 0;
		v0_[3] = vf;

		x0_[0] = x[0];
		// No 2nd and 3rd stage
		x0_[1] = 0;
		x0_[2] = 0;
		x0_[3] = x0_[1] + v0_[0]*t[0];

		a0_[0] = x[2];
		// No 2nd and 3rd stage
		a0_[1] = 0;
		a0_[2] = 0;
		a0_[3] = 0;
	}

	else{
		j_[0] = j_temp;
		j_[1] = 0;
		j_[2] = -j_temp;

		double t1 = -x[2]/j_temp + std::sqrt(0.5*pow(x[2],2) - j_temp*(x0_[1]-vf))/j_temp;
		double t2 = -x[2]/j_temp - std::sqrt(0.5*pow(x[2],2) - j_temp*(x0_[1]-vf))/j_temp;

		t1 = std::max(t1,t2);

		// Check to see if we'll saturate
		double a1f = x[2] + j_max_*t1;

		if (std::abs(a1f) > a_max_){
			double am = copysign(a_max_,j_temp);
			t[0] = (am-x[2])/j_[0];
			t[2] = -am/j_[2];

			a0_[0] = x[2];
			a0_[1] = a0_[0] + j_[0]*t[0];
			a0_[2] = am;
			a0_[3] = 0;

			v0_[0] = x[1];
			v0_[1] = v0_[1] + a0_[0]*t[0] + 0.5*j_[0]*pow(t[0],2);	
			v0_[2] = vf - am*t[2] - 0.5*j_[2]*pow(t[0],2);
			v0_[3] = vf;

			t[1] = (v0_[2]-v0_[1])/am;			

			x0_[0] = x[0];
			x0_[1] = x0_[0] + v0_[0]*t[0] + 0.5*a0_[0]*pow(t[0],2) + 1./6*j_[0]*pow(t[0],3);
			x0_[2] = x0_[1] + v0_[1]*t[1] + 0.5*am*pow(t[1],2) ;
			x0_[3] = x0_[2] + v0_[2]*t[2] + 0.5*am*pow(t[2],2) + 1./6*j_[2]*pow(t[2],3);

		}
		else{
			j_[0] = j_temp;
			j_[1] = 0; // No second phase
			j_[2] = -j_temp;

			t[0] = t1;
			t[1] = 0; // No second phase
			t[2] = -(x[2]+j_[0]*t1)/j_[2];

			a0_[0] = x[2];
			a0_[1] = 0; // No second phase
			a0_[2] = a0_[0] + j_[0]*t[0];
			a0_[3] = 0;

			v0_[0] = x[1];
			v0_[1] = 0; // No second phase
			v0_[2] = v0_[0] + a0_[0]*t[0] + 0.5*j_[0]*pow(t[0],2);
			v0_[3] = vf;		

			x0_[0] = x[0];
			x0_[1] = 0; // No second phase
			x0_[2] = x0_[0] + v0_[0]*t[0] + 0.5*a0_[0]*pow(t[0],2) + 1./6*j_[0]*pow(t[0],3);
			x0_[3] = x0_[2] + v0_[2]*t[2] + 0.5*a0_[2]*pow(t[2],2) + 1./6*j_[2]*pow(t[2],3);

		}
	}

	X0.row(0) << x0_[0],x0_[1],x0_[2],x0_[3];
	X0.row(1) << v0_[0],v0_[1],v0_[2],v0_[3];
	X0.row(2) << a0_[0],a0_[1],a0_[2],a0_[3];
	X0.row(3) << j_[0],j_[1],j_[2],j_[3];
}


void REACT::eval_trajectory(acl_system::QuadGoal& goal, Eigen::Matrix4d X0, Eigen::Matrix4d Y0, std::vector<double> t_x, std::vector<double> t_y, double t){
	// Eval x trajectory
	int k = 0;
	if (t < t_x[0]){
		k = 0;
	}
	else if (t < t_x[0]+t_x[1]){
		t -= t_x[0];
		k = 1;
	}
	else if (t < t_x[0]+t_x[1]+t_x[2]){
		t -= (t_x[0]+t_x[1]);
		k = 2;
	}
	else{
		t -= (t_x[0]+t_x[1]+t_x[2]);
		k = 3;
	}

	// Eval y trajectory
	int l = 0;
	if (t < t_y[0]){
		l = 0;
	}
	else if (t < t_y[0]+t_y[1]){
		t -= t_y[0];
		l = 1;
	}
	else if (t < t_y[0]+t_y[1]+t_y[2]){
		t -= (t_y[0]+t_y[1]);
		l = 2;
	}
	else{
		t -= (t_y[0]+t_y[1]+t_y[2]);
		l = 3;
	}

	goal.pos.x = X0(0,k) + X0(1,k)*t + 0.5*X0(2,k)*pow(t,2) + 1.0/6.0*X0(3,k)*pow(t,3);
	goal.vel.x = X0(1,k) + X0(2,k)*t + 0.5*X0(3,k)*pow(t,2);
	goal.accel.x = X0(2,k) + X0(3,k)*t;

	goal.pos.y = Y0(0,l) + Y0(1,l)*t + 0.5*Y0(2,l)*pow(t,2) + 1.0/6.0*Y0(3,l)*pow(t,3);
	goal.vel.y = Y0(1,l) + Y0(2,l)*t + 0.5*Y0(3,l)*pow(t,2);
	goal.accel.y = Y0(2,l) + Y0(3,l)*t;
}



void REACT::scanCB(const sensor_msgs::LaserScan& msg)
 {
 	if (debug_){
 		vis_better_scan(msg);
 	}
 	msg_received_ = ros::Time::now().toSec();
 	check_goal(msg);
 	if (!can_reach_goal_){
 		partition_scan(msg);
 		find_inter_goal();
 	}
 	else{
 		new_goal_.header.stamp = ros::Time::now();
	 	new_goal_.header.frame_id = "vicon";
	 	new_goal_.point.x = goal_.point.x;
	 	new_goal_.point.y = goal_.point.y;
	 	new_goal_pub.publish(new_goal_);
 	}
 	
 }

// We might not need this...
 void REACT::scan2Eig(const sensor_msgs::LaserScan msg, Eigen::MatrixXd scan){
 	angle_max_ = msg.angle_max;
	angle_min_ = msg.angle_min;
	angle_increment_ = msg.angle_increment;

	num_samples_ = (int) std::floor((angle_max_ - angle_min_) / angle_increment_ );

	scan.resize(2,num_samples_);

	// Seems really inefficient
	for (int i=0;i<num_samples_;i++){
		scan(0,i) = angle_min_ + i*angle_increment_;
		scan(1,i) = msg.ranges[i];
	}
}
 

void REACT::check_goal(const sensor_msgs::LaserScan& msg)
{
	std::cout << "Received scan" << std::endl;

	// Distance to goal
	dist_2_goal_ = sqrt( pow(goal_.point.x-pose_.getX(),2) + pow(goal_.point.y-pose_.getY(),2));
	// Angle to goal in body frame
	angle_2_goal_ = atan2( goal_.point.y - pose_.getY(), goal_.point.x - pose_.getX() ) - yaw_; 
	std::cout << "Distance: " << dist_2_goal_ << " Angle: " << angle_2_goal_ << std::endl;

	collision_counter_ = 0;

	angle_max_ = msg.angle_max;
	angle_min_ = msg.angle_min;
	angle_increment_ = msg.angle_increment;

	num_samples_ = (angle_max_ - angle_min_) / angle_increment_ + 1;
	double sum = 0;
	double temp_range = 0;

	int j = (int) ((angle_2_goal_ - angle_min_)/angle_increment_);
	int delta = (int) (angle_check_/angle_increment_) ;

	for (int i=j-delta; i < j+delta; i++)
	{
		if(isinf(msg.ranges[i]) || isnan(msg.ranges[i])){
    		temp_range = msg.range_max;
    	}
    	else{
    		temp_range = msg.ranges[i];
    	}
		sum += temp_range;
		if (dist_2_goal_ > temp_range) collision_counter_+=1;
	}

	double r = sum/(2*delta);


	std::cout << "r: " << r <<std::endl;
	std::cout << "collision counter: " << collision_counter_ << std::endl;

	if (r > dist_2_goal_ && collision_counter_ < 10){
		can_reach_goal_ = true;
	}
	else{
		can_reach_goal_ = false;
	}

	std::cout << "Can reach goal: " << can_reach_goal_ << std::endl;
}


void REACT::find_inter_goal(){
 	// Re-initialize cost
 	std::vector<double> cost_v;
 	std::vector<double> angles;
 	std::vector<double> ranges;

 	corridor_free_ = false;

	std::priority_queue<double, std::vector<double>, std::greater<double> > cost_queue;

	last_goal_v_.setX(last_goal_.point.x - pose_.getX());
 	last_goal_v_.setY(last_goal_.point.y - pose_.getY());
 	last_goal_v_.setZ(last_goal_.point.z - pose_.getZ());

 	last_goal_v_.normalize();

 	for (int i=0; i < num_of_partitions_ ; i++){

 		next_goal_v_.setX(goal_points_.poses[i].pose.position.x - pose_.getX());
 		next_goal_v_.setY(goal_points_.poses[i].pose.position.y - pose_.getY());
 		next_goal_v_.setZ(goal_points_.poses[i].pose.position.z - pose_.getZ());

 		next_goal_v_.normalize();


 		double r_i = sqrt(pow(pose_.getX() - goal_points_.poses[i].pose.position.x, 2) + pow( pose_.getY() - goal_points_.poses[i].pose.position.y, 2));
 		double angle_i = atan2 ( goal_points_.poses[i].pose.position.y - pose_.getY(), goal_points_.poses[i].pose.position.x - pose_.getX() ) - yaw_;
 		angle_diff_  =  std::abs(angle_i)  - std::abs(angle_2_goal_ );

 		angle_diff_last_ = next_goal_v_.angle(last_goal_v_);

 		cost_i_ = pow(angle_diff_,2) + pow(angle_diff_last_,2) + pow(1/r_i,2);

 		// std::cout << "i: " << i << " cost_i_: " << cost_i_ << std::endl;
 		// std::cout << "r_i: " << r_i << " angle_i: " << angle_i << " angle_diff: " << angle_diff << " angle_diff_last_: " << angle_diff_last_ << std::endl;

 		cost_queue.push(cost_i_);
 		cost_v.push_back(cost_i_);
 		angles.push_back(angle_i);
 		ranges.push_back(r_i);
 	}

 	int goal_counter = 0;
 	// Collision check
 	while (!corridor_free_){
 		collision_counter_corridor_ = 0;

 		// Collision scan for debug
 		sensor_msgs::LaserScan collision_scan;
 		collision_scan.header.stamp = ros::Time::now();
 		collision_scan.header.frame_id = "laser";
 		collision_scan.range_min = 0.01;
 		collision_scan.range_max = 1.1*safe_distance_;
 		collision_scan.angle_increment = angle_increment_; 

 		min_cost_ = cost_queue.top();

 		std::vector<double>::iterator it;
 		it = std::find(cost_v.begin(),cost_v.end(),min_cost_);
 		goal_index_ = it - cost_v.begin();

 		double current_part_angle = angles[goal_index_];
 		double current_part_range = ranges[goal_index_];

		int j = (int) ((current_part_angle - angle_min_)/angle_increment_);
		int delta = (int) (PI/4/angle_increment_) ;
		int delta_1 = (int) (PI/4/angle_increment_);
		int delta_2 = (int) (PI/4/angle_increment_);

		// Check we're within scan bounds
		if (j-delta < 0){
			delta_1 = j;
		}
		else if (j+delta > num_samples_){
			delta_2 = num_samples_-j;
		}

		collision_scan.angle_min = angle_min_ + angle_increment_*(j-delta_1);
		collision_scan.angle_max = angle_min_ + angle_increment_*(j+delta_2);

		std::cout << "j: " << j << " delta: " << delta << " goal index: " << goal_index_  << " goal_counter: " << goal_counter <<  std::endl;

		std::cout << "range: " << current_part_range << std::endl;
		// r and theta used to check predicted ranges
		double r_temp ;
		double theta = angle_increment_*(delta-delta_1) + PI/4;
		// double theta = 0;

		std::cout << "delta_1: " << delta_1 << " delta_2: " << delta_2 << std::endl;

		// Check scan ccw
		for (int i=j-delta_1; i < j+delta_2; i++){
			r_temp = std::abs(buffer_/std::cos(theta));

			r_temp = std::min(r_temp,safe_distance_);

			collision_scan.ranges.push_back(r_temp);

			if (r_temp > filtered_scan_.ranges[i]){
				collision_counter_corridor_+=1;
			}

			if (collision_counter_corridor_>9) break;

			theta+=angle_increment_;		

		}

		if (collision_counter_corridor_<10) 
		{
			corridor_free_=true; 
 			corridor_scan_pub.publish(collision_scan);
 		}


		std::cout << "corridor collision counter: " << collision_counter_corridor_ << std::endl;


 		// Erase current elements from cost vector
 		if (!corridor_free_){
 			cost_queue.pop();
 			goal_counter+=1;

 			if(cost_queue.empty()){
 				std::cout << "Need to stop!!!!!!" << std::endl;
 				break;
 			}
 		}
		std::cout << "cost size: " << cost_v.size() << std::endl;
 	}

 	std::cout << "min_cost_: " << min_cost_ << " goal index: " << goal_index_ <<  std::endl;

 	new_goal_.header.stamp = ros::Time::now();
 	new_goal_.header.frame_id = "vicon";
 	new_goal_.point.x = goal_points_.poses[goal_index_].pose.position.x;
 	new_goal_.point.y = goal_points_.poses[goal_index_].pose.position.y;
 	new_goal_.point.z = goal_.point.z;

 	new_goal_pub.publish(new_goal_);
 	last_goal_pub.publish(last_goal_);

 	last_goal_.header.stamp = ros::Time::now();
 	last_goal_.header.frame_id = "vicon";
 	last_goal_.point = new_goal_.point;

	std::cout << "Latency: " << ros::Time::now().toSec() - msg_received_ << std::endl;
 }


void REACT::partition_scan(const sensor_msgs::LaserScan& msg){
	std::cout << "Partioning scan" << std::endl;

	// geometry_msgs::PoseArray goal_points;

	goal_points_.header.stamp = ros::Time::now();
	goal_points_.header.frame_id = "vicon";

	goal_points_.poses.clear();

 	// screenPrint();

 	int j = 0;
 	double sum = 0;

 	std::vector<double> r;
 	std::vector<double> r_temp;
 	std::vector<double> angle;
 	std::vector<double> angle_temp;
 
 	num_of_partitions_ = 0;

 	filtered_scan_ = msg;
 	filtered_scan_.range_max = 1.1*msg.range_max;

 	geometry_msgs::PoseStamped temp;

    for (int i=0; i < num_samples_; i++){
    	if (isinf(filtered_scan_.ranges[i]) || isnan(filtered_scan_.ranges[i]) ){
    		filtered_scan_.ranges[i] = msg.range_max;
    	}

    	if (isinf(filtered_scan_.ranges[i+1]) || isnan(filtered_scan_.ranges[i+1])){
    		filtered_scan_.ranges[i+1] = msg.range_max;
    	}

    	if (std::abs(filtered_scan_.ranges[i+1]-filtered_scan_.ranges[j]) < thresh_){
    			sum += filtered_scan_.ranges[i+1];  
    	}
    	else{
    		if ((i-j)>10){

    			// Convert angle segment incerement to index
    			double angle_2_index = angle_seg_inc_/angle_increment_;

    			// Probably a better way to do this...
    			if (double((i-j))/(3*angle_2_index) < 1) num_of_partitions_ = 1;
    			if (double((i-j))/(3*angle_2_index) > 1) num_of_partitions_ = 3;
    			if (double((i-j))/(5*angle_2_index) > 1) num_of_partitions_ = 5;
    			if (double((i-j))/(7*angle_2_index) > 1) num_of_partitions_ = 7;
    			if (double((i-j))/(9*angle_2_index) > 1) num_of_partitions_ = 9;
    			if (double((i-j))/(11*angle_2_index) > 1) num_of_partitions_ = 11;

    			for (int k=0; k < num_of_partitions_; k++){
    				r_temp.push_back(sum/(i-j));
		    		angle_temp.push_back(filtered_scan_.angle_min + filtered_scan_.angle_increment*((i+j)/2 + (k-(num_of_partitions_-1)/2)*angle_2_index) + yaw_);
    			}
    		}

    			sum = 0;
    			j = i+1;   			
		}
	}

	for (int i = 0; i < r_temp.size(); i++){
		if (r_temp[i] > safe_distance_){
			r.push_back(r_temp[i]);
			angle.push_back(angle_temp[i]);

			temp.pose.position.x = r[num_of_partitions_]*cos(angle[num_of_partitions_]) + pose_.getX();
			temp.pose.position.y = r[num_of_partitions_]*sin(angle[num_of_partitions_]) + pose_.getY();
			temp.pose.position.z = goal_.point.z;
			temp.header.seq = num_of_partitions_;
			goal_points_.poses.push_back(temp);
			num_of_partitions_+=1;
		}
	}
	
	int_goal_pub.publish(goal_points_);
}


void REACT::vis_better_scan(const sensor_msgs::LaserScan& msg)
 {
 	sensor_msgs::LaserScan clean_scan;
 	clean_scan = msg;
 	clean_scan.header.frame_id = "laser";
 	clean_scan.range_max = 1.1*msg.range_max;
 	double num_samples = (msg.angle_max - msg.angle_min) / msg.angle_increment + 1;
    for (int i=0; i < num_samples; i++){
    	if(isinf(clean_scan.ranges[i])){
    		clean_scan.ranges[i] = msg.range_max;
    	}
    }
    pub_clean_scan.publish(clean_scan);
 }

void REACT::saturate(double &var, double min, double max){
	if (var < min){
		var = min;
	}
	else if (var > max){
		var = max;
	}
}
