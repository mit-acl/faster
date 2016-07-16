#include "rp.hpp"

REACT::REACT(){


	// Should be read as param
	ros::param::get("~thresh",thresh_);
	ros::param::get("~debug",debug_);
	ros::param::get("~angle_check",angle_check_);
	ros::param::get("~safe_distance",safe_distance_);
	ros::param::get("~buffer",buffer_);

	last_goal_ = Eigen::Vector3d::Zero();
	pose_= Eigen::Vector3d::Zero();
	goal_ = Eigen::Vector3d::Zero();

	ros::param::get("~goal_x",goal_(0));
	ros::param::get("~goal_y",goal_(1));
	ros::param::get("~goal_z",goal_(2));

	ros::param::get("cntrl/spinup_time",spinup_time_);

	ros::param::get("~heading",heading_);

	// Should be params
	j_max_ = 30;
	a_max_ = 5;

	angle_seg_inc_ = 10*PI/180;

	num_of_clusters_ = 0;
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
	goal_(0) = msg.point.x;
	goal_(1) = msg.point.y;
	goal_(2) = msg.point.z;

}

void REACT::stateCB(const acl_system::ViconState& msg)
{
	// TODO time check.
	if (msg.has_pose) {
		pose_(0) = msg.pose.position.x;
		pose_(1) = msg.pose.position.y;
		pose_(2) = msg.pose.position.z;

		yaw_ = tf::getYaw(msg.pose.orientation);
	} 
	// if (msg.has_twist) velCallback(msg.twist);
}

void REACT::sendGoal(const ros::TimerEvent& e)
{
	if (quad_status_== state_.TAKEOFF){
		takeoff();
		if (quad_goal_.pos.z == goal_(2)){
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

		// Need to convert to quad_goal_!!!
		eval_trajectory(Xc_,X0_,Y0_,t_x_,t_y_,t_);
	}

	quad_goal_.header.stamp = ros::Time::now();
	quad_goal_.header.frame_id = "vicon";
	quad_goal_pub.publish(quad_goal_);
}


void REACT::takeoff(){
	quad_goal_.pos.z+=0.003;
	saturate(quad_goal_.pos.z,-0.1,goal_(2));
}


void REACT::land(){
	if (quad_goal_.pos.z > 0.4){
		quad_goal_.pos.z-=0.003;
		saturate(quad_goal_.pos.z,-0.1,goal_(2));
	}
	else{
		quad_goal_.pos.z-=0.001;
		saturate(quad_goal_.pos.z,-0.1,goal_(2));
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
		quad_goal_.pos.x = pose_(0);
		quad_goal_.pos.y = pose_(1);
		quad_goal_.pos.z = pose_(2);

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

void REACT::find_times(std::vector<double>& t, Eigen::Matrix4d& X0, Eigen::Vector3d x, double vf){
	double j_temp = copysign(j_max_,vf-x(1));
	double vfp = x(1) + pow(x(2),2)/(2*j_temp);

	if (std::abs(vfp-vf) < 0.05){
		j_[0] = -j_temp;
		// No 2nd and 3rd stage
		j_[1] = 0;
		j_[2] = 0;

		t[0] = -x(2)/j_[0];
		// No 2nd and 3rd stage
		t[1] = 0;
		t[2] = 0;

		v0_[0] = x(1);
		// No 2nd and 3rd stage
		v0_[1] = 0;
		v0_[2] = 0;
		v0_[3] = vf;

		x0_[0] = x(0);
		// No 2nd and 3rd stage
		x0_[1] = 0;
		x0_[2] = 0;
		x0_[3] = x0_[1] + v0_[0]*t[0];

		a0_[0] = x(2);
		// No 2nd and 3rd stage
		a0_[1] = 0;
		a0_[2] = 0;
		a0_[3] = 0;
	}

	else{
		j_[0] = j_temp;
		j_[1] = 0;
		j_[2] = -j_temp;

		double t1 = -x(2)/j_temp + std::sqrt(0.5*pow(x(2),2) - j_temp*(x(1)-vf))/j_temp;
		double t2 = -x(2)/j_temp - std::sqrt(0.5*pow(x(2),2) - j_temp*(x(1)-vf))/j_temp;

		t1 = std::max(t1,t2);

		// Check to see if we'll saturate
		double a1f = x(2) + j_max_*t1;

		if (std::abs(a1f) > a_max_){
			double am = copysign(a_max_,j_temp);
			t[0] = (am-x(2))/j_[0];
			t[2] = -am/j_[2];

			a0_[0] = x(2);
			a0_[1] = a0_[0] + j_[0]*t[0];
			a0_[2] = am;
			a0_[3] = 0;

			v0_[0] = x(1);
			v0_[1] = v0_[0] + a0_[0]*t[0] + 0.5*j_[0]*pow(t[0],2);	
			v0_[2] = vf - am*t[2] - 0.5*j_[2]*pow(t[0],2);
			v0_[3] = vf;

			t[1] = (v0_[2]-v0_[1])/am;		

			x0_[0] = x(0);
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
			t[2] = -(x(2)+j_[0]*t1)/j_[2];

			a0_[0] = x(2);
			a0_[1] = 0; // No second phase
			a0_[2] = a0_[0] + j_[0]*t[0];
			a0_[3] = 0;

			v0_[0] = x(1);
			v0_[1] = 0; // No second phase
			v0_[2] = v0_[0] + a0_[0]*t[0] + 0.5*j_[0]*pow(t[0],2);
			v0_[3] = vf;		

			x0_[0] = x(0);
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


void REACT::eval_trajectory(Eigen::MatrixXd& Xc, Eigen::Matrix4d X0, Eigen::Matrix4d Y0, std::vector<double> t_x, std::vector<double> t_y, double t){
	// Eval x trajectory
	int k = 0;
	if (t < t_x[0]){
		k = 0;
		tx_ = t;
	}
	else if (t < t_x[0]+t_x[1]){
		tx_ = t - t_x[0];
		k = 1;
	}
	else if (t < t_x[0]+t_x[1]+t_x[2]){
		tx_ = t - (t_x[0]+t_x[1]);
		k = 2;
	}
	else{
		tx_ = t - (t_x[0]+t_x[1]+t_x[2]);
		k = 3;
	}

	// Eval y trajectory
	int l = 0;
	if (t < t_y[0]){
		ty_ = t;
		l = 0;
	}
	else if (t < t_y[0]+t_y[1]){
		ty_ = t - t_y[0];
		l = 1;
	}
	else if (t < t_y[0]+t_y[1]+t_y[2]){
		ty_ = t - (t_y[0]+t_y[1]);
		l = 2;
	}
	else{
		ty_ = t - (t_y[0]+t_y[1]+t_y[2]);
		l = 3;
	}

	Xc(0,0) = X0(0,k) + X0(1,k)*tx_ + 0.5*X0(2,k)*pow(tx_,2) + 1.0/6.0*X0(3,k)*pow(tx_,3);
	Xc(1,0) = 			X0(1,k)   +     X0(2,k)*tx_        +     0.5*X0(3,k)*pow(tx_,2);
	Xc(2,0) = 					        X0(2,k)          +         X0(3,k)*tx_;

	Xc(0,1) = Y0(0,l) + Y0(1,l)*ty_ + 0.5*Y0(2,l)*pow(ty_,2) + 1.0/6.0*Y0(3,l)*pow(ty_,3);
	Xc(1,1) = Y0(1,l) + Y0(2,l)*ty_ + 0.5*Y0(3,l)*pow(ty_,2);
	Xc(2,1) = Y0(2,l) + Y0(3,l)*ty_;
}



void REACT::scanCB(const sensor_msgs::LaserScan& msg)
 {
 	msg_received_ = ros::Time::now().toSec();
 	convert_scan(msg, scanE_, scanV_);
 	// Cluster
 	partition_scan(scanE_, Goals_, partition_, pose_, goal_);
 	// Sort clusters
 	sort_clusters(Sorted_Goals_, last_goal_, Goals_, pose_, goal_);
 	// Pick cluster

 	std::cout << "Latency: " << ros::Time::now().toSec() - msg_received_ << std::endl;

 	if(debug_){
 		convert2ROS(Goals_);
 		pubROS();
 	}
 	
 }

 void REACT::convert2ROS(Eigen::MatrixXd Goals){
 	goal_points_ros_.header.stamp = ros::Time::now();
 	goal_points_ros_.header.frame_id = "world";

 	for (int i=0; i < Goals.cols(); i++){
 		temp_goal_point_ros_.position.x = Goals(i,0);
 		temp_goal_point_ros_.position.y = Goals(i,1);
 		temp_goal_point_ros_.position.z = Goals(i,2);

 		temp_goal_point_ros_.orientation.w = cos(Goals(i,4)/2 + yaw_) ;
 		temp_goal_point_ros_.orientation.z = sin(Goals(i,4)/2 + yaw_);


 		goal_points_ros_.poses.push_back(temp_goal_point_ros_);
	}
 }

void REACT::pubROS(){
	int_goal_pub.publish(goal_points_ros_);
}


 void  REACT::pick_cluster( Eigen::MatrixXd Sorted_Goals, Eigen::Vector3d& local_goal, Eigen::MatrixXd Xc){
 	// Iterate through and pick cluster based on collision check
 }

// We might not need this...
 void REACT::convert_scan(sensor_msgs::LaserScan msg, Eigen::MatrixXd& scanE , std::vector<double>& scanV ){
 	angle_max_ = msg.angle_max;
	angle_min_ = msg.angle_min;
	angle_increment_ = msg.angle_increment;

	num_samples_ = (int) std::floor((angle_max_ - angle_min_) / angle_increment_ );

	scanE.resize(2,num_samples_);

	// Seems really inefficient
	for (int i=0;i<num_samples_;i++){
		if (!isinf(msg.ranges[i]) && !isnan(msg.ranges[i])){
			scanE(0,i) = angle_min_ + i*angle_increment_;
			scanE(1,i) = msg.ranges[i];
			scanV.push_back(msg.ranges[i]);
		}
		else{
			scanE(0,i) = angle_min_ + i*angle_increment_;
			scanE(1,i) = msg.range_max;
			scanV.push_back(msg.range_max);
		}
	}
}


void REACT::collision_check(Eigen::MatrixXd X, Eigen::MatrixXd scan, Eigen::Vector3d goal, double buff, double v, bool& can_reach_goal){
	// Find angle to goal

	angle_2_goal_ = atan2( goal(1) - X(0,1), goal(0) - X(0,0));
	vfx_ = v*cos(angle_2_goal_);
	vfy_ = v*sin(angle_2_goal_);

	x_ << X.col(0);
	y_ << X.col(1);

	find_times(t_x_, X0_, x_, vfx_);
	find_times(t_y_, Y0_, y_, vfy_);

	t_ = 0;
	dt_ = 0.01;
	collision_counter_ = 0;
	T_ = 1;

	int num = (int) T_/dt_;

	max_angle_ = scan.row(0).maxCoeff();
	min_angle_ = scan.row(0).minCoeff();
	d_angle_ = scan(0,1)-scan(0,0);

	Xc_ = X;

	for(int i=0; i<num; i++){
		eval_trajectory(Xc_,X0_,Y0_,t_x_,t_y_,t_);

		// These need to be in body frame
		r_ = sqrt(pow(Xc_(0,0)-X(0,0),2) + pow(Xc_(0,1)-X(0,1),2));
		theta_ = atan2(Xc_(0,1)-X(0,1),Xc_(0,0)-X(0,0)) - heading_;

		if (r_ > 2*buff){
			d_theta_ = sin(buff/r_);
		}
		else{
			d_theta_ = PI/2;
		}

		theta_1_ = theta_ - d_theta_;
		theta_2_ = theta_ + d_theta_;


		// Make sure they're in bounds
		saturate(theta_1_,min_angle_, max_angle_);
		saturate(theta_2_,min_angle_, max_angle_);

		index1_ = (int) std::floor((theta_1_ - min_angle_)/d_angle_);
		index2_ = (int) std::floor((theta_2_ - min_angle_)/d_angle_);


		d_min_ = scan.block(1,index1_,1,index2_-index1_).minCoeff();

		if (r_ > d_min_){
			collision_counter_++;
		}

		t_+=dt_;

		if (collision_counter_ > 9){
			break;
		}
	}

	if (collision_counter_ < 9) {
		can_reach_goal = true;
	}
	else {
		can_reach_goal = false;
	}
}

// Use this one
void REACT::collision_check2(Eigen::MatrixXd X, std::vector<double> scan, Eigen::Vector3d goal, double buff, double v, bool& can_reach_goal){
	// Find angle to goal

	angle_2_goal_ = atan2( goal(1) - X(0,1), goal(0) - X(0,0));
	vfx_ = v*cos(angle_2_goal_);
	vfy_ = v*sin(angle_2_goal_);

	x_ << X.col(0);
	y_ << X.col(1);

	find_times(t_x_, X0_, x_, vfx_);
	find_times(t_y_, Y0_, y_, vfy_);

	t_ = 0;
	dt_ = 0.1;
	collision_counter_ = 0;
	double t1 = t_x_[0] + t_x_[1] + t_x_[2];
	double t2 = t_y_[0] + t_y_[1] + t_y_[2];

	// T_ = std::max(t1,t2);
	T_ = 1;

	int num = (int) T_/dt_;


	// These should not be hard-coded
	max_angle_ = 2;
	min_angle_ = -2;
	d_angle_ = 0.00628;

	Xc_ = X;

	// // Find closest obstacle to me
	// d_min_ = *std::min_element(scan.begin(),scan.begin());
	// std::cout << "closest obstacle: " << d_min_ << std::endl;

	// No need to do collision checking until 

	for(int i=0; i<num; i++){
		eval_trajectory(Xc_,X0_,Y0_,t_x_,t_y_,t_);

		// These need to be in body frame
		r_ = sqrt(pow(Xc_(0,0)-X(0,0),2) + pow(Xc_(0,1)-X(0,1),2));
		theta_ = atan2(Xc_(0,1)-X(0,1),Xc_(0,0)-X(0,0)) - heading_;

		if (r_ > buff){
			d_theta_ = sin(buff/r_);
		}
		else{
			d_theta_ = PI/2;
			// Buffer is bigger dimension, check against that
			r_ = buff;
		}

		theta_1_ = theta_ - d_theta_;
		theta_2_ = theta_ + d_theta_;

		// Make sure they're in bounds
		saturate(theta_1_,min_angle_, max_angle_);
		saturate(theta_2_,min_angle_, max_angle_);

		index1_ = (int) std::floor((theta_1_ - min_angle_)/d_angle_);
		index2_ = (int) std::floor((theta_2_ - min_angle_)/d_angle_);

		// Find min distance in this angle range
		d_min_ = *std::min_element(scan.begin()+index1_,scan.begin()+index2_);

		if (r_ > d_min_){
			collision_counter_++;
		}

		if (collision_counter_ > 9){
			break;
		}

		t_+=dt_;
	}

	if (collision_counter_ < 9) {
		can_reach_goal = true;
	}
	else {
		can_reach_goal = false;
	}
}
 
void REACT::partition_scan(Eigen::MatrixXd scan, Eigen::MatrixXd& Goals, int& partition, Eigen::Vector3d pose, Eigen::Vector3d goal){
 	int j = 0;
 	double sum = 0;
 	double angle_2_index;

 	std::vector<double> r;
 	std::vector<double> r_temp;
 	std::vector<double> angle;
 	std::vector<double> angle_temp;

 	min_angle_ = scan(0,0);
 	d_angle_ = scan(0,1)-scan(0,0);

 	num_samples_ = scan.cols(); 
 	num_of_clusters_ = 0;

 	// Clean up logic
    for (int i=0; i < num_samples_-1; i++){
    	if (std::abs(scan(1,i+1)-scan(1,j)) < 0.5 && i!=num_samples_-2){
    			sum += scan(1,i+1);  
    	}
    	else if ((std::abs(scan(1,i+1)-scan(1,j)) > 0.5) || i==num_samples_-2){

    		if ((i-j)>10){

    			// Convert angle segment incerement to index
    			angle_2_index = angle_seg_inc_/angle_increment_;

    			// Probably a better way to do this...
    			// Numbers slightly arbitrary
    			if (double((i-j))/(3*angle_2_index) < 1) num_of_clusters_ = 2;
    			if (double((i-j))/(3*angle_2_index) > 1) num_of_clusters_ = 3;
    			if (double((i-j))/(5*angle_2_index) > 1) num_of_clusters_ = 5;
    			if (double((i-j))/(7*angle_2_index) > 1) num_of_clusters_ = 7;
    			if (double((i-j))/(9*angle_2_index) > 1) num_of_clusters_ = 9;
    			if (double((i-j))/(11*angle_2_index) > 1) num_of_clusters_ = 11;

    			for (int k=0; k < num_of_clusters_; k++){
    				r_temp.push_back(sum/(i-j));

    				angle_temp.push_back(min_angle_ + heading_ + d_angle_*(j + k*(i-j)/(num_of_clusters_-1)));
    			}
    		}
    			sum = 0;
    			j = i+1;   			
		}
	}

	Goals = Eigen::MatrixXd::Zero(r_temp.size(),5);
	int count = 0;

	// This is wrong, i >= count
	for (int i = 0; i < r_temp.size(); i++){
		if (r_temp[i] > safe_distance_){
			r.push_back(r_temp[i]);
			angle.push_back(angle_temp[i]);

			Goals(count,0) = r[i]*cos(angle[i]) + pose(0);
			Goals(count,1) = r[i]*sin(angle[i]) + pose(1);
			Goals(count,2) = goal(2);
			Goals(count,3) = r[i];
			Goals(count,4) = angle[i];
			count++;
		}
	}

	partition = r_temp.size();
}



void REACT::sort_clusters(Eigen::MatrixXd& Sorted_Goals, Eigen::Vector3d last_goal, Eigen::MatrixXd Goals,  Eigen::Vector3d pose, Eigen::Vector3d goal){

 	// Re-initialize
	cost_queue_ = std::priority_queue<double, std::vector<double>, std::greater<double> > ();
	Sorted_Goals = Eigen::MatrixXd::Zero(Goals.rows()+1,Goals.cols()+1);
	cost_v_.clear();

	last_goal_V_ = last_goal - pose;
	last_goal_V_ = last_goal_V_/last_goal_V_.norm();

	r_goal_ = (goal - pose).norm();
	angle_2_goal_ = atan2( goal(1) -pose(1), goal(0) - pose(0)) - yaw_;

	num_of_clusters_ = Goals.rows();

 	for (int i=0; i < num_of_clusters_ ; i++){

 		next_goal_V_ = Goals.block(i,0,1,3).transpose() - pose;

 		r_i_ = next_goal_V_.norm();
 		angle_i_ = atan2 ( Goals(i,1) - pose(1), Goals(i,0) - pose(0) ) - yaw_;
 		
 		angle_diff_  =  std::abs(angle_i_)  - std::abs(angle_2_goal_ );

 		// Normalize
		next_goal_V_ = next_goal_V_/next_goal_V_.norm();

 		angle_diff_last_ = next_goal_V_.dot(last_goal_V_);

 		cost_i_ = pow(angle_diff_,2) + pow(angle_diff_last_,2) + pow(1/r_i_,2);

 		cost_queue_.push(cost_i_);
 		cost_v_.push_back(cost_i_);

 	}

 	// There should be a better way to do this
 	Sorted_Goals.row(0) << goal(0), goal(1), goal(2), r_goal_, angle_2_goal_, 0; 

 	for (int i=0; i < num_of_clusters_ ; i++){

	 	min_cost_ = cost_queue_.top();

		it_ = std::find(cost_v_.begin(),cost_v_.end(),min_cost_);
		goal_index_ = it_ - cost_v_.begin();

		Sorted_Goals.row(i+1) << Goals.row(goal_index_), min_cost_;

		cost_queue_.pop();
	}
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
