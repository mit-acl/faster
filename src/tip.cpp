#include "tip.hpp"

TIP::TIP() : tf_listener_(tf_buffer_) {

	std::string name = ros::this_node::getNamespace();
	// Erase slashes
	name.erase(0,2);

	// Should be read as param
	ros::param::get("~debug",debug_);
	ros::param::get("~use_memory",use_memory_);
	ros::param::get("~safe_distance",safe_distance_);
	ros::param::get("~buffer",buffer_);
	ros::param::get("~sensor_distance",sensor_distance_);
	ros::param::get("~mem_distance",mem_distance_);


	last_goal_ = Eigen::Vector3d::Zero();
	pose_= Eigen::Vector3d::Zero();
	goal_ = Eigen::Vector3d::Zero();
	X_ = Eigen::MatrixXd::Zero(4,3);
	X_stop_ = Eigen::MatrixXd::Zero(4,3);
	XE_ = Eigen::MatrixXd::Zero(4,3);

	ros::param::get("~goal_x",goal_(0));
	ros::param::get("~goal_y",goal_(1));
	ros::param::get("~goal_z",goal_(2));

	heading_ = atan2(goal_(1),goal_(0));
	angle_2_last_goal_ = heading_;
	local_goal_angle_ = heading_;
	last_goal_ << goal_;
	local_goal_ << goal_;

	ros::param::get("cntrl/spinup_time",spinup_time_);
	ros::param::get("~max_speed",v_max_);

	ros::param::get("~jerk",j_max_);
	ros::param::get("~accel",a_max_);
	ros::param::get("~accel_stop",a_stop_);

	ros::param::get("~plan_eval",plan_eval_time_);

	ros::param::get("~K",K_);

	ros::param::get("~h_fov",h_fov_);
	ros::param::get("~h_samples",h_samples_);
	ros::param::get("~v_fov",v_fov_);
	ros::param::get("~v_samples",v_samples_);

	ros::param::get("~r_max",r_max_);
	ros::param::get("~jump_thresh",jump_thresh_);

	ros::param::get("~z_min",z_min_);
	ros::param::get("~z_max",z_max_);
	
	h_fov_ = h_fov_*M_PI/180;
	v_fov_ = v_fov_*M_PI/180;

	// Generate allowable final states
	sample_ss(Goals_);

	dist_trav_last_ = 0;
	last_prim_cost_ = 100;
	dist_safe_last_ = sensor_distance_;
	pose_last_mp_= Eigen::Vector3d::Zero();

	bias_x_=0;
	bias_y_=0;
	bias_z_=0;

	v_ = v_max_;
	yaw_ = 0;

	yawing_ = false;
	stop_ = false;
	still_clear_ = true;
	can_reach_global_goal_ = true;
	can_reach_goal_ = false;
	following_prim_ = false;

	quad_status_ = state_.NOT_FLYING;

	quad_goal_.cut_power = true;
	// quad_goal_.mode = acl_msgs::QuadGoal::MODE_VEL;
	quad_goal_.xy_mode = acl_msgs::QuadGoal::MODE_POS;
	quad_goal_.z_mode = acl_msgs::QuadGoal::MODE_POS;

	inf = std::numeric_limits<double>::max();

	// wait for body transform to be published before initializing
	ROS_INFO("Waiting for world to body transform...");
	while (true) {
	  try {
	    tf_buffer_.lookupTransform("world", name, ros::Time::now(), ros::Duration(1.0));
	    break;
	  } catch (tf2::TransformException &ex) {
	    // nothing
	  }
	}
	ROS_INFO("Planner initialized.");
}

void TIP::global_goalCB(const geometry_msgs::PointStamped& msg){
	goal_ << msg.point.x, msg.point.y, msg.point.z;
	heading_ = atan2(goal_(1)-X_(0,1),goal_(0)-X_(0,0));
}

void TIP::stateCB(const acl_msgs::ViconState& msg)
{
	// Check if estimate jumped 
	if (sqrt(pow(pose_(0)-msg.pose.position.x,2) + pow(pose_(1)- msg.pose.position.y,2) + pow(pose_(2)- msg.pose.position.z,2)) > jump_thresh_){
		ROS_WARN("Jump Detected -- magnitude: %0.3f",sqrt(pow(pose_(0)-msg.pose.position.x,2) + pow(pose_(1)- msg.pose.position.y,2) + pow(pose_(2)- msg.pose.position.z,2)));
		if (quad_status_ == state_.GO){
			bias_x_ = msg.pose.position.x-pose_(0);
			bias_y_ = msg.pose.position.y-pose_(1);
			bias_z_ = msg.pose.position.z-pose_(2);
			X_.row(0) << X_(0,0)+bias_x_,X_(0,1)+bias_y_,X_(0,2)+bias_z_;
			gen_new_traj_ = true;
		}
	}

	pose_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z; 

	qw2b_.w() = msg.pose.orientation.w;
	qw2b_.vec() << -msg.pose.orientation.x,-msg.pose.orientation.y,-msg.pose.orientation.z;

	tf::quaternionMsgToTF(msg.pose.orientation, att_);

	// Check this
	if (quad_status_ == state_.NOT_FLYING){
		X_.row(0) << pose_.transpose();
	}
}

void TIP::sendGoal(const ros::TimerEvent& e)
{	
	if (gen_new_traj_){
		gen_new_traj_ = false;
		v_plan_ = v_;
		mtx.lock();
		if (stop_) get_traj(X_,local_goal_,v_,t_xf_,t_yf_,t_zf_,Xf_switch_,Yf_switch_,Zf_switch_,true);
		else get_traj(X_,local_goal_,v_,t_xf_,t_yf_,t_zf_,Xf_switch_,Yf_switch_,Zf_switch_,false);
		mtx.unlock();
		t0_ = ros::Time::now().toSec() - plan_eval_time_;
	}

	if (quad_status_== state_.TAKEOFF){
		takeoff(X_(0,2));
		if (X_(0,2) == goal_(2)){
			quad_status_ = state_.FLYING;
			ROS_INFO("Take-off Complete");
		}
	}

	else if (quad_status_== state_.LAND){
		if (X_.block(1,0,1,2).norm() == 0){
			land(X_(0,2));
			if (X_(0,2) == -0.1){
				quad_status_ = state_.NOT_FLYING;
				quad_goal_.cut_power = true;
				ROS_INFO("Landing Complete");
			}
		}
	}

	else if (quad_status_ == state_.GO){			
		double diff = heading_ - quad_goal_.yaw;
		// diff =  fmod(diff+M_PI,2*M_PI);
	    if (diff < 0)
	        diff += 2*M_PI;
	    diff -= M_PI;
	    diff = 0;
		if(fabs(diff)>0.02 && !stop_){
			if (!yawing_){
				if (fabs(diff)>M_PI/2 || X_.block(1,0,1,3).norm()==0) {v_ = 0; gen_new_traj_=true;}
				else if (!stop_) v_ = v_max_;
				else {v_ = 0; gen_new_traj_ = true;}
			}
			// Only yaw if the vehicle is at right speed
			if (X_.block(1,0,1,3).norm() <= 1.1*v_ && X_.block(1,0,1,3).norm() >= 0.9*v_){
				yawing_ = true;
				saturate(diff,-plan_eval_time_*r_max_,plan_eval_time_*r_max_);
				if (diff>0) quad_goal_.dyaw =  r_max_;
				else        quad_goal_.dyaw = -r_max_;
				quad_goal_.yaw+=diff;
			}				
			else{
				yawing_=false;
			}			
		}
		else if (!stop_) {
			v_ = v_max_;
			quad_goal_.dyaw=0;
			yawing_ = false;
		} 	

		if (!stop_){
			get_stop_dist(X_,goal_,can_reach_global_goal_,stop_);
			if (stop_){
				v_ = 0;
				gen_new_traj_ = true;
		 		ROS_INFO_THROTTLE(1.0,"stopping");
			}
		}

		tE_ = ros::Time::now().toSec() - t0_;
		
		mtx.lock();		
		eval_trajectory(Xf_switch_,Yf_switch_,Zf_switch_,t_xf_,t_yf_,t_zf_,tE_,X_);
		mtx.unlock();

		if (X_.row(1).norm() == 0 && stop_){
			// We're done	
			stop_ = false;
			if ((goal_.head(2)-X_.block(0,0,1,2).transpose()).norm() < 5){ ROS_INFO("Flight Complete"); quad_status_ = state_.FLYING; }
			else {v_ = v_max_; gen_new_traj_ = true;};
		}
	}

	eigen2quadGoal(X_,quad_goal_);
	yaw_ = quad_goal_.yaw;
	quad_goal_.header.stamp = ros::Time::now();
	quad_goal_.header.frame_id = "world";
	quad_goal_pub.publish(quad_goal_);
}


void TIP::eventCB(const acl_msgs::QuadFlightEvent& msg)
{
	// Takeoff
	if (msg.mode == msg.TAKEOFF && quad_status_== state_.NOT_FLYING){
		ROS_INFO("Waiting for spinup");
		quad_goal_.pos.x = pose_(0);
		quad_goal_.pos.y = pose_(1);
		quad_goal_.pos.z = pose_(2);

		X_(0,0) = pose_(0);
		X_(0,1) = pose_(1);
		X_(0,2) = pose_(2);

		quad_goal_.vel.x = 0;
		quad_goal_.vel.y = 0;
		quad_goal_.vel.z = 0;

		quad_goal_.yaw = yaw_;
		quad_goal_.dyaw = 0;

		quad_goal_.cut_power = false;

		ros::Duration(spinup_time_).sleep();
		ROS_INFO("Taking off");

		quad_status_ = state_.TAKEOFF; 
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
		double diff = heading_ - quad_goal_.yaw;
		diff =  fmod(diff+M_PI,2*M_PI) - M_PI;
		if (diff < 0)
	        diff += 2*M_PI;
	    diff -= M_PI;
		while(std::abs(diff)>0.001){
			saturate(diff,-0.002*r_max_,0.002*r_max_);
			if (diff>0) quad_goal_.dyaw =  r_max_;
			else        quad_goal_.dyaw = -r_max_;
			quad_goal_.yaw+=diff;	
			diff = heading_ - quad_goal_.yaw;
			diff =  fmod(diff+M_PI,2*M_PI) - M_PI;	
			ros::Duration(0.002).sleep();
		}
		quad_goal_.dyaw = 0;
		ROS_INFO("Initialized");
	}
	// GO!!!!
	else if (msg.mode == msg.START && quad_status_ == state_.FLYING){
		// Set speed to desired speed
		v_ = v_max_;
		// Generate new traj
		gen_new_traj_ = true;
		quad_status_ = state_.GO;
		ROS_INFO("Starting");
		
	}
	// STOP!!!
	else if (msg.mode == msg.ESTOP && quad_status_ == state_.GO){
		ROS_WARN("Stopping");
		// Stay in go command but set speed to zero
		v_ = 0;
		stop_ = true;
		quad_goal_.dyaw = 0;
		// Generate new traj
		gen_new_traj_ = true;
		
	}
}

void TIP::pclCB(const sensor_msgs::PointCloud2ConstPtr& msg)
 {
 	msg_received_ = ros::WallTime::now().toSec();
 	
 	// Convert pcl
	convert2pcl(msg,cloud_);

	int size = cloud_->points.size();
	int i;
	int count = 0;
	for(i=0;i<size;i++){
		// nan is the only number that doesn't equal itself -- John Carter
		if (cloud_->points[i].x == cloud_->points[i].x){
			count++;
			// Ensure there are enough good points to perform collision detection
			if (count > K_) break;
		}
	}

 	if (i!=size && quad_status_!=state_.NOT_FLYING){
		// Build k-d tree
		kdtree_.setInputCloud(cloud_);
		
		// Sort allowable final states
		sort_ss(Goals_,X_.row(0).transpose(),goal_, last_goal_, Sorted_Goals_,v_los_);

		// Pick desired final state
		pick_ss(Sorted_Goals_, X_, can_reach_goal_);
		
		if (!v_los_) following_prim_ = true;

		// Check if current primitive is still collision free
		if (following_prim_ && v_plan_ > 0) {
			check_current_prim(Xf_switch_,Yf_switch_,Zf_switch_,t_xf_,t_yf_,t_zf_,tE_,X_,still_clear_);
			// ROS_INFO("Still clear: %i",still_clear_);		
		}

	 	if (still_clear_ && v_plan_ > 0 && use_memory_ && !stop_ && dist_trav_last_ < mem_distance_ && min_cost_prim_ > last_prim_cost_ && quad_status_== state_.GO){
			// Update distance traveled
			dist_trav_last_ = (X_.block(0,0,1,3).transpose() - pose_last_mp_).norm();
			if (!following_prim_) {count2 = 0; ROS_INFO("following primitive");}
			// ROS_INFO("%i",count2);
			count2++;
			following_prim_ = true;
		}

		else{
			if (following_prim_) {ROS_INFO("not following primitive"); ROS_INFO("Distance traveled: %0.3f",dist_trav_last_);}
			following_prim_ = false;
			still_clear_ = true;
			if (!can_reach_goal_ && quad_status_==state_.GO){
		 		// Need to stop!!!
		 		v_ = 0;
		 		stop_ = true;
		 		ROS_ERROR_THROTTLE(1.0,"Emergency stop -- no feasible path");
		 	}
		 	else if (v_los_ && !stop_ && !yawing_) v_ = v_max_;
			
		 	gen_new_traj_ = true;
		 	pose_last_mp_ = X_.block(0,0,1,3).transpose();
		 	dist_safe_last_ = distance_traveled_;
		 	dist_trav_last_ = 0;
			last_prim_cost_ = min_cost_prim_ ;
	 	}
	 	// std::cout << "Latency (ms): " << 1000*(traj_gen_ - msg_received_) << std::endl;		
	 }
	else {
		// Generate traj
		local_goal_ = goal_ - X_.row(0).transpose();
		gen_new_traj_ = true;

		pose_last_mp_ = X_.block(0,0,1,3).transpose();
		dist_safe_last_ = distance_traveled_;
		dist_trav_last_ = 0;
		last_prim_cost_ = min_cost_prim_ ;
	}

 	tipData_.header.stamp = ros::Time::now();
 	tipData_.latency = 1000*(ros::WallTime::now().toSec() - msg_received_);
 	tipData_.speed = X_.row(1).norm();
 	tipData_.follow_prim = following_prim_;

 	saturate(min_cost_prim_,0,5);

 	if (quad_status_!=state_.GO || stop_) tipData_.prim_cost = 0;
 	else if (following_prim_) tipData_.prim_cost = last_prim_cost_;
    else tipData_.prim_cost = min_cost_prim_;

 	if(debug_){
 		convert2ROS();
 		pubROS();
 	} 	
 } 	


void TIP::check_current_prim(Eigen::Matrix4d X0, Eigen::Matrix4d Y0, Eigen::Matrix4d Z0, std::vector<double> t_x, std::vector<double> t_y, std::vector<double> t_z, double t, Eigen::MatrixXd X, bool& clear){
	clear = false;
	collision_detected_ = false;

	X_prop_ = Eigen::MatrixXd::Zero(4,3);
	X_prop_ << X;

	searchPoint_.x = X(0,0);
	searchPoint_.y = X(0,1);
	searchPoint_.z = X(0,2);

	std::vector<int> pointIdxNKNSearch(K_);
  	std::vector<float> pointNKNSquaredDistance(K_);

  	kdtree_.nearestKSearch(searchPoint_, K_, pointIdxNKNSearch, pointNKNSquaredDistance);	
    mean_distance_ = std::sqrt(std::accumulate(pointNKNSquaredDistance.begin(), pointNKNSquaredDistance.end(), 0.0f)/pointIdxNKNSearch.size());

    // If the obstacle is farther than safe distance or goal is within mean distance then we're good
    if (mean_distance_ > safe_distance_){
    	clear = true;
    }
    // Something else is closer, need to prop to next time step
	else{
		// evaluate at time required to travel d_min
		t_ = std::max(buffer_/v_max_,mean_distance_/v_max_) + t;

		while (!collision_detected_ && !clear){

			mtx.lock();
			eval_trajectory(X0,Y0,Z0,t_x,t_y,t_z,t_,X_prop_);
			mtx.unlock();

			searchPoint_.x = X_prop_(0,0);
			searchPoint_.y = X_prop_(0,1);
			searchPoint_.z = X_prop_(0,2);

		  	kdtree_.nearestKSearch(searchPoint_, K_, pointIdxNKNSearch, pointNKNSquaredDistance);	

    		mean_distance_ = std::sqrt(std::accumulate(pointNKNSquaredDistance.begin(), pointNKNSquaredDistance.end(), 0.0f)/pointIdxNKNSearch.size());

    		distance_traveled_ = (X_prop_.row(0)-X.row(0)).norm();

    		// Check if the distance is less than our buffer
			if (mean_distance_ < buffer_){
				collision_detected_ = true;
				clear = false;
			}
			// Check if the min distance is the current goal
			else if (distance_traveled_ > safe_distance_){
					clear = true;
					distance_traveled_ = sensor_distance_;
			}			
			// Neither have happened so propogate again
			else{
				t_ += mean_distance_/v_max_;
			}
		}
	}
}


void TIP::sample_ss(Eigen::MatrixXd& Goals){
	theta_ = Eigen::VectorXd::Zero(h_samples_);
	theta_.setLinSpaced(h_samples_,-h_fov_/2,h_fov_/2);
	
	if (v_samples_==0) v_samples_=1;
	phi_ = Eigen::VectorXd::Zero(v_samples_);
	phi_.setLinSpaced(v_samples_,-v_fov_/2,v_fov_/2);

	Goals = Eigen::MatrixXd::Zero((h_samples_)*(v_samples_),3);
	proj_goals_ = Eigen::MatrixXd::Zero(Goals_.rows(),Goals_.cols());
 	Eigen::VectorXd x;
 	Eigen::VectorXd y;
 	Eigen::VectorXd z;
 	x = Eigen::VectorXd::Zero(Goals_.rows());
 	x.setConstant(3);
 	y.setLinSpaced(h_samples_,-3*tan(h_fov_/2),3*tan(h_fov_/2));
 	z.setLinSpaced(v_samples_,-3*tan(v_fov_/2),3*tan(v_fov_/2));

	int k = 0;
	for(int j=0; j < v_samples_; j++){
		for (int i=0; i < h_samples_; i++){
			Goals.row(k) << cos(theta_(i))*cos(phi_(j)), sin(theta_(i))*cos(phi_(j)), sin(phi_(j));
			proj_goals_.row(k) << x(k),y(i),z(j);
			k++;
		}
	}
}

void TIP::sort_ss(Eigen::MatrixXd Goals, Eigen::Vector3d pose, Eigen::Vector3d goal, Eigen::Vector3d vector_last, Eigen::MatrixXd& Sorted_Goals, bool& v_los){
 	// Re-initialize
	cost_queue_ = std::priority_queue<double, std::vector<double>, std::greater<double> > ();
	cost_v_.clear();

	vector_2_goal_= goal-pose ;
	vector_2_goal_.normalize();

	vector_2_goal_body_ = qw2b_._transformVector(vector_2_goal_);

	Eigen::Vector3d temp = vector_last-pose;
	temp.normalize();
	vector_last_body_ = qw2b_._transformVector(temp);

	num_of_pnts_ = Goals.rows();

 	for (int i=0; i < num_of_pnts_ ; i++){
		vector_i_ << Goals.row(i).transpose();
		angle_diff_ = acos(vector_i_.dot(vector_2_goal_body_));

		angle_diff_last_ = acos(vector_i_.dot(vector_last_body_));

 		cost_i_ = pow(angle_diff_,2) + pow(angle_diff_last_,2);

 		cost_queue_.push(cost_i_);
 		cost_v_.push_back(cost_i_);

 	}

 	Eigen::MatrixXd Sorted_Goals_temp;
 	Sorted_Goals_temp = Eigen::MatrixXd::Zero(Goals.rows(),Goals.cols()+1);

 	for (int i=0; i < num_of_pnts_ ; i++){
	 	min_cost_ = cost_queue_.top();

		it_ = std::find(cost_v_.begin(),cost_v_.end(),min_cost_);
		goal_index_ = it_ - cost_v_.begin();
		Sorted_Goals_temp.row(i) << Goals.row(goal_index_), min_cost_;

		cost_queue_.pop();
	}

	 // Check if vector_2_goal_body_ is within FOV
 	double r, p, y;
 	tf::Matrix3x3(att_).getRPY(r, p, y);

 	double angle_v = p;

	double angle_h = heading_ - quad_goal_.yaw;
	angle_h =  fmod(angle_h+M_PI,2*M_PI);
    if (angle_h < 0)
        angle_h += 2*M_PI;
    angle_h -= M_PI;

    if (std::abs(angle_v) < v_fov_/2) v_los = true;
    else v_los = false;
   
    if (std::abs(angle_h) < h_fov_/2 && std::abs(angle_v) < v_fov_/2){
 		Sorted_Goals = Eigen::MatrixXd::Zero(Goals.rows()+1,Goals.cols()+1);
    	Sorted_Goals.row(0)<< vector_2_goal_body_.transpose(),0;
    	Sorted_Goals.block(1,0,Sorted_Goals.rows()-1,Sorted_Goals.cols()) << Sorted_Goals_temp;
 	}
 	else{
 		Sorted_Goals = Eigen::MatrixXd::Zero(Goals.rows(),Goals.cols()+1);
 		Sorted_Goals << Sorted_Goals_temp;
 	}
}

void TIP::pick_ss(Eigen::MatrixXd Sorted_Goals, Eigen::MatrixXd X, bool& can_reach_goal){
	goal_index_ = 0;
	can_reach_goal = false;

 	while(!can_reach_goal && goal_index_ < Sorted_Goals.rows()){
 		// Tranform temp local goal to world frame
 		Eigen::Vector4d temp_local_goal_aug;
 		temp_local_goal_aug << qw2b_.conjugate()._transformVector(Sorted_Goals.block(goal_index_,0,1,3).transpose()), Sorted_Goals(goal_index_,3);

		collision_check(X,buffer_,v_max_,can_reach_goal,temp_local_goal_aug); 	

		// Update cost
		Sorted_Goals(goal_index_,3) = temp_local_goal_aug(3);

 		goal_index_++;
 	}

 	int index;
	min_cost_prim_ = Sorted_Goals.col(3).minCoeff(&index);
	double min_cost = min_cost_prim_;

 	if (!can_reach_goal) {	
 		if (min_cost!=inf){
 			goal_index_ = index+1;
 			can_reach_goal = true;
 		} 
 	}

 	if(can_reach_goal){
 		goal_index_--;
 		// Tranform temp local goal to world frame
 		local_goal_ = qw2b_.conjugate()._transformVector(Sorted_Goals.block(goal_index_,0,1,3).transpose());
 		last_goal_ << local_goal_;
 		double d = (goal_.head(2)-X_.block(0,0,1,2).transpose()).norm();
 		if (goal_index_ == 0 && d < sensor_distance_) can_reach_global_goal_ = true;
 		else can_reach_global_goal_ = false;
 	}
}


void TIP::collision_check(Eigen::MatrixXd X, double buff, double v, bool& can_reach_goal, Eigen::Vector4d& local_goal_aug){
	//Re-intialize
	can_reach_goal = false;
	collision_detected_ = false;
	
	X_prop_ = Eigen::MatrixXd::Zero(4,3);
	X_prop_ << X;

	Eigen::Vector3d local_goal;
	local_goal << local_goal_aug.head(3);
	
	mtx.lock();
	get_traj(X,local_goal,v,t_x_,t_y_,t_z_,X_switch_,Y_switch_,Z_switch_,false);
	mtx.unlock();

	goal_distance_ = (goal_ - X.row(0).transpose()).norm();

	searchPoint_.x = X(0,0);
	searchPoint_.y = X(0,1);
	searchPoint_.z = X(0,2);

	std::vector<int> pointIdxNKNSearch(K_);
  	std::vector<float> pointNKNSquaredDistance(K_);

  	kdtree_.nearestKSearch(searchPoint_, K_, pointIdxNKNSearch, pointNKNSquaredDistance);	
    mean_distance_ = std::sqrt(std::accumulate(pointNKNSquaredDistance.begin(), pointNKNSquaredDistance.end(), 0.0f)/pointIdxNKNSearch.size());

    // If the obstacle is farther than safe distance or goal is within mean distance then we're good
    if (mean_distance_ > sensor_distance_ || mean_distance_ > goal_distance_){
    	can_reach_goal = true;
    }
    // Something else is closer, need to prop to next time step
	else{
		// evaluate at time required to travel d_min
		t_ = std::max(buff/v,mean_distance_/v);

		while (!collision_detected_ && !can_reach_goal){

			mtx.lock();
			eval_trajectory(X_switch_,Y_switch_,Z_switch_,t_x_,t_y_,t_z_,t_,X_prop_);
			mtx.unlock();

			searchPoint_.x = X_prop_(0,0);
			searchPoint_.y = X_prop_(0,1);
			searchPoint_.z = X_prop_(0,2);

		  	kdtree_.nearestKSearch(searchPoint_, K_, pointIdxNKNSearch, pointNKNSquaredDistance);	

    		mean_distance_ = std::sqrt(std::accumulate(pointNKNSquaredDistance.begin(), pointNKNSquaredDistance.end(), 0.0f)/pointIdxNKNSearch.size());

    		distance_traveled_ = (X_prop_.row(0)-X.row(0)).norm();

    		// Check if the distance is less than our buffer
			if (mean_distance_ < buff){
				collision_detected_ = true;
				can_reach_goal = false;
				// ROS_INFO("Distance traveled: %0.2f", distance_traveled_);
				if (distance_traveled_ < safe_distance_) local_goal_aug(3) = inf;
				else local_goal_aug(3) = 0.05*pow(sensor_distance_-distance_traveled_,2);

			}
			// Check if the min distance is the current goal
			else if (distance_traveled_ > sensor_distance_){
				// If traj is not within z bounds then it's not valid
				if (X_prop_(0,2) < z_min_ || X_prop_(0,2) > z_max_){
			    	can_reach_goal = false;
			    	local_goal_aug(3) = inf;
			    	return;
			    }
		    	else{
					can_reach_goal = true;
					distance_traveled_ = sensor_distance_;
				}
			}			
			// Neither have happened so propogate again
			else{
				t_ += mean_distance_/v;
			}
		}
	}
}

void TIP::get_traj(Eigen::MatrixXd X, Eigen::Vector3d local_goal, double v, std::vector<double>& t_fx, std::vector<double>& t_fy, std::vector<double>& t_fz, Eigen::Matrix4d& Xf_switch, Eigen::Matrix4d& Yf_switch, Eigen::Matrix4d& Zf_switch, bool stop_check ){
	//Generate new traj
	get_vels(X,local_goal,v,vfx_,vfy_,vfz_);

	x0_ << X.block(0,0,4,1);
	y0_ << X.block(0,1,4,1);
	z0_ << X.block(0,2,4,1);
	find_times(x0_, vfx_, t_fx, Xf_switch,stop_check);
	find_times(y0_, vfy_, t_fy, Yf_switch,stop_check);
	find_times(z0_, vfz_, t_fz, Zf_switch,stop_check);

	if (X.row(1).norm()==0){
		// Sync traj
		double tx = std::accumulate(t_fx.begin(), t_fx.end(), 0.0);
		double ty = std::accumulate(t_fx.begin(), t_fx.end(), 0.0);
		double tz = std::accumulate(t_fx.begin(), t_fx.end(), 0.0);
		double tmax = std::max(std::max(tx,ty),tz);

		sync_times(x0_, tmax, vfx_, t_fx, Xf_switch);
		sync_times(y0_, tmax, vfy_, t_fy, Yf_switch);
		sync_times(z0_, tmax, vfz_, t_fz, Zf_switch);
	}

}

void TIP::sync_times(Eigen::Vector4d x0, double tmax, double vf, std::vector<double>& t, Eigen::Matrix4d& X_switch){
	if (tmax == std::accumulate(t.begin(), t.end(), 0.0) || vf == 0) return;
	else {
		double j_temp = copysign(4*vf/pow(tmax,2),vf);
		double t1 = std::sqrt(vf/j_temp);

		j_V_[0] = j_temp;
		j_V_[1] = 0;
		j_V_[2] = -j_temp;


		t[0] = t1;
		t[1] = 0; // No second phase
		t[2] = t1;

		a0_V_[0] = x0(2);
		a0_V_[1] = 0; // No second phase
		a0_V_[2] = a0_V_[0] + j_V_[0]*t[0];
		a0_V_[3] = 0;

		v0_V_[0] = x0(1);
		v0_V_[1] = 0; // No second phase
		v0_V_[2] = v0_V_[0] + a0_V_[0]*t[0] + 0.5*j_V_[0]*pow(t[0],2);
		v0_V_[3] = vf;		

		x0_V_[0] = x0(0);
		x0_V_[1] = 0; // No second phase
		x0_V_[2] = x0_V_[0] + v0_V_[0]*t[0] + 0.5*a0_V_[0]*pow(t[0],2) + 1./6*j_V_[0]*pow(t[0],3);
		x0_V_[3] = x0_V_[2] + v0_V_[2]*t[2] + 0.5*a0_V_[2]*pow(t[2],2) + 1./6*j_V_[2]*pow(t[2],3);

		X_switch.row(0) << x0_V_[0],x0_V_[1],x0_V_[2],x0_V_[3];
		X_switch.row(1) << v0_V_[0],v0_V_[1],v0_V_[2],v0_V_[3];
		X_switch.row(2) << a0_V_[0],a0_V_[1],a0_V_[2],a0_V_[3];
		X_switch.row(3) << j_V_[0],j_V_[1],j_V_[2],j_V_[3];
	}
}


void TIP::get_stop_dist(Eigen::MatrixXd X, Eigen::Vector3d goal, bool can_reach_global_goal, bool& stop){
	if (can_reach_global_goal){
		Eigen::Vector3d vector_2_goal = goal - X.row(0).transpose() ;
		vector_2_goal.normalize();

		mtx.lock();
		get_traj(X,vector_2_goal,0,t_x_stop_,t_y_stop_,t_z_stop_,X_switch_stop_,Y_switch_stop_,Z_switch_stop_,true);
		mtx.unlock();

		t_stop_ = std::max(std::accumulate(t_x_stop_.begin(), t_x_stop_.end(), 0.0), std::accumulate(t_y_stop_.begin(), t_y_stop_.end(), 0.0));
		t_stop_ = std::max(t_stop_,std::accumulate(t_z_stop_.begin(),t_z_stop_.end(),0.0));

		mtx.lock();
		eval_trajectory(X_switch_stop_,Y_switch_stop_,Z_switch_stop_,t_x_stop_,t_y_stop_,t_z_stop_,t_stop_,X_stop_);
		mtx.unlock();

		// Double check this
		d_stop_ = (X_stop_.block(0,0,1,2) - X.block(0,0,1,2)).norm();
		d_goal_ = (X.block(0,0,1,2).transpose() - goal.head(2)).norm();

		// Prevents oscillation if our stopping distance is really small (low speed)
		saturate(d_stop_,0.1,d_stop_);

		if (d_stop_ >= d_goal_){
			// Need to stop
			stop = true;
		}
	}
}


void TIP::get_vels(Eigen::MatrixXd X, Eigen::Vector3d local_goal, double v, double& vx, double& vy, double& vz){
	Eigen::Vector3d temp = local_goal;
	temp.normalize();
	vx = v*temp(0);
	vy = v*temp(1);
	vz = v*temp(2);
}



void TIP::find_times(Eigen::Vector4d x0, double vf, std::vector<double>& t, Eigen::Matrix4d&  X_switch, bool stop_check){
 	if (vf == x0(1)){
 		j_V_[0] = 0;
		j_V_[1] = 0; 
		j_V_[2] = 0;

		t[0] = 0;
		t[1] = 0; 
		t[2] = 0;

		a0_V_[0] = 0;
		a0_V_[1] = 0; 
		a0_V_[2] = 0;
		a0_V_[3] = x0(2);

		v0_V_[0] = 0;
		v0_V_[1] = 0; 
		v0_V_[2] = 0;
		v0_V_[3] = x0(1);		

		x0_V_[0] = 0;
		x0_V_[1] = 0; 
		x0_V_[2] = 0;
		x0_V_[3] = x0(0);
 	}
 	else{
		
		double j_temp;
		double a_temp;
		if (stop_check){
			j_temp = j_max_;
			a_temp = a_stop_;
		}
		else{
			a_temp = a_max_;
			// Could be interesting, need to justify 
			if (std::abs(vf-x0(1))/v_max_ < 0.2 && std::abs(x0(3)) != j_max_ && std::abs(x0(2))!=a_max_){
				j_temp = 5;
			}
			else{
				j_temp = j_max_;
			}
		}
		j_temp = copysign(j_temp,vf-x0(1));

		double vfp = x0(1) + pow(x0(2),2)/(2*j_temp);

		if (std::abs(vfp-vf) < 0.02*std::abs(vf) && x0(2)*(vf-x0(1))>0){

			j_V_[0] = -j_temp;
			// No 2nd and 3rd stage
			j_V_[1] = 0;
			j_V_[2] = 0;

			t[0] = -x0(2)/j_V_[0];
			// No 2nd and 3rd stage
			t[1] = 0;
			t[2] = 0;

			v0_V_[0] = x0(1);
			// No 2nd and 3rd stage
			v0_V_[1] = 0;
			v0_V_[2] = 0;
			v0_V_[3] = vf;

			x0_V_[0] = x0(0);
			// No 2nd and 3rd stage
			x0_V_[1] = 0;
			x0_V_[2] = 0;
			x0_V_[3] = x0_V_[0] + v0_V_[0]*t[0];

			a0_V_[0] = x0(2);
			// No 2nd and 3rd stage
			a0_V_[1] = 0;
			a0_V_[2] = 0;
			a0_V_[3] = 0;
		}

		else{
			j_V_[0] = j_temp;
			j_V_[1] = 0;
			j_V_[2] = -j_temp;

			double t1 = -x0(2)/j_temp + std::sqrt(0.5*pow(x0(2),2) - j_temp*(x0(1)-vf))/j_temp;
			double t2 = -x0(2)/j_temp - std::sqrt(0.5*pow(x0(2),2) - j_temp*(x0(1)-vf))/j_temp;

			t1 = std::fmax(t1,t2);
			t1 = std::fmax(0,t1);

			// Check to see if we'll saturate
			double a1f = x0(2) + j_temp*t1;

			if (std::abs(a1f) >= a_temp){
				double am = copysign(a_temp,j_temp);
				t[0] = (am-x0(2))/j_V_[0];
				t[2] = -am/j_V_[2];

				if (x0(2)==am) j_V_[0] = 0;

				a0_V_[0] = x0(2);
				a0_V_[1] = a0_V_[0] + j_V_[0]*t[0];
				a0_V_[2] = am;
				a0_V_[3] = 0;

				v0_V_[0] = x0(1);
				v0_V_[1] = v0_V_[0] + a0_V_[0]*t[0] + 0.5*j_V_[0]*pow(t[0],2);	
				v0_V_[2] = vf - am*t[2] - 0.5*j_V_[2]*pow(t[2],2);
				v0_V_[3] = vf;

				t[1] = (v0_V_[2]-v0_V_[1])/am;	

				x0_V_[0] = x0(0);
				x0_V_[1] = x0_V_[0] + v0_V_[0]*t[0] + 0.5*a0_V_[0]*pow(t[0],2) + 1./6*j_V_[0]*pow(t[0],3);
				x0_V_[2] = x0_V_[1] + v0_V_[1]*t[1] + 0.5*am*pow(t[1],2) ;
				x0_V_[3] = x0_V_[2] + v0_V_[2]*t[2] + 0.5*am*pow(t[2],2) + 1./6*j_V_[2]*pow(t[2],3);

			}
			else{
				j_V_[0] = j_temp;
				j_V_[1] = 0; // No second phase
				j_V_[2] = -j_temp;

				t[0] = t1;
				t[1] = 0; // No second phase
				t[2] = -(x0(2)+j_V_[0]*t1)/j_V_[2];

				a0_V_[0] = x0(2);
				a0_V_[1] = 0; // No second phase
				a0_V_[2] = a0_V_[0] + j_V_[0]*t[0];
				a0_V_[3] = 0;

				v0_V_[0] = x0(1);
				v0_V_[1] = 0; // No second phase
				v0_V_[2] = v0_V_[0] + a0_V_[0]*t[0] + 0.5*j_V_[0]*pow(t[0],2);
				v0_V_[3] = vf;		

				x0_V_[0] = x0(0);
				x0_V_[1] = 0; // No second phase
				x0_V_[2] = x0_V_[0] + v0_V_[0]*t[0] + 0.5*a0_V_[0]*pow(t[0],2) + 1./6*j_V_[0]*pow(t[0],3);
				x0_V_[3] = x0_V_[2] + v0_V_[2]*t[2] + 0.5*a0_V_[2]*pow(t[2],2) + 1./6*j_V_[2]*pow(t[2],3);
			}
		}
	}

	X_switch.row(0) << x0_V_[0],x0_V_[1],x0_V_[2],x0_V_[3];
	X_switch.row(1) << v0_V_[0],v0_V_[1],v0_V_[2],v0_V_[3];
	X_switch.row(2) << a0_V_[0],a0_V_[1],a0_V_[2],a0_V_[3];
	X_switch.row(3) << j_V_[0],j_V_[1],j_V_[2],j_V_[3];

}

void TIP::eval_trajectory(Eigen::Matrix4d X_switch, Eigen::Matrix4d Y_switch, Eigen::Matrix4d Z_switch, std::vector<double> t_x, std::vector<double> t_y, std::vector<double> t_z, double t, Eigen::MatrixXd& Xc){
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

	// Eval z trajectory
	int m = 0;
	if (t < t_z[0]){
		tz_ = t;
		m = 0;
	}
	else if (t < t_z[0]+t_z[1]){
		tz_ = t - t_z[0];
		m = 1;
	}
	else if (t < t_z[0]+t_z[1]+t_z[2]){
		tz_ = t - (t_z[0]+t_z[1]);
		m = 2;
	}
	else{
		tz_ = t - (t_z[0]+t_z[1]+t_z[2]);
		m = 3;
	}

	Xc(0,0) = X_switch(0,k) + X_switch(1,k)*tx_ + 0.5*X_switch(2,k)*pow(tx_,2) + 1.0/6.0*X_switch(3,k)*pow(tx_,3);
	Xc(1,0) = X_switch(1,k) + X_switch(2,k)*tx_ + 0.5*X_switch(3,k)*pow(tx_,2);
	Xc(2,0) = X_switch(2,k) + X_switch(3,k)*tx_;
	Xc(3,0) = X_switch(3,k);

	Xc(0,1) = Y_switch(0,l) + Y_switch(1,l)*ty_ + 0.5*Y_switch(2,l)*pow(ty_,2) + 1.0/6.0*Y_switch(3,l)*pow(ty_,3);
	Xc(1,1) = Y_switch(1,l) + Y_switch(2,l)*ty_ + 0.5*Y_switch(3,l)*pow(ty_,2);
	Xc(2,1) = Y_switch(2,l) + Y_switch(3,l)*ty_;
	Xc(3,1) = Y_switch(3,l);

	Xc(0,2) = Z_switch(0,m) + Z_switch(1,m)*tz_ + 0.5*Z_switch(2,m)*pow(tz_,2) + 1.0/6.0*Z_switch(3,m)*pow(tz_,3);
	Xc(1,2) = Z_switch(1,m) + Z_switch(2,m)*tz_ + 0.5*Z_switch(3,m)*pow(tz_,2);
	Xc(2,2) = Z_switch(2,m) + Z_switch(3,m)*tz_;
	Xc(3,2) = Z_switch(3,m);

}


void TIP::convert2pcl(const sensor_msgs::PointCloud2ConstPtr msg,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out){
  	
	geometry_msgs::TransformStamped tf_body;
    sensor_msgs::PointCloud2 msg_out;
	try {
	  tf_body = tf_buffer_.lookupTransform("world", msg->header.frame_id,ros::Time(0.0));
	} catch (tf2::TransformException &ex) {
	  ROS_ERROR("%s", ex.what());
	  return;
	}

	tf2::doTransform(*msg, msg_out, tf_body);

	pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2; 
	pcl_conversions::toPCL(msg_out, *cloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*cloud2,*cloud);

	cloud_out = cloud;
}


void TIP::eigen2quadGoal(Eigen::MatrixXd Xc, acl_msgs::QuadGoal& quad_goal){
 	quad_goal_.pos.x   = Xc(0,0);
 	quad_goal_.vel.x   = Xc(1,0);
 	quad_goal_.accel.x = Xc(2,0);
 	quad_goal_.jerk.x  = Xc(3,0);

 	quad_goal_.pos.y   = Xc(0,1);
 	quad_goal_.vel.y   = Xc(1,1);
 	quad_goal_.accel.y = Xc(2,1);
 	quad_goal_.jerk.y  = Xc(3,1);

 	quad_goal_.pos.z   = Xc(0,2);
 	quad_goal_.vel.z   = Xc(1,2);
 	quad_goal_.accel.z = Xc(2,2);
 	quad_goal_.jerk.z  = Xc(3,2);
 }



void TIP::takeoff(double& z){
	z+=0.003;
	saturate(z,-0.1,goal_(2));
}


void TIP::land(double& z){
	if (z > 0.4){
		z-=0.003;
		saturate(z,-0.1,goal_(2));
	}
	else{
		z-=0.001;
		saturate(z,-0.1,goal_(2));
	}
}

void TIP::saturate(double &var, double min, double max){
	if (var < min){
		var = min;
	}
	else if (var > max){
		var = max;
	}
}

void TIP::convert2ROS(){
	// Trajectory
	traj_ros_.poses.clear();
	traj_ros_.header.stamp = ros::Time::now();
	traj_ros_.header.frame_id = "world";

	Xf_eval_ = Xf_switch_;
	Yf_eval_ = Yf_switch_;
	Zf_eval_ = Zf_switch_;
	t_xf_e_ = t_xf_;
	t_yf_e_ = t_yf_;
	t_zf_e_ = t_zf_;

	dt_ = sensor_distance_/v_max_/num_;
	t_ = 0;
	XE_ << X_;
	for(int i=0; i<num_; i++){
		mtx.lock();
		eval_trajectory(Xf_eval_,Yf_eval_,Zf_eval_,t_xf_e_,t_yf_e_,t_zf_e_,t_,XE_);
		mtx.unlock();
		temp_path_point_ros_.pose.position.x = XE_(0,0);
		temp_path_point_ros_.pose.position.y = XE_(0,1);
		temp_path_point_ros_.pose.position.z = XE_(0,2);
		t_+=dt_;
		traj_ros_.poses.push_back(temp_path_point_ros_);
	}

	ros_new_global_goal_.header.stamp = ros::Time::now();
	ros_new_global_goal_.header.frame_id = "world";
	if (can_reach_global_goal_){
		ros_new_global_goal_.point.x = goal_(0);
		ros_new_global_goal_.point.y = goal_(1);
		ros_new_global_goal_.point.z = goal_(2);

	} 
	else{
		ros_new_global_goal_.point.x = 3*local_goal_(0)+pose_(0);
		ros_new_global_goal_.point.y = 3*local_goal_(1)+pose_(1);
		ros_new_global_goal_.point.z = 3*local_goal_(2)+pose_(2);

	}
 }

void TIP::pubROS(){
	traj_pub.publish(traj_ros_);
	tipData_pub.publish(tipData_);
	new_goal_pub.publish(ros_new_global_goal_);
}
