#include "tip.hpp"

TIP::TIP() : tf_listener_(tf_buffer_),trees_(ntree_) {

	std::string name = ros::this_node::getNamespace();
	// Erase slashes
	name.erase(0,2);

	// Should be read as param
	ros::param::param<bool>("~debug",debug_,true);
	ros::param::param<bool>("~use_memory",use_memory_,true);
	ros::param::param<double>("~safe_distance",safe_distance_,2.0);
	ros::param::param<double>("~buffer",buffer_,0.5);
	ros::param::param<double>("~sensor_distance",sensor_distance_,4.0);
	ros::param::param<double>("~mem_distance",mem_distance_,1.5);


	last_goal_ = Eigen::Vector3d::Zero();
	pose_= Eigen::Vector3d::Zero();
	goal_ = Eigen::Vector3d::Zero();
	X_ = Eigen::MatrixXd::Zero(4,3);
	X_stop_ = Eigen::MatrixXd::Zero(4,3);
	XE_ = Eigen::MatrixXd::Zero(4,3);

	ros::param::param<double>("~goal_x",goal_(0),0.0);
	ros::param::param<double>("~goal_y",goal_(1),0.0);
	ros::param::param<double>("~goal_z",goal_(2),0.0);

	heading_ = atan2(goal_(1),goal_(0));
	angle_2_last_goal_ = heading_;
	local_goal_angle_ = heading_;
	last_goal_ << goal_;
	local_goal_ << goal_;

	ros::param::param<double>("cntrl/spinup_time",spinup_time_,2.5);
	ros::param::param<double>("~max_speed",v_max_,2);

	v_max_org_ = v_max_;

	ros::param::param<double>("~jerk",j_max_,10);
	ros::param::param<double>("~accel",a_max_,5);
	ros::param::param<double>("~accel_stop",a_stop_,5);

	ros::param::param<double>("~plan_eval",plan_eval_time_,0.01);

	ros::param::param<int>("~K",K_,10);
	ros::param::param<int>("~N_pcl",ntree_,10);
	ros::param::param<double>("~time_min",time_min_,1.0);

	// ntree_+1 to include the most recent pcl
	trees_.resize(ntree_+1);
	clouds_.resize(ntree_+1);
	tree_times_.resize(ntree_);
	tree_times_[0] = 0;

	ros::param::param<double>("~h_fov",h_fov_,60.0);
	ros::param::param<int>("~h_samples",h_samples_,10);
	ros::param::param<double>("~v_fov",v_fov_,45.0);
	ros::param::param<int>("~v_samples",v_samples_,8);

	ros::param::param<double>("~r_max",r_max_,1.0);
	ros::param::param<double>("~jump_thresh",jump_thresh_,0.5);

	ros::param::param<double>("~z_min",z_min_,0.2);
	ros::param::param<double>("~z_max",z_max_,1.5);

	ros::param::param<double>("~goal_radius",goal_radius_,0.5);
	
	ros::param::param<double>("~W",W_,0.5);

	ros::param::param<double>("~p_min",p_min_,0.5);
	ros::param::param<double>("~p_max",p_max_,2.0);
	ros::param::param<double>("~p_dot",p_dot_,10);

	p_dot_*=plan_eval_time_;

	P_.data = p_min_;

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

	virgin_ = true;
	yawing_ = false;
	stop_ = false;
	still_clear_ = true;
	can_reach_global_goal_ = true;
	can_reach_goal_ = false;
	following_prim_ = false;
	e_stop_ = false;
	stuck_ = false;
	search_ = false;

	quad_status_.mode = quad_status_.NOT_FLYING;

	quad_goal_.cut_power = true;
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
	ROS_INFO("Planner initialized");
}

void TIP::modeCB(const acl_msgs::QuadMode& msg)
{
	// If we're in idle or waypoint mode then track position
	if (msg.mode==msg.MODE_IDLE || msg.mode == msg.MODE_WAYPOINT){
		quad_goal_.xy_mode = acl_msgs::QuadGoal::MODE_POS;
		quad_goal_.z_mode = acl_msgs::QuadGoal::MODE_POS;
	}
	// Else go open loop in xy with z position control
	else{
		quad_goal_.xy_mode = acl_msgs::QuadGoal::MODE_ACCEL;
		quad_goal_.z_mode = acl_msgs::QuadGoal::MODE_POS;
	}
}


void TIP::global_goalCB(const acl_msgs::QuadWaypoint& msg)
{
	goal_ << msg.point.x, msg.point.y, msg.point.z;
	final_heading_ = msg.heading;
}

void TIP::stateCB(const acl_msgs::State& msg)
{
	// Check if estimate jumped 
	if (sqrt(pow(pose_(0)-msg.pos.x,2) + pow(pose_(1)- msg.pos.y,2) + pow(pose_(2)- msg.pos.z,2)) > jump_thresh_){
		ROS_WARN("Jump Detected -- magnitude: %0.3f",sqrt(pow(pose_(0)-msg.pos.x,2) + pow(pose_(1)- msg.pos.y,2) + pow(pose_(2)- msg.pos.z,2)));
		if (quad_status_.mode == quad_status_.GO){
			bias_x_ = msg.pos.x-pose_(0);
			bias_y_ = msg.pos.y-pose_(1);
			bias_z_ = msg.pos.z-pose_(2);
			X_.row(0) << X_(0,0)+bias_x_,X_(0,1)+bias_y_,X_(0,2)+bias_z_;
			gen_new_traj_ = true;
		}
	}

	pose_ << msg.pos.x, msg.pos.y, msg.pos.z; 


	// Make sure quaterion is normalized
	geometry_msgs::Quaternion temp;
	temp = msg.quat;
	normalize(temp);

	qw2b_.w() = temp.w;
	qw2b_.vec() << -temp.x,-temp.y,-temp.z;

	tf::quaternionMsgToTF(temp, att_);

	// Check this
	if (quad_status_.mode == quad_status_.NOT_FLYING){
		X_.row(0) << pose_.transpose();
		yaw_ = tf::getYaw(msg.quat);
	}
}

void TIP::sendGoal(const ros::TimerEvent& e)
{	
	if (gen_new_traj_){
		gen_new_traj_ = false;
		v_plan_ = v_;
		mtx.lock();
		if (stop_) get_traj(X_,local_goal_,v_,t_xf_,t_yf_,t_zf_,Xf_switch_,Yf_switch_,Zf_switch_,stop_);
		else get_traj(X_,local_goal_,v_,t_xf_,t_yf_,t_zf_,Xf_switch_,Yf_switch_,Zf_switch_,stop_);
		mtx.unlock();
		t0_ = ros::Time::now().toSec() - plan_eval_time_;
	}

	if (quad_status_.mode == quad_status_.TAKEOFF){
		takeoff(X_);
		if (X_(0,2) == goal_(2) && fabs(goal_(2)-pose_(2))<0.2){
			quad_status_.mode = quad_status_.GO;
			ROS_INFO("Take-off Complete. GO mode engaged!");
		} 
	}

	else if (quad_status_.mode == quad_status_.LAND){
		// Make sure vx and vy are zero before landing
		if (X_.row(1).head(2).norm() == 0){
			land(X_);
			if (X_(0,2) == 0){
				quad_status_.mode = quad_status_.NOT_FLYING;
				quad_goal_.cut_power = true;
				ROS_INFO("Landing Complete");
			}
		}
		else {
			tE_ = ros::Time::now().toSec() - t0_;
		
			mtx.lock();		
			eval_trajectory(Xf_switch_,Yf_switch_,Zf_switch_,t_xf_,t_yf_,t_zf_,tE_,X_);
			mtx.unlock();
		}
	}

	else if (quad_status_.mode == quad_status_.GO){		
		dist_2_goal_ = (goal_.head(2) - X_.row(0).transpose().head(2)).norm();
	
		heading_ = atan2(goal_(1)-X_(0,1),goal_(0)-X_(0,0));
		// if (X_.row(1).norm()==0) heading_ = atan2(goal_(1)-X_(0,1),goal_(0)-X_(0,0));
		// else heading_ = atan2(local_goal_(1),local_goal_(0));

		double diff = heading_ - quad_goal_.yaw; 
		angle_wrap(diff);

		// if (X_.row(2).norm()>2){diff=0;}

		// if (search_){
		// 	quad_goal_.yaw += 0.01;
		// 	diff = 0;
		// }		

		if(fabs(diff)>0.02 && !stop_ && dist_2_goal_ > goal_radius_){
			if (!yawing_ && (fabs(diff)>M_PI/2 || X_.row(1).norm()==0.0)) {v_ = 0; gen_new_traj_=true;}			
			// Only yaw if the vehicle is at right speed
			if (X_.row(1).norm() <= (v_+0.1*v_max_) && X_.row(1).norm() >= (v_-0.1*v_max_)){
				yawing_ = true;
				yaw(diff,quad_goal_);
			}	
		}
		else {
			// Could be spot for picking new speed
			if (!stop_ && (can_reach_goal_ || following_prim_) && dist_2_goal_ > goal_radius_) {v_ = v_max_; stuck_=false; search_=false;}
			else if (!stop_ && !can_reach_goal_ && dist_2_goal_>goal_radius_) {
				ROS_INFO_THROTTLE(1.0,"stuck");
				if (!stuck_) {stuck_=true; t_stuck = ros::Time::now().toSec();}
				if (ros::Time::now().toSec()-t_stuck>2 && !search_){
					search_ = true;
					if (c_search==-1) c_search = 1;
					else c_search =-1;
					ROS_INFO("Search");
					// v_max_ = 0.1; 
					// safe_distance_= 0.4;
				}
			}
			quad_goal_.dyaw = 0;
			yawing_ = false;
		} 	

		if (!stop_ && X_.row(1).norm() > 0){
			get_stop_dist(X_,goal_,can_reach_global_goal_,stop_);
			if (stop_){
				v_ = 0;
				gen_new_traj_ = true;
		 		ROS_INFO_THROTTLE(1.0,"stopping");
			}
		}

		// Desired heading at waypoint
		diff = final_heading_ - quad_goal_.yaw;
		angle_wrap(diff);
		if (!stop_ && dist_2_goal_ < goal_radius_ && X_.row(1).norm()==0){
			if(fabs(diff)>0.01){
				yawing_ = true;
				yaw(diff,quad_goal_);
			}
			else{
				yawing_ = false;
				quad_goal_.dyaw = 0;
			}
		}

		tE_ = ros::Time::now().toSec() - t0_;
		
		mtx.lock();		
		eval_trajectory(Xf_switch_,Yf_switch_,Zf_switch_,t_xf_,t_yf_,t_zf_,tE_,X_);
		mtx.unlock();

		if (X_.row(1).norm() == 0 && stop_){
			// We're done	
			stop_ = false;
			yawing_ = false;
			v_=0;
			if (e_stop_){quad_status_.mode = quad_status_.FLYING;}
			else if (can_reach_goal_ && dist_2_goal_ > goal_radius_) {v_ = v_max_; gen_new_traj_ = true;}
		}
		yaw_ = quad_goal_.yaw;
	}

	eigen2quadGoal(X_,quad_goal_);
	quad_goal_.header.stamp = ros::Time::now();
	quad_goal_.header.frame_id = "world";
	quad_goal_pub.publish(quad_goal_);
}

void TIP::eventCB(const acl_msgs::QuadFlightEvent& msg)
{
	// Takeoff
	if (msg.mode == msg.TAKEOFF && quad_status_.mode == quad_status_.NOT_FLYING){
		ROS_INFO("Waiting for spinup");
		quad_goal_.pos.x = pose_(0);
		quad_goal_.pos.y = pose_(1);
		quad_goal_.pos.z = pose_(2);

		

		quad_goal_.vel.x = 0;
		quad_goal_.vel.y = 0;
		quad_goal_.vel.z = 0;

		quad_goal_.yaw = yaw_;
		quad_goal_.dyaw = 0;

		quad_goal_.cut_power = false;

		ros::Duration(spinup_time_).sleep();
		ROS_INFO("Taking off");

		quad_status_.mode = quad_status_.TAKEOFF; 
		X_(0,0) = pose_(0);
		X_(0,1) = pose_(1);
		X_(0,2) = pose_(2)+0.5;
	}
	// Emergency kill
	else if (msg.mode == msg.KILL && quad_status_.mode != quad_status_.NOT_FLYING){
		quad_status_.mode = quad_status_.NOT_FLYING;
		quad_goal_.cut_power = true;
		ROS_ERROR("Killing");
	}
	// Landing
	else if (msg.mode == msg.LAND && quad_status_.mode != quad_status_.NOT_FLYING){
		quad_status_.mode = quad_status_.LAND;
		if (v_ != 0){
			v_ = 0;
			stop_ = true;
			quad_goal_.dyaw = 0;
			// Generate new traj
			gen_new_traj_ = true;
		}
		ROS_INFO_THROTTLE(1.0,"Landing");
	}
	// GO!!!!
	else if (msg.mode == msg.START && quad_status_.mode == quad_status_.FLYING){
		e_stop_ = false;
		quad_status_.mode = quad_status_.GO;
		ROS_INFO("Starting");		
	}
	// STOP!!!
	else if (msg.mode == msg.ESTOP && quad_status_.mode == quad_status_.GO){
		ROS_INFO_THROTTLE(1.0,"Stopping");
		// Stay in go command but set speed to zero
		v_ = 0;
		stop_ = true;
		quad_goal_.dyaw = 0;
		// Generate new traj
		gen_new_traj_ = true;
		e_stop_ = true;		
	}
}

void TIP::pclCB(const sensor_msgs::PointCloud2ConstPtr& msg)
{
 	msg_received_ = ros::WallTime::now().toSec();
 	
 	// Convert pcl
	convert2pcl(msg,cloud_);

	checkpcl(cloud_,cloud_empty_);

 	if (!cloud_empty_ && quad_status_.mode!=quad_status_.NOT_FLYING){
 		
 		if (c==0) update_tree(cloud_,trees_);

		// Build k-d tree
		if (!stop_ && (X_.row(1).norm()>0 || yawing_)){
			update_tree(cloud_,trees_);
		}

		// Sort allowable final states
		sort_ss(Goals_,X_.row(0).transpose(),goal_, last_goal_, Sorted_Goals_,v_los_);

		// Pick desired final state
		pick_ss(Sorted_Goals_, X_, can_reach_goal_);
		
		if (!v_los_ && !stop_) following_prim_ = true;

		// Check if current primitive is still collision free
		if (following_prim_ && v_plan_ > 0) {
			check_current_prim(Xf_switch_,Yf_switch_,Zf_switch_,t_xf_,t_yf_,t_zf_,tE_,X_,still_clear_);
		}

	 	if (still_clear_ && v_plan_ > 0 && use_memory_ && !stop_ && dist_trav_last_ < mem_distance_ && min_cost_prim_ > last_prim_cost_ && quad_status_.mode == quad_status_.GO){
			// Update distance traveled
			dist_trav_last_ = (X_.row(0).transpose() - pose_last_mp_).norm();
			following_prim_ = true;
		}

		else{
			following_prim_ = false;
			still_clear_ = true;
			if (!can_reach_goal_ && !stop_ && quad_status_.mode==quad_status_.GO && X_.row(1).norm()>0){
		 		// Need to stop!!!
		 		v_ = 0;
		 		// stop_ = true;
		 		ROS_ERROR_THROTTLE(1.0,"Emergency stop -- no feasible path");
		 	}
			
		 	gen_new_traj_ = true;
		 	pose_last_mp_ = X_.row(0).transpose();
		 	dist_safe_last_ = distance_traveled_;
		 	dist_trav_last_ = 0;
			last_prim_cost_ = min_cost_prim_ ;
	 	}

	 	static int counter = 0;

	 	double p_temp = P_.data;
	 	if (!following_prim_ && min_cost_prim_==0){
	 		// ROS_INFO("Expand BL");
	 		counter++;
	 		if (counter > 30){
		 		p_temp += p_dot_;
		 		saturate(p_temp,p_min_,p_max_);
	 		}
	 	}
	 	else{
	 		if (min_cost_prim_!=0)	counter = 0;
	 		p_temp -= p_dot_;
	 		saturate(p_temp,p_min_,p_max_);
	 	}
	 	
	 	P_.data = p_temp;
	 	P_.header.stamp = ros::Time::now();

	 	buffer_ = 0.3+P_.data/3.0;

	 	// std::cout << "Latency (ms): " << 1000*(traj_gen_ - msg_received_) << std::endl;		
	 }
	else {
		// Generate traj
		local_goal_ = goal_ - X_.row(0).transpose();
		gen_new_traj_ = true;

		pose_last_mp_ = X_.row(0).transpose();
		dist_safe_last_ = distance_traveled_;
		dist_trav_last_ = 0;
		last_prim_cost_ = min_cost_prim_ ;
	}

 	tipData_.header.stamp = ros::Time::now();
 	tipData_.latency = 1000*(ros::WallTime::now().toSec() - msg_received_);
 	tipData_.speed = X_.row(1).norm();
 	tipData_.follow_prim = following_prim_;

 	saturate(min_cost_prim_,0,5);

 	if (quad_status_.mode!=quad_status_.GO || stop_) tipData_.prim_cost = 0;
 	else if (following_prim_) tipData_.prim_cost = last_prim_cost_;
    else tipData_.prim_cost = min_cost_prim_;

 	if(debug_){
 		convert2ROS();
 		pubROS();
 		pubClouds();
 	} 	
 } 	

void TIP::sample_ss(Eigen::MatrixXd& Goals)
{
	theta_ = Eigen::VectorXd::Zero(h_samples_);
	theta_.setLinSpaced(h_samples_,-h_fov_/2,h_fov_/2);
	
	if (v_samples_==0) v_samples_=1;
	phi_ = Eigen::VectorXd::Zero(v_samples_);
	phi_.setLinSpaced(v_samples_,-v_fov_/2,v_fov_/2);

	Goals = Eigen::MatrixXd::Zero((h_samples_)*(v_samples_),3);
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
			k++;
		}
	}
}

void TIP::sort_ss(Eigen::MatrixXd Goals, Eigen::Vector3d pose, Eigen::Vector3d goal, Eigen::Vector3d vector_last, Eigen::MatrixXd& Sorted_Goals, bool& v_los)
{
 	// Re-initialize
	cost_queue_ = std::priority_queue<double, std::vector<double>, std::greater<double> > ();
	cost_v_.clear();

	// TODO: add check that the norm > 0
	vector_2_goal_= goal-pose ;
	vector_2_goal_.normalize();

	vector_2_goal_body_ = qw2b_._transformVector(vector_2_goal_);

	vector_last_ = vector_last-pose;
	vector_last_.normalize();

	num_of_pnts_ = Goals.rows();

 	for (int i=0; i < num_of_pnts_ ; i++){
		vector_i_ << Goals.row(i).transpose();

		// Transform to world
		vector_i_ = qw2b_.conjugate()._transformVector(vector_i_);

		Eigen::Vector2d vector_i_xy = vector_i_.head(2);
		Eigen::Vector2d vector_i_xz(vector_i_(0), vector_i_(2));

		Eigen::Vector2d vector_goal_xz(vector_2_goal_(0), vector_2_goal_(2));
		vector_goal_xz.normalize();

		Eigen::Vector2d vector_goal_xy(vector_2_goal_(0), vector_2_goal_(1));
		vector_goal_xy.normalize();

		vector_i_xy.normalize();
		vector_i_xz.normalize();

		double d_xy = vector_i_xy.dot(vector_goal_xy);
		double d_xz = vector_i_xz.dot(vector_goal_xz);

		saturate(d_xz,-1,1);
		saturate(d_xy,-1,1);

		double a_d_xy = acos(d_xy);
		double a_d_xz = acos(d_xz);

		// Make sure dot product is within [-1 1] bounds for acos
		double dot = vector_i_.dot(vector_last_);
		saturate(dot,-1,1);
		angle_diff_last_ = acos(dot);

 		cost_i_ = pow(a_d_xy,2) + pow(W_*a_d_xz,2) + 0.05*pow(angle_diff_last_,2);

 		cost_queue_.push(cost_i_);
 		cost_v_.push_back(cost_i_);
 	}

 	Eigen::MatrixXd Sorted_Goals_temp;
 	Sorted_Goals_temp = Eigen::MatrixXd::Zero(Goals.rows(),Goals.cols()+1);

 	for (int i=0; i < num_of_pnts_ ; i++){
	 	min_cost_ = cost_queue_.top();

		it_ = std::find(cost_v_.begin(),cost_v_.end(),min_cost_);
		goal_index_ = it_ - cost_v_.begin();

		if (goal_index_ == Goals.rows()) goal_index_=Goals.rows()-1;

		Sorted_Goals_temp.row(i) << Goals.row(goal_index_), min_cost_;

		cost_queue_.pop();
	}

	 // Check if vector_2_goal_body_ is within FOV
 	double r, p, y;
 	tf::Matrix3x3(att_).getRPY(r, p, y);

 	double angle_v = p;

	double angle_h = heading_ - quad_goal_.yaw;
	angle_wrap(angle_h);

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

void TIP::pick_ss(Eigen::MatrixXd Sorted_Goals, Eigen::MatrixXd X, bool& can_reach_goal)
{
	goal_index_ = 0;
	bool temp_reach_goal = false;
	// can_reach_goal = false;

 	while(!temp_reach_goal && goal_index_ < Sorted_Goals.rows()){
 		// Tranform temp local goal to world frame
 		Eigen::Vector4d temp_local_goal_aug;
 		temp_local_goal_aug << qw2b_.conjugate()._transformVector(Sorted_Goals.block(goal_index_,0,1,3).transpose()), Sorted_Goals(goal_index_,3);

		collision_check(X,buffer_,v_max_,temp_reach_goal,temp_local_goal_aug); 	

		// Update cost
		Sorted_Goals(goal_index_,3) = temp_local_goal_aug(3);

 		goal_index_++;
 	}

 	int index;
	min_cost_prim_ = Sorted_Goals.col(3).minCoeff(&index);
	double min_cost = min_cost_prim_;

 	if (!temp_reach_goal) {	
 		if (min_cost!=inf){
 			goal_index_ = index+1;
 			temp_reach_goal = true;
 		} 
 	}

 	can_reach_goal = temp_reach_goal;

 	if(can_reach_goal){
 		goal_index_--;
 		// Tranform temp local goal to world frame
 		local_goal_ = qw2b_.conjugate()._transformVector(Sorted_Goals.block(goal_index_,0,1,3).transpose());
 		last_goal_ << local_goal_;
 		Eigen::Vector3d x = X.row(0).transpose();
 		double d = (goal_.head(2)-x.head(2)).norm();
 		if (goal_index_ == 0 && d < sensor_distance_) can_reach_global_goal_ = true;
 		else can_reach_global_goal_ = false;
 	}
}


void TIP::get_traj(Eigen::MatrixXd X, Eigen::Vector3d local_goal, double v, std::vector<double>& t_fx, std::vector<double>& t_fy, std::vector<double>& t_fz, Eigen::Matrix4d& Xf_switch, Eigen::Matrix4d& Yf_switch, Eigen::Matrix4d& Zf_switch, bool stop_check )
{
	//Generate new traj
	get_vels(X,local_goal,v,vfx_,vfy_,vfz_);

	x0_ << X.col(0);
	y0_ << X.col(1);
	z0_ << X.col(2);
	find_times(x0_, vfx_, t_fx, Xf_switch,stop_check);
	find_times(y0_, vfy_, t_fy, Yf_switch,stop_check);
	find_times(z0_, vfz_, t_fz, Zf_switch,stop_check);

	// Sync trajectory experimentation
	// if (X.row(1).norm()==0){
	// 	// Sync traj
	// 	double tx = std::accumulate(t_fx.begin(), t_fx.end(), 0.0);
	// 	double ty = std::accumulate(t_fy.begin(), t_fy.end(), 0.0);
	// 	double tz = std::accumulate(t_fz.begin(), t_fz.end(), 0.0);
	// 	double tmax = std::max(std::max(tx,ty),tz);

	// 	sync_times(x0_, tmax, vfx_, t_fx, Xf_switch);
	// 	sync_times(y0_, tmax, vfy_, t_fy, Yf_switch);
	// 	sync_times(z0_, tmax, vfz_, t_fz, Zf_switch);
	// }

}

void TIP::sync_times(Eigen::Vector4d x0, double tmax, double vf, std::vector<double>& t, Eigen::Matrix4d& X_switch)
{
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


void TIP::get_stop_dist(Eigen::MatrixXd X, Eigen::Vector3d goal, bool can_reach_global_goal, bool& stop)
{
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
		Eigen::Vector3d x_stop = X_stop_.row(0).transpose();
		Eigen::Vector3d x = X.row(0).transpose();
		d_stop_ = (x_stop.head(2) - x.head(2)).norm();
		d_goal_ = (x.head(2) - goal.head(2)).norm();

		// Prevents oscillation if our stopping distance is really small (low speed)
		saturate(d_stop_,0.1,d_stop_);

		if (d_stop_ >= d_goal_){
			// Need to stop
			stop = true;
		}
	}
}


void TIP::get_vels(Eigen::MatrixXd X, Eigen::Vector3d local_goal, double v, double& vx, double& vy, double& vz)
{
	Eigen::Vector3d temp = local_goal;
	temp.normalize();
	vx = v*temp(0);
	vy = v*temp(1);
	vz = v*temp(2);
}



void TIP::find_times(Eigen::Vector4d x0, double vf, std::vector<double>& t, Eigen::Matrix4d&  X_switch, bool stop_check)
{
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

void TIP::eval_trajectory(Eigen::Matrix4d X_switch, Eigen::Matrix4d Y_switch, Eigen::Matrix4d Z_switch, std::vector<double> t_x, std::vector<double> t_y, std::vector<double> t_z, double t, Eigen::MatrixXd& Xc)
{
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

void TIP::takeoff(Eigen::MatrixXd& X)
{
	X(0,2)+=0.005;
	X(1,2) = 0.0;
	saturate(X(0,2),-0.1,goal_(2));
	if (X(0,2)==goal_(2)) X_(1,2) = 0;
	ros::Duration(0.01).sleep();
}


void TIP::land(Eigen::MatrixXd& X)
{
	X(0,2)-=0.005;
	X(1,2) = -0.0;
	saturate(X(0,2),0,goal_(2));
	if (X(0,2)==0) X_(1,2) = 0;
	ros::Duration(0.01).sleep();	
}

void TIP::yaw(double diff, acl_msgs::QuadGoal &quad_goal){
	saturate(diff,-plan_eval_time_*r_max_,plan_eval_time_*r_max_);
	if (diff>0) quad_goal.dyaw =  r_max_;
	else        quad_goal.dyaw = -r_max_;
	quad_goal.yaw+=diff;
}

