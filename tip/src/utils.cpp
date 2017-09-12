#include "tip.hpp"

void TIP::angle_wrap(double& diff)
{
	diff =  fmod(diff+M_PI,2*M_PI);
    if (diff < 0)
        diff += 2*M_PI;
    diff -= M_PI;
}

void TIP::normalize(geometry_msgs::Quaternion &q)
{
	double root = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
	q.w = q.w/root;
	q.x = q.x/root;
	q.y = q.y/root;
	q.z = q.z/root;
}

void TIP::saturate(double &var, double min, double max)
{
	if (var < min){
		var = min;
	}
	else if (var > max){
		var = max;
	}
}

void TIP::eigen2quadGoal(Eigen::MatrixXd Xc, acl_msgs::QuadGoal& quad_goal)
{
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

void TIP::convert2pcl(const sensor_msgs::PointCloud2ConstPtr msg,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
{ 	
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
	delete cloud2;
}

void TIP::convert2rospcl(pcl::PointCloud<pcl::PointXYZ> cloud, sensor_msgs::PointCloud2 &ros_cloud)
{ 	
	pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2; 

	pcl::toPCLPointCloud2(cloud,*cloud2);
	pcl_conversions::fromPCL(*cloud2, ros_cloud);

	delete cloud2;
}

void TIP::checkpcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool& cloud_empty){
	int size = cloud->points.size();
	int i;
	int count = 0;
	cloud_empty = true;
	for(i=0;i<size;i++){
		// nan is the only number that doesn't equal itself -- John Carter
		if (cloud->points[i].x == cloud->points[i].x){
			count++;
			// Ensure there are enough good points to perform collision detection
			if (count > K_) {cloud_empty=false;break;}
		}
	}
}

void TIP::update_tree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::KdTreeFLANN<pcl::PointXYZ>> &trees){
	kdtree_.setInputCloud(cloud);

	if (c < ntree_ && virgin_){
		int eval;
		if (c==0) eval = 0;
		trees[ntree_] = kdtree_;
		clouds_[ntree_] = cloud;
		if (ros::WallTime::now().toSec()-tree_times_[eval]>time_min_)
		{
			clouds_[c] = cloud;
			trees[c] = kdtree_;
			tree_times_[c] = ros::WallTime::now().toSec();
			c++;
		}
		if (c%ntree_==0) virgin_ = false;
	}
	else {
		trees[ntree_] = kdtree_;
		clouds_[ntree_] = cloud;
		int eval = c-1;
		if (c%ntree_==0) {c = 0; eval = ntree_;}
		if (ros::WallTime::now().toSec()-tree_times_[eval]>time_min_)
		{
			clouds_[c] = cloud;
			trees[c] = kdtree_;
			tree_times_[c] = ros::WallTime::now().toSec();
			c++;		
		}
	}
}

void TIP::convert2ROS()
{
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
 }

void TIP::pubROS()
{
	traj_pub.publish(traj_ros_);
	tipData_pub.publish(tipData_);	
	bl_pub.publish(P_);
}

void TIP::pubClouds()
{	
	sensor_msgs::PointCloud2 ros_cloud;
	int iter;
	if (virgin_) iter = c;
	else iter = ntree_+1;

	pcl::PointCloud<pcl::PointXYZ> all_clouds;

	for (int i=0;i<iter;i++) all_clouds += *clouds_[i];

	convert2rospcl(all_clouds,ros_cloud);

	ros_cloud.header.frame_id = "world";
	ros_cloud.header.stamp = ros::Time::now();

	clouds_pub.publish(ros_cloud);
}