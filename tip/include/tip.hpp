#ifndef TIP_HPP_
#define TIP_HPP_

// ROS includes
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "nav_msgs/Path.h"

#include <Eigen/Dense>

// custom messages
#include "acl_msgs/State.h"
#include "acl_msgs/QuadGoal.h"
#include "acl_msgs/QuadState.h"
#include "acl_msgs/QuadFlightEvent.h"
#include "acl_msgs/QuadMode.h"
#include "acl_msgs/FloatStamped.h"
#include "acl_msgs/QuadWaypoint.h"
#include "tip/TIP.h"

#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

// Global includes
#include <stdio.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <queue>
#include <mutex>

class TIP
{
public:
	TIP();

	double plan_eval_time_ ;
	int ntree_=1;

	ros::Publisher traj_pub, goal_pub, new_goal_pub, quad_goal_pub, tipData_pub, clouds_pub, bl_pub;


	void pclCB(const sensor_msgs::PointCloud2ConstPtr& msg);
	void stateCB(const acl_msgs::State& msg);
	void global_goalCB(const acl_msgs::QuadWaypoint& msg);
	void eventCB(const acl_msgs::QuadFlightEvent& msg);
	void modeCB(const acl_msgs::QuadMode& msg);

	// ROS timed functions
	void sendGoal(const ros::TimerEvent&);

	void convert2ROS();
	void pubROS();

	// Make these private after testing
	void get_stop_dist(Eigen::MatrixXd X, Eigen::Vector3d goal,bool can_reach_global_goal, bool& stop);
	void get_traj(Eigen::MatrixXd X, Eigen::Vector3d local_goal, double v, std::vector<double>& t_fx, std::vector<double>& t_fy, std::vector<double>& t_fz, Eigen::Matrix4d& Xf_switch, Eigen::Matrix4d& Yf_switch, Eigen::Matrix4d& Zf_switch, bool stop_check );	
	
	void sample_ss(Eigen::MatrixXd& Goals);
	void sort_ss(Eigen::MatrixXd Goals, Eigen::Vector3d pose, Eigen::Vector3d goal, Eigen::Vector3d vector_last, Eigen::MatrixXd& Sorted_Goals, bool& v_los);
	void pick_ss(Eigen::MatrixXd Sorted_Goals, Eigen::MatrixXd X, bool& can_reach_goal);

	void collision_check(Eigen::MatrixXd X, double buff, double v, bool& can_reach_goal, Eigen::Vector4d& local_goal_aug);
	
	void convert2pcl(const sensor_msgs::PointCloud2ConstPtr msg,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);
	void find_times( Eigen::Vector4d x0, double vf, std::vector<double>& t, Eigen::Matrix4d&  X_switch , bool stop_check );
	void eval_trajectory(Eigen::Matrix4d X0, Eigen::Matrix4d Y0, Eigen::Matrix4d Z0, std::vector<double> t_x, std::vector<double> t_y, std::vector<double> t_z, double t, Eigen::MatrixXd& X);
	void get_vels(Eigen::MatrixXd X, Eigen::Vector3d local_goal, double v, double& vx, double& vy, double& vz);
	void saturate(double &var, double min, double max);
	void eigen2quadGoal(Eigen::MatrixXd X, acl_msgs::QuadGoal& quad_goal);
	void check_current_prim(Eigen::Matrix4d X0, Eigen::Matrix4d Y0, Eigen::Matrix4d Z0, std::vector<double> t_x, std::vector<double> t_y, std::vector<double> t_z, double t, Eigen::MatrixXd X, bool& clear);
	void sync_times(Eigen::Vector4d x0, double tmax, double vf, std::vector<double>& tf, Eigen::Matrix4d& X_switch);
	void angle_wrap(double& diff);
	void normalize(geometry_msgs::Quaternion &q);
	void yaw(double diff, acl_msgs::QuadGoal &quad_goal);
	void checkpcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool& cloud_empty);
	void update_tree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::KdTreeFLANN<pcl::PointXYZ>> &trees);
	void convert2rospcl(pcl::PointCloud<pcl::PointXYZ> cloud, sensor_msgs::PointCloud2 &ros_cloud);

	void pubClouds();

private:

	tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

	double yaw_, dist_2_goal_, angle_2_goal_, msg_received_, cost_, cost_i_, angle_diff_, angle_diff_last_, safe_distance_, sensor_distance_, min_cost_, buffer_;
	double vfx_, vfy_, vfz_, t_, tE_, dt_, tf_, r_, d_min_, tx_, ty_, tz_, v_, v_max_, v_max_org_;
	double traj_gen_, t_stop_, d_stop_, d_goal_;
	double h_fov_, v_fov_, angle_2_last_goal_, current_angle_2_local_goal_, mean_distance_, goal_distance_, distance_traveled_, local_goal_angle_ ;
	double tE_prev_, angle_i_, r_goal_, spinup_time_, heading_, j_max_, a_max_, a_stop_, t0_, r_max_;
	double dist_trav_last_, dist_safe_last_, last_prim_cost_, min_cost_prim_;
	double jump_thresh_, bias_x_, bias_y_, bias_z_;
	double inf, z_min_, z_max_, v_plan_, mem_distance_, goal_radius_, final_heading_, W_;
	double p_min_, p_max_, p_dot_;

	int num_ = 50, K_, goal_index_, num_of_pnts_, h_samples_, v_samples_, count2 ;
	bool debug_, can_reach_goal_, collision_detected_, gen_new_traj_, stop_, can_reach_global_goal_, yawing_, following_prim_, v_los_, use_memory_, still_clear_, e_stop_, cloud_empty_;
	

	std::ostringstream errorMsg, warnMsg;

	std::mutex mtx;

	// KDTree<double> kd_tree_;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

	std::vector<pcl::KdTreeFLANN<pcl::PointXYZ>> trees_;
	std::vector<double> tree_times_;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;
	int c = 0, c_search=-1;
	double time_min_ ;
	bool virgin_, first = true, stuck_, search_;
	double t0, t_stuck;
	// // // // //
	// Ros var initialization
	// geometry_msgs::PoseArray goal_points_ros_ ;
	sensor_msgs::PointCloud goal_points_ros_;
	geometry_msgs::Pose temp_goal_point_ros_;
	geometry_msgs::PoseStamped temp_path_point_ros_;
	nav_msgs::Path traj_ros_;
	tip::TIP tipData_;

	acl_msgs::QuadGoal quad_goal_;
	acl_msgs::QuadState quad_status_;
	acl_msgs::QuadFlightEvent quad_event_;
	acl_msgs::FloatStamped P_;


	// Weird initialization
	std::vector<double> t_x_{std::vector<double>(3,0)};
	std::vector<double> t_y_{std::vector<double>(3,0)}; 
	std::vector<double> t_z_{std::vector<double>(3,0)}; 

	std::vector<double> t_xf_{std::vector<double>(3,0)};
	std::vector<double> t_yf_{std::vector<double>(3,0)};
	std::vector<double> t_zf_{std::vector<double>(3,0)};

	std::vector<double> t_xf_e_{std::vector<double>(3,0)};
	std::vector<double> t_yf_e_{std::vector<double>(3,0)};
	std::vector<double> t_zf_e_{std::vector<double>(3,0)};

	std::vector<double> t2_xf_{std::vector<double>(3,0)};
	std::vector<double> t2_yf_{std::vector<double>(3,0)};
	std::vector<double> t2_zf_{std::vector<double>(3,0)};


	std::vector<double> t_x_stop_{std::vector<double>(3,0)};
	std::vector<double> t_y_stop_{std::vector<double>(3,0)};
	std::vector<double> t_z_stop_{std::vector<double>(3,0)};

	std::vector<double> x0_V_{std::vector<double>(4,0)};
	std::vector<double> v0_V_{std::vector<double>(4,0)};
	std::vector<double> a0_V_{std::vector<double>(4,0)};
	// Note the last entry is always zero
	std::vector<double> j_V_{std::vector<double>(4,0)};

	std::vector<double> cost_v_;
	std::vector<double>::iterator it_;

	std::priority_queue<double, std::vector<double>, std::greater<double> > cost_queue_;


	Eigen::Matrix4d X_switch_; // [x0; v0; a0; j0]
	Eigen::Matrix4d Y_switch_;
	Eigen::Matrix4d Z_switch_;

	Eigen::Matrix4d Xf_switch_; // [x0; v0; a0; j0]
	Eigen::Matrix4d Yf_switch_;
	Eigen::Matrix4d Zf_switch_;

	Eigen::Matrix4d Xf_eval_; // [x0; v0; a0; j0]
	Eigen::Matrix4d Yf_eval_;
	Eigen::Matrix4d Zf_eval_;

	Eigen::Matrix4d X2f_switch_; // [x0; v0; a0; j0]
	Eigen::Matrix4d Y2f_switch_;
	Eigen::Matrix4d Z2f_switch_;

	Eigen::Matrix4d X_switch_stop_ ;
	Eigen::Matrix4d Y_switch_stop_ ;
	Eigen::Matrix4d Z_switch_stop_ ;

	Eigen::MatrixXd X_; 
	Eigen::MatrixXd XE_;
	Eigen::MatrixXd Goals_;
	Eigen::MatrixXd Sorted_Goals_;
	Eigen::MatrixXd X_prop_;
	Eigen::MatrixXd X_stop_;

	Eigen::Vector4d x0_;
	Eigen::Vector4d y0_;
	Eigen::Vector4d z0_;
	
	Eigen::Vector3d goal_;
	Eigen::Vector3d local_goal_;
	Eigen::Vector3d last_goal_;
	Eigen::Vector3d last_goal_V_;
	Eigen::Vector3d next_goal_V_;
	Eigen::Vector3d pose_;
	Eigen::Vector3d pose_last_mp_;

	Eigen::Vector3d vector_2_goal_;
	Eigen::Vector3d vector_2_goal_body_;
	Eigen::Vector3d vector_last_;
	Eigen::Vector3d vector_i_;

	Eigen::Vector3d temp_local_goal_;

	Eigen::Quaterniond qw2b_;

	Eigen::VectorXd theta_, phi_;

	pcl::PointXYZ searchPoint_;

	tf::Quaternion att_;

	//## Logging and Debugging Functions

	void takeoff(Eigen::MatrixXd& X);
	void land(Eigen::MatrixXd& X);
	
};

#endif /* RP_HPP_ */
