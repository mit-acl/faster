#ifndef RP_HPP_
#define RP_HPP_

#define SCREEN_PRINT_RATE	0.5
#define PI 3.14159

// ROS includes
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>
#include "nav_msgs/Path.h"

#include <Eigen/Dense>

// custom messages
#include "acl_system/ViconState.h"
#include "acl_system/QuadGoal.h"
#include "acl_system/QuadState.h"
#include "acl_system/QuadFlightEvent.h"
#include "acl_system/FloatStamped.h"

#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <nanoflann.hpp>
#include <pcl/point_cloud.h>
#include "kd_tree.h"

// Global includes
#include <stdio.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <queue>
#include <mutex>

class REACT
{
public:
	REACT();

	double plan_eval_time_ ;

	ros::Publisher traj_pub, goal_pub, new_goal_pub, int_goal_pub, last_goal_pub, quad_goal_pub, latency_pub, pub_clean_scan;


	void pclCB(const sensor_msgs::PointCloud2ConstPtr& msg);
	void stateCB(const acl_system::ViconState& msg);
	void global_goalCB(const geometry_msgs::PointStamped& msg);
	void eventCB(const acl_system::QuadFlightEvent& msg);

	// ROS timed functions
	void sendGoal(const ros::TimerEvent&);

	void convert2ROS();
	void pubROS();

	// Make these private after testing
	void get_stop_dist(Eigen::MatrixXd X, Eigen::Vector3d goal,Eigen::Vector3d pose, bool can_reach_global_goal, bool& stop);
	void get_traj(Eigen::MatrixXd X, double angle_2_local_goal, double v, std::vector<double>& t_fx, std::vector<double>& t_fy, Eigen::Matrix4d& Xf_switch, Eigen::Matrix4d& Yf_switch, bool stop_check  );
	
	void sample_ss(Eigen::MatrixXd& Goals);
	void sort_ss(Eigen::MatrixXd Goals, Eigen::Vector3d pose, Eigen::Vector3d goal, double angle_2_last_goal, Eigen::MatrixXd& Sorted_Goals);
	void pick_ss(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::MatrixXd Sorted_Goals, Eigen::MatrixXd X, bool& can_reach_goal);

	void collision_check(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::MatrixXd X, double current_angle_2_local_goal, double buff, double v, double& tf, bool& can_reach_goal);
	
	void convert2pcl(const sensor_msgs::PointCloud2ConstPtr msg,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);
	void find_times( Eigen::Vector4d x0, double vf, std::vector<double>& t, Eigen::Matrix4d&  X_switch , bool stop_check );
	void eval_trajectory(Eigen::Matrix4d X0, Eigen::Matrix4d Y0, std::vector<double> t_x, std::vector<double> t_y, double t, Eigen::MatrixXd& X);
	void get_vels(Eigen::MatrixXd X, double angle_2_local_goal, double v, double& vx, double& vy);
	void saturate(double &var, double min, double max);
	void eigen2quadGoal(Eigen::MatrixXd X, acl_system::QuadGoal& quad_goal);

private:

	tf::TransformListener tf_listener_;

	double yaw_, dist_2_goal_, angle_2_goal_, msg_received_, cost_, cost_i_, angle_diff_, angle_diff_last_, safe_distance_, min_cost_, buffer_;
	double vfx_, vfy_, t_, tE_, dt_, tf_, r_, d_min_, tx_, ty_, v_, v_max_;
	double traj_gen_, t_stop_, d_stop_, d_goal_;
	double h_fov_, v_fov_, angle_2_last_goal_, current_angle_2_local_goal_, mean_distance_, goal_distance_, distance_traveled_, local_goal_angle_ ;
	double tE_prev_;
	double angle_i_, r_goal_, spinup_time_, heading_, j_max_, a_max_, t0_;
	int num_ = 100, K_, goal_index_, num_of_v_pnts_, num_of_h_pnts_, quad_status_ ;
	bool debug_, can_reach_goal_, collision_detected_, gen_new_traj_, stop_, can_reach_global_goal_;
	

	std::ostringstream errorMsg, warnMsg;

	std::mutex mtx;

	// KDTree<double> kd_tree_;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
	// // // // //
	// Ros var initialization
	geometry_msgs::PoseArray goal_points_ros_ ;
	geometry_msgs::Pose temp_goal_point_ros_;
	geometry_msgs::PoseStamped temp_path_point_ros_;
	geometry_msgs::PointStamped ros_new_global_goal_, ros_last_global_goal_;
	nav_msgs::Path traj_ros_;
	acl_system::FloatStamped latency_;

	acl_system::QuadGoal quad_goal_;
	acl_system::QuadState state_;
	acl_system::QuadFlightEvent quad_event_;


	// Weird initialization
	std::vector<double> t_x_{std::vector<double>(3,0)};
	std::vector<double> t_y_{std::vector<double>(3,0)}; 

	std::vector<double> t_xf_{std::vector<double>(3,0)};
	std::vector<double> t_yf_{std::vector<double>(3,0)};

	std::vector<double> t2_xf_{std::vector<double>(3,0)};
	std::vector<double> t2_yf_{std::vector<double>(3,0)};


	std::vector<double> t_x_stop_{std::vector<double>(3,0)};
	std::vector<double> t_y_stop_{std::vector<double>(3,0)};

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

	Eigen::Matrix4d Xf_switch_; // [x0; v0; a0; j0]
	Eigen::Matrix4d Yf_switch_;

	Eigen::Matrix4d X2f_switch_; // [x0; v0; a0; j0]
	Eigen::Matrix4d Y2f_switch_;

	Eigen::Matrix4d X_switch_stop_ ;
	Eigen::Matrix4d Y_switch_stop_ ;

	Eigen::MatrixXd X_; 
	Eigen::MatrixXd XE_;
	Eigen::MatrixXd Goals_;
	Eigen::MatrixXd Sorted_Goals_;
	Eigen::MatrixXd X_prop_;
	Eigen::MatrixXd X_stop_;


	Eigen::Vector4d x0_;
	Eigen::Vector4d y0_;
	
	Eigen::Vector3d goal_;
	Eigen::Vector3d local_goal_;
	Eigen::Vector3d last_goal_;
	Eigen::Vector3d last_goal_V_;
	Eigen::Vector3d next_goal_V_;
	Eigen::Vector3d pose_;

	Eigen::VectorXd theta_, phi_;

	pcl::PointXYZ searchPoint_;


	//## Logging and Debugging Functions

	void takeoff(double& z);
	void land(double& z);
	
};

#endif /* RP_HPP_ */