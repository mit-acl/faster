/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
//#include "geometry_msgs/Twist.h"
#include "visualization_msgs/MarkerArray.h"

#include "ros/ros.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

//#include <atomic>

#include <Eigen/Dense>
#include <snapstack_msgs/State.h>
#include <snapstack_msgs/QuadGoal.h>
#include <faster_msgs/Mode.h>

// TimeSynchronizer includes
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "utils.hpp"

#include "faster.hpp"
#include "faster_types.hpp"

#define WHOLE 1  // Whole trajectory (part of which is planned on unkonwn space)
#define SAFE 2   // Safe path
#define COMMITTED_COLORED 3
#define WHOLE_COLORED 4
#define SAFE_COLORED 5

#define JPSk_NORMAL 1
#define JPS2_NORMAL 2
#define JPS_WHOLE 3
#define JPS_SAFE 4

//####Class FasterRos
class FasterRos
{
public:
  FasterRos(ros::NodeHandle nh);

private:
  std::unique_ptr<Faster> faster_ptr_;

  // class methods
  void pubTraj(const std::vector<state>& data, int type);
  void terminalGoalCB(const geometry_msgs::PoseStamped& msg);
  void pubState(const state& msg, const ros::Publisher pub);
  void stateCB(const snapstack_msgs::State& msg);
  // void odomCB(const nav_msgs::Odometry& odom_ptr);
  void modeCB(const faster_msgs::Mode& msg);
  void pubCB(const ros::TimerEvent& e);
  void replanCB(const ros::TimerEvent& e);

  visualization_msgs::Marker createMarkerLineStrip(Eigen::MatrixXd X);

  void clearMarkerActualTraj();
  void clearMarkerColoredTraj();
  void mapCB(const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_msg,
             const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_msg2);  // Callback for the occupancy pcloud
  void unkCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg);     // Callback for the unkown pcloud
  void pclCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg);
  void frontierCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg);

  void pubActualTraj();
  visualization_msgs::MarkerArray clearArrows();

  void updateInitialCond(int i);
  void yaw(double diff, snapstack_msgs::QuadGoal& quad_goal);

  void clearMarkerArray(visualization_msgs::MarkerArray* tmp, ros::Publisher* publisher);
  void publishJPSPath(vec_Vecf<3>& path, int i);
  void clearJPSPathVisualization(int i);

  void pubJPSIntersection(Eigen::Vector3d& inters);
  Eigen::Vector3d getFirstCollisionJPS(vec_Vecf<3>& path, bool* thereIsIntersection, int map = MAP,
                                       int type_return = RETURN_LAST_VERTEX);
  Eigen::Vector3d projectClickedGoal(Eigen::Vector3d& P1);

  void publishJPS2handIntersection(vec_Vecf<3> JPS2_fix, Eigen::Vector3d& inter1, Eigen::Vector3d& inter2,
                                   bool solvedFix);

  void createMoreVertexes(vec_Vecf<3>& path, double d);

  bool ARisInFreeSpace(int index);

  int findIndexR(int indexH);
  int findIndexH(bool& needToComputeSafePath);

  void publishPoly(const vec_E<Polyhedron<3>>& poly, int type);

  std::string world_name_ = "world";

  visualization_msgs::Marker R_;
  visualization_msgs::Marker I_;
  visualization_msgs::Marker E_;
  visualization_msgs::Marker M_;
  visualization_msgs::Marker H_;
  visualization_msgs::Marker A_;
  visualization_msgs::Marker setpoint_;

  ros::NodeHandle nh_;

  ros::Publisher pub_goal_jackal_;
  ros::Publisher pub_point_G_;
  ros::Publisher pub_point_G_term_;
  ros::Publisher pub_goal_;
  ros::Publisher pub_traj_whole_;
  ros::Publisher pub_traj_safe_;
  ros::Publisher pub_setpoint_;
  ros::Publisher pub_actual_traj_;
  ros::Publisher pub_path_jps1_;
  ros::Publisher pub_path_jps2_;
  ros::Publisher pub_path_jps_safe_;
  ros::Publisher pub_path_jps_whole_;
  ros::Publisher pub_intersectionI_;
  ros::Publisher pub_point_R_;
  ros::Publisher pub_point_M_;
  ros::Publisher pub_point_E_;
  ros::Publisher pub_point_H_;
  ros::Publisher pub_point_A_;
  ros::Publisher pub_traj_committed_colored_;
  ros::Publisher pub_traj_whole_colored_;
  ros::Publisher pub_traj_safe_colored_;

  ros::Publisher pub_planning_vis_;
  ros::Publisher pub_intersec_points_;
  ros::Publisher pub_jps_inters_;
  ros::Publisher pub_samples_safe_path_;
  ros::Publisher pub_log_;
  ros::Publisher poly_whole_pub_;
  ros::Publisher poly_safe_pub_;

  // ros::Publisher cvx_decomp_poly_uo_pub_;
  ros::Subscriber sub_goal_;
  ros::Subscriber sub_state_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_mode_;
  ros::Subscriber sub_vicon_;

  // Eigen::Vector3d accel_vicon_;

  ros::Subscriber sub_frontier_;
  ros::Timer pubCBTimer_;
  ros::Timer replanCBTimer_;

  parameters par_;  // where all the parameters are
  // snapstack_msgs::Cvx log_;  // to log all the data
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener* tfListener;
  std::string name_drone_;

  visualization_msgs::MarkerArray trajs_sphere_;  // all the trajectories generated in the sphere
  visualization_msgs::MarkerArray path_jps1_;
  visualization_msgs::MarkerArray path_jps2_;
  visualization_msgs::MarkerArray path_jps2_fix_;
  visualization_msgs::MarkerArray path_jps_safe_;
  visualization_msgs::MarkerArray path_jps_whole_;
  visualization_msgs::MarkerArray traj_committed_colored_;
  visualization_msgs::MarkerArray traj_whole_colored_;
  visualization_msgs::MarkerArray traj_safe_colored_;

  visualization_msgs::MarkerArray intersec_points_;
  visualization_msgs::MarkerArray samples_safe_path_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> occup_grid_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> unknown_grid_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
      MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  int actual_trajID_ = 0;
  // faster_msgs::Mode mode_;

  // Params specific to ROS wrapper
  double rviz_goal_height_ = 1.;  // [m] Assumed height for clicked 2d nav goals in Rviz  (overridden for ground robots)
};
