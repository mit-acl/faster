// Authors: Jesus Tordesillas
// Date: August 2018, December 2018, November 2019

#include "cvx.hpp"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/MarkerArray.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/StdVector>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <assert.h>
#include <stdlib.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>

using namespace JPS;
using namespace termcolor;

// Uncomment only one to choose the type of timer you want:
typedef ROSTimer MyTimer;
// typedef ROSWallTimer MyTimer;
// typedef Timer MyTimer;

CVX::CVX(ros::NodeHandle nh, ros::NodeHandle nh_replan_CB, ros::NodeHandle nh_pub_CB)
  : nh_(nh), nh_replan_CB_(nh_replan_CB), nh_pub_CB_(nh_pub_CB)
{
  std::cout << "Doing the setup\n";
  ros::param::param<bool>("~use_ff", par_.use_ff, 1);
  ros::param::param<bool>("~visual", par_.visual, true);

  ros::param::param<double>("~dc", par_.dc, 0.01);
  ros::param::param<double>("~goal_radius", par_.goal_radius, 0.2);
  ros::param::param<double>("~drone_radius", par_.drone_radius, 0.15);

  ros::param::param<int>("~N_safe", par_.N_safe, 10);
  ros::param::param<int>("~N_whole", par_.N_whole, 10);

  ros::param::param<double>("~factor_deltaT", par_.factor_deltaT, 1.5);
  ros::param::param<double>("~factor_min_deltaT", par_.factor_min_deltaT, 1.0);

  ros::param::param<int>("~min_states_deltaT", par_.min_states_deltaT, 0);

  ros::param::param<double>("~Ra", par_.Ra, 2.0);
  ros::param::param<double>("~Ra_max", par_.Ra_max, 2.5);
  ros::param::param<double>("~w_max", par_.w_max, 0.5);
  ros::param::param<double>("~alpha_filter_dyaw", par_.alpha_filter_dyaw, 0.8);
  ros::param::param<double>("~alpha_0_deg", par_.alpha_0_deg, 15);
  ros::param::param<double>("~z_ground", par_.z_ground, 0.0);
  ros::param::param<double>("~z_max", par_.z_max, 5.0);
  ros::param::param<double>("~inflation_jps", par_.inflation_jps, 0.8);
  ros::param::param<double>("~factor_jps", par_.factor_jps, 2);

  ros::param::param<double>("~v_max", par_.v_max, 2.0);
  ros::param::param<double>("~a_max", par_.a_max, 2.0);
  ros::param::param<double>("~j_max", par_.j_max, 10.0);

  ros::param::param<double>("~z_land", par_.z_land, 0.02);

  ros::param::param<double>("cntrl/spinup_time", spinup_time_, 0.5);

  ros::param::param<double>("~gamma_whole", par_.gamma_whole, 4.0);
  ros::param::param<double>("~gammap_whole", par_.gammap_whole, 4.0);
  ros::param::param<double>("~increment_whole", par_.increment_whole, 1.0);
  ros::param::param<double>("~gamma_safe", par_.gamma_safe, 4.0);
  ros::param::param<double>("~gammap_safe", par_.gammap_safe, 4.0);
  ros::param::param<double>("~increment_safe", par_.increment_safe, 1.0);

  ros::param::param<double>("~kw", par_.kw, 2.0);
  ros::param::param<double>("~kyaw", par_.kyaw, 2.0);
  ros::param::param<double>("~kdalpha", par_.kdalpha, 2.0);
  ros::param::param<double>("~kv", par_.kv, 2.0);
  ros::param::param<double>("~kdist", par_.kdist, 2.0);
  ros::param::param<double>("~kalpha", par_.kalpha, 2.0);
  ros::param::param<double>("~hack", par_.hack, 2.0);  // hacktodo

  ros::param::param<double>("~delta_a", par_.delta_a, 0.5);
  ros::param::param<double>("~delta_H", par_.delta_H, 0.7);

  ros::param::param<int>("~max_poly_whole", par_.max_poly_whole, 4);
  ros::param::param<int>("~max_poly_safe", par_.max_poly_safe, 4);
  ros::param::param<double>("~dist_max_vertexes", par_.dist_max_vertexes, 1.5);

  ros::param::param<int>("~gurobi_threads", par_.gurobi_threads, 1);
  ros::param::param<int>("~gurobi_verbose", par_.gurobi_verbose, 0);

  ros::param::param<bool>("~use_faster", par_.use_faster, true);
  ros::param::param<bool>("~keep_optimizing_after_found", par_.keep_optimizing_after_found, false);

  // And now obtain the parameters from the mapper
  std::vector<double> world_dimensions;
  std::vector<double> tmp{ 10, 10, 4 };
  ros::param::param<std::vector<double>>("~/mapper/world_dimensions", world_dimensions, tmp);
  ros::param::param<double>("~/mapper/resolution", par_.res, 0.15);

  par_.wdx = world_dimensions[0];
  par_.wdy = world_dimensions[1];
  par_.wdz = world_dimensions[2];

  std::cout << bold << green << "world_dimensions=" << world_dimensions << reset << std::endl;
  std::cout << bold << green << "resolution=" << par_.res << reset << std::endl;

  std::cout << "Parameters obtained" << std::endl;

  if (par_.N_safe <= par_.max_poly_safe + 2)
  {
    std::cout << bold << red << "Needed: N_safe>=max_poly+ 2 at least" << reset
              << std::endl;  // To decrease the probability of not finding a solution
    abort();
  }
  if (par_.N_whole <= par_.max_poly_whole + 2)
  {
    std::cout << bold << red << "Needed: N_whole>=max_poly + 2 at least" << reset
              << std::endl;  // To decrease the probability of not finding a solution
    abort();
  }

  if (par_.factor_jps * par_.res / 2.0 > par_.inflation_jps)
  {
    std::cout << bold << red << "Needed: par_.factor_jps * par_.res / 2 <= par_.inflation_jps" << reset
              << std::endl;  // If not JPS will find a solution between the voxels.
    abort();
  }

  /*  if (par_.Ra_max > (par_.wdx / 2.0) || (par_.Ra_max > par_.wdy / 2.0))
    {
      std::cout << bold << red << "Needed: par_.Ra_max > par_.wdx/2.0|| par_.Ra_max > par_.wdy/2.0" << reset
                << std::endl;  // To decrease the probability of not finding a solution
      abort();f
    }*/

  /*  if (par_.drone_radius <= 2 * par_.res)
    {
      std::cout << bold << red << "Needed: par_.drone_radius > 2*par_.res" << reset
                << std::endl;  // If not the convex decomposition finds polytopes between the voxels of the obstacles
      abort();
    }*/

  /*  if (par_.inflation_jps <= par_.res/2.0 + par_.drone_radius)
    {
      std::cout << bold << red << "Needed: par_.inflation_jps > par_.res/2.0 + par_.drone_radius" << reset
                << std::endl; //JPS should be run with at least drone_radius + half of the size of a voxel
      abort();
    }
  */

  std::cout << "Checks of parameters satisfied\n";

  optimized_ = false;
  // flight_mode_.mode = flight_mode_.NOT_FLYING;
  flight_mode_.mode = flight_mode_.GO;  // TODO (changed for the jackal)

  pub_goal_jackal_ = nh_.advertise<geometry_msgs::Twist>("goal_jackal", 1);
  pub_goal_ = nh_.advertise<acl_msgs::QuadGoal>("goal", 1);

  pub_traj_whole_ = nh_.advertise<nav_msgs::Path>("traj_whole", 1);
  pub_traj_safe_ = nh_.advertise<nav_msgs::Path>("traj_safe", 1);

  pub_setpoint_ = nh_.advertise<visualization_msgs::Marker>("setpoint", 1);
  pub_intersectionI_ = nh_.advertise<visualization_msgs::Marker>("intersection_I", 1);
  pub_point_G_ = nh_.advertise<geometry_msgs::PointStamped>("point_G", 1);
  pub_point_E_ = nh_.advertise<visualization_msgs::Marker>("point_E", 1);
  pub_point_R_ = nh_.advertise<visualization_msgs::Marker>("point_R", 1);
  pub_point_M_ = nh_.advertise<visualization_msgs::Marker>("point_M", 1);
  pub_point_H_ = nh_.advertise<visualization_msgs::Marker>("point_H", 1);
  pub_point_A_ = nh_.advertise<visualization_msgs::Marker>("point_A", 1);

  pub_actual_traj_ = nh_.advertise<visualization_msgs::Marker>("actual_traj", 1);
  pub_path_jps1_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps1", 1);
  pub_path_jps2_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps2", 1);
  pub_path_jps_whole_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps_whole", 1);
  pub_path_jps_safe_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps_safe", 1);
  cvx_whole_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("cvx_whole", 1, true);
  cvx_safe_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("cvx_safe", 1, true);
  pub_jps_inters_ = nh_.advertise<geometry_msgs::PointStamped>("jps_intersection", 1);
  // pub_intersec_points_ = nh_.advertise<visualization_msgs::MarkerArray>("intersection_points", 1);
  pub_log_ = nh_.advertise<acl_msgs::Cvx>("log_topic", 1);
  // pub_trajs_sphere_ = nh_.advertise<visualization_msgs::MarkerArray>("trajs_sphere", 1);
  pub_traj_committed_colored_ = nh_.advertise<visualization_msgs::MarkerArray>("traj_committed_colored", 1);
  pub_traj_whole_colored_ = nh_.advertise<visualization_msgs::MarkerArray>("traj_whole_colored", 1);
  pub_traj_safe_colored_ = nh_.advertise<visualization_msgs::MarkerArray>("traj_safe_colored", 1);

  occup_grid_sub_.subscribe(nh_, "occup_grid", 1);
  unknown_grid_sub_.subscribe(nh_, "unknown_grid", 1);
  sync_.reset(new Sync(MySyncPolicy(1), occup_grid_sub_, unknown_grid_sub_));
  sync_->registerCallback(boost::bind(&CVX::mapCB, this, _1, _2));

  sub_goal_ = nh_.subscribe("term_goal", 1, &CVX::goalCB, this);
  sub_mode_ = nh_.subscribe("flightmode", 1, &CVX::modeCB, this);
  sub_state_ = nh_.subscribe("state", 1, &CVX::stateCB, this);
  sub_odom_ = nh_.subscribe("odom", 1, &CVX::odomCB, this);

  pubCBTimer_ = nh_pub_CB_.createTimer(ros::Duration(par_.dc), &CVX::pubCB, this);

  replanCBTimer_ = nh_.createTimer(ros::Duration(par_.dc), &CVX::replanCB, this);

  // If you want another thread for the replanCB: replanCBTimer_ = nh_.createTimer(ros::Duration(par_.dc),
  // &CVX::replanCB, this);

  setpoint_ = getMarkerSphere(0.35, ORANGE_TRANS);
  R_ = getMarkerSphere(0.35, ORANGE_TRANS);
  I_ = getMarkerSphere(0.35, YELLOW);
  E_ = getMarkerSphere(0.35, RED);
  M_ = getMarkerSphere(0.35, BLUE);
  H_ = getMarkerSphere(0.35, GREEN);
  A_ = getMarkerSphere(0.35, RED);

  // mtx_G.lock();
  G_ << 0, 0, 0;
  // mtx_G.unlock();
  G_term_ << 0, 0, 0;

  quadGoal_.pos = vectorNull();
  quadGoal_.vel = vectorNull();
  quadGoal_.accel = vectorNull();
  quadGoal_.jerk = vectorNull();

  mtx_initial_cond.lock();
  stateA_.setZero();
  mtx_initial_cond.unlock();

  log_.total_dist = 0;

  markerID_ = 0;

  jps_manager_.setNumCells((int)par_.wdx / par_.res, (int)par_.wdy / par_.res, (int)par_.wdz / par_.res);
  jps_manager_.setFactorJPS(par_.factor_jps);
  jps_manager_.setResolution(par_.res);
  jps_manager_.setInflationJPS(par_.inflation_jps);
  jps_manager_.setZGroundAndZMax(par_.z_ground, par_.z_max);
  jps_manager_.setVisual(par_.visual);
  jps_manager_.setDroneRadius(par_.drone_radius);

  double max_values[3] = { par_.v_max, par_.a_max, par_.j_max };

  std::cout << "Going to do setup of sg_whole_\n";
  sg_whole_.setN(par_.N_whole);
  sg_whole_.createVars();
  sg_whole_.setDC(par_.dc);
  sg_whole_.set_max(max_values);
  sg_whole_.setMode(WHOLE_TRAJ);
  sg_whole_.setForceFinalConstraint(true);
  sg_whole_.setFactorInitialAndFinalAndIncrement(1, 10, par_.increment_whole);
  sg_whole_.setVerbose(par_.gurobi_verbose);
  sg_whole_.setThreads(par_.gurobi_threads);
  sg_whole_.setWMax(par_.w_max);

  std::cout << "Going to do setup of sg_safe_\n";
  sg_safe_.setN(par_.N_safe);
  sg_safe_.createVars();
  sg_safe_.setDC(par_.dc);
  sg_safe_.set_max(max_values);
  sg_safe_.setMode(WHOLE_TRAJ);  // SAFE_PATH
  sg_safe_.setForceFinalConstraint(false);
  sg_safe_.setFactorInitialAndFinalAndIncrement(1, 10, par_.increment_safe);
  sg_safe_.setVerbose(par_.gurobi_verbose);
  sg_safe_.setThreads(par_.gurobi_threads);
  sg_safe_.setWMax(par_.w_max);

  std::cout << "Done Setups\n";

  pclptr_unk_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pclptr_map_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  name_drone_ = ros::this_node::getNamespace();
  name_drone_.erase(0, 2);  // Erase slashes

  JPS_old_.clear();

  tfListener = new tf2_ros::TransformListener(tf_buffer_);
  // wait for body transform to be published before initializing
  ROS_INFO("Waiting for world to camera transform...");
  while (true)
  {
    try
    {
      tf_buffer_.lookupTransform("world", name_drone_ + "/camera", ros::Time::now(), ros::Duration(0.5));  //
      break;
    }
    catch (tf2::TransformException& ex)
    {
      // nothing
    }
  }
  clearMarkerActualTraj();
  ROS_INFO("Planner initialized");
}

void CVX::clearJPSPathVisualization(int i)
{
  switch (i)
  {
    case JPSk_NORMAL:
      clearMarkerArray(&path_jps1_, &pub_path_jps1_);
      break;
    case JPS2_NORMAL:
      clearMarkerArray(&path_jps2_, &pub_path_jps2_);
      break;
    case JPS_WHOLE:
      clearMarkerArray(&path_jps_whole_, &pub_path_jps_whole_);
      break;
    case JPS_SAFE:
      clearMarkerArray(&path_jps_safe_, &pub_path_jps_safe_);
      break;
  }
}

void CVX::clearMarkerArray(visualization_msgs::MarkerArray* tmp, ros::Publisher* publisher)
{
  if ((*tmp).markers.size() == 0)
  {
    return;
  }
  int id_begin = (*tmp).markers[0].id;
  // int id_end = (*path).markers[markers.size() - 1].id;

  for (int i = 0; i < (*tmp).markers.size(); i++)
  {
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::DELETE;
    m.id = i + id_begin;
    (*tmp).markers[i] = m;
  }

  (*publisher).publish(*tmp);
  (*tmp).markers.clear();
}

void CVX::publishJPSPath(vec_Vecf<3>& path, int i)
{
  /*vec_Vecf<3> traj, visualization_msgs::MarkerArray* m_array*/
  clearJPSPathVisualization(i);
  switch (i)
  {
    case JPSk_NORMAL:
      vectorOfVectors2MarkerArray(path, &path_jps1_, color(BLUE));
      pub_path_jps1_.publish(path_jps1_);
      break;

    case JPS2_NORMAL:
      vectorOfVectors2MarkerArray(path, &path_jps2_, color(RED));
      pub_path_jps2_.publish(path_jps2_);
      break;
    case JPS_WHOLE:
      vectorOfVectors2MarkerArray(path, &path_jps_whole_, color(GREEN));
      pub_path_jps_whole_.publish(path_jps_whole_);
      break;
    case JPS_SAFE:
      vectorOfVectors2MarkerArray(path, &path_jps_safe_, color(YELLOW));
      pub_path_jps_safe_.publish(path_jps_safe_);
      break;
  }
}

void CVX::vectorOfVectors2MarkerArray(vec_Vecf<3> traj, visualization_msgs::MarkerArray* m_array,
                                      std_msgs::ColorRGBA color, int type, std::vector<double> radii)
{
  // printf("In vectorOfVectors2MarkerArray\n");
  geometry_msgs::Point p_last = eigen2point(traj[0]);

  bool first_element = true;
  int i = 50000;  // large enough to prevent conflict with other markers
  int j = 0;

  for (const auto& it : traj)
  {
    i++;
    if (first_element and type == visualization_msgs::Marker::ARROW)  // skip the first element
    {
      first_element = false;
      continue;
    }

    visualization_msgs::Marker m;
    m.type = type;
    m.action = visualization_msgs::Marker::ADD;
    m.id = i;
    m.color = color;
    // m.scale.z = 1;

    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    geometry_msgs::Point p = eigen2point(it);
    if (type == visualization_msgs::Marker::ARROW)
    {
      m.scale.x = 0.02;
      m.scale.y = 0.04;
      m.points.push_back(p_last);
      m.points.push_back(p);
      // std::cout << "pushing marker\n" << m << std::endl;
      p_last = p;
    }
    else
    {
      double scale = 0.1;  // Scale is the diameter of the sphere
      if (radii.size() != 0)
      {  // If argument provided
        scale = 2 * radii[j];
      }
      m.scale.x = scale;
      m.scale.y = scale;
      m.scale.z = scale;
      m.pose.position = p;
    }
    (*m_array).markers.push_back(m);
    j = j + 1;
  }
}

void CVX::goalCB(const acl_msgs::TermGoal& msg)
{
  printf("NEW GOAL************************************************\n");
  log_.total_dist = 0;
  mtx_G_term.lock();
  G_term_ = Eigen::Vector3d(msg.pos.x, msg.pos.y, msg.pos.z);
  mtx_G_term.unlock();
  // std::cout << "G_term_=\n" << G_term_ << std::endl;
  mtx_G.lock();
  mtx_state.lock();
  Eigen::Vector3d temp = state_.pos;
  G_ = projectPointToBox(temp, G_term_, par_.wdx, par_.wdy, par_.wdz);
  pubTerminalGoal();
  mtx_state.unlock();
  mtx_G.unlock();

  quadGoal_.yaw = current_yaw_;

  status_ = (status_ == GOAL_REACHED) ? YAWING : TRAVELING;
  mtx_planner_status_.lock();
  planner_status_ = START_REPLANNING;
  mtx_planner_status_.unlock();
  force_reset_to_0_ = true;
  // std::cout << "GCB: planner_status_ = START_REPLANNING\n";
  goal_click_initialized_ = true;
  clearMarkerActualTraj();
  // std::cout << "Exiting from goalCB\n";
}

void CVX::yaw(double diff, acl_msgs::QuadGoal& quad_goal)
{
  saturate(diff, -par_.dc * par_.w_max, par_.dc * par_.w_max);
  double dyaw_not_filtered;

  dyaw_not_filtered = copysign(1, diff) * par_.w_max;

  dyaw_filtered_ = (1 - par_.alpha_filter_dyaw) * dyaw_not_filtered + par_.alpha_filter_dyaw * dyaw_filtered_;
  quad_goal.dyaw = dyaw_filtered_;

  quad_goal.yaw += dyaw_filtered_ * par_.dc;
}

void CVX::createMoreVertexes(vec_Vecf<3>& path, double d)
{
  for (int j = 0; j < path.size() - 1; j++)
  {
    double dist = (path[j + 1] - path[j]).norm();
    int vertexes_to_add = floor(dist / d);
    Eigen::Vector3d v = (path[j + 1] - path[j]).normalized();
    // std::cout << "Vertexes to add=" << vertexes_to_add << std::endl;
    if (dist > d)
    {
      for (int i = 0; i < vertexes_to_add; i++)
      {
        path.insert(path.begin() + j + 1, path[j] + v * d);
        j = j + 1;
      }
    }
  }
}

void CVX::replanCB(const ros::TimerEvent& e)
{
  // par_.use_faster = !par_.use_faster;  // hacktodo

  // print_status();
  MyTimer replanCB_t(true);
  MyTimer otherStuff_t(true);

  if (!state_initialized_ || !kdtree_map_initialized_ || !kdtree_unk_initialized_ || !goal_click_initialized_)
  {
    ROS_WARN("Waiting to initialize kdTree_map and/or kdTree_unk and/or goal_click and/or state_");

    std::cout << "state_initialized_= " << state_initialized_ << std::endl;
    std::cout << "kdtree_map_initialized_= " << kdtree_map_initialized_ << std::endl;
    std::cout << "kdtree_unk_initialized_= " << kdtree_unk_initialized_ << std::endl;
    std::cout << "goal_click_initialized_= " << goal_click_initialized_ << std::endl;
    return;
  }

  sg_whole_.ResetToNormalState();
  sg_safe_.ResetToNormalState();

  mtx_state.lock();
  state state_local = state_;  // Local copy of state
  mtx_state.unlock();

  mtx_G.lock();
  mtx_G_term.lock();
  G_ = projectPointToBox(state_local.pos, G_term_, par_.wdx, par_.wdy, par_.wdz);
  pubTerminalGoal();
  Eigen::Vector3d G = G_;            // Local copy of the terminal goal
  Eigen::Vector3d G_term = G_term_;  // Local copy of the terminal terminal goal
  mtx_G.unlock();
  mtx_G_term.unlock();

  double dist_to_goal = (G_term - state_local.pos).norm();

  double dist_to_goal_commanded = (G_term - state_local.pos).norm();

  // std::cout << "G_term" << G_term.transpose() << std::endl;
  // std::cout << "state_pos" << state_pos.transpose() << std::endl;
  // std::cout << "dist_to_goal_commanded=" << dist_to_goal_commanded << std::endl;
  if (dist_to_goal_commanded < par_.goal_radius)
  {
    if (takeoff_done_ == false)
    {
      std::cout << bold << green << "Takeoff_done_!" << std::endl;
      takeoff_done_ = true;
    }
  }

  if (dist_to_goal < par_.goal_radius && status_ != GOAL_REACHED)
  {
    status_ = GOAL_REACHED;

    printf("STATUS=GOAL_REACHED\n");
  }

  if (status_ == GOAL_SEEN || status_ == GOAL_REACHED || (status_ == YAWING && optimized_ == true) ||
      (planner_status_ == REPLANNED && par_.keep_optimizing_after_found == false))  // TODO (changed for the jackal,
                                                                                    // added optimized_ == true)
  {
    /*    printf("No replanning needed because planner_status_=%d and/or status_=%d \n", planner_status_, status_);
        printf("or because status_=%d\n", status_);*/
    return;
  }

  std::cout << bold << on_red << "************IN REPLAN CB*********" << reset << std::endl;
  // print_status();

  ///////////////////////////////////////////////////////////////////////////////

  double dist1 = 0, dist2 = 0, J1 = 0, J2 = 0, JPrimj1 = 0, JPrimj2 = 0, JPrimv1 = 0, JPrimv2 = 0, JDist1 = 0,
         JDist2 = 0;
  J1 = std::numeric_limits<double>::max();
  J2 = std::numeric_limits<double>::max();

  // double x0[9];
  state x0;
  if (X_initialized_)  // Needed to skip the first time (X_ still not initialized)
  {
    if (planner_status_ != REPLANNED)  // If I've already replanned, just keep finding new trajectories but with the
                                       // previous initial condition. The last one found when P reaches A will the one
                                       // be chosen
    {
      mtx_k.lock();
      mtx_offsets.lock();
      // k_initial_cond_1_ = std::min(k_ + deltaT_, (int)(X_.rows() - 1));

      log_.entered_safe_path = 0;

      k_initial_cond_1_ = std::min(k_ + deltaT_, (int)(X_.rows() - 1));
      if (par_.use_smart_deltaT)
      {
        int states_remaining = (indexR_ - k_);  // states remaining to start executing the rescue path.
        k_initial_cond_1_ = (states_remaining > deltaT_min_) ?
                                std::min(std::max(indexR_ - 1, 0), (int)(X_.rows() - 1)) :
                                k_initial_cond_1_;
        std::cout << "states_remaining=" << states_remaining << std::endl;
        std::cout << "k_initial_cond_1_=" << k_initial_cond_1_ << "/ (safe path is " << indexR_ << " --> "
                  << (int)(X_.rows() - 1) << " )" << std::endl;
      }

      if (k_initial_cond_1_ >= indexR_ && status_ == TRAVELING)  // Here index_R_ has the value of the previous
                                                                 // replanning step
      {
        ROS_WARN("Switched to the SAFE PATH!!");
        log_.entered_safe_path = 1;
      }

      log_.deltaT_percentage = (k_initial_cond_1_ - k_) / (1.0 * (indexR_ - k_));
      log_.deltaT = deltaT_;
      // log_.deltaP2Rold = deltaTp_old_ - k_;
      log_.k = k_;
      // log_.deltaTp_old = deltaTp_old_;
      mtx_offsets.unlock();
      mtx_k.unlock();
      // printf("Ahora mismo, k_initial_cond_1=%d\n", k_initial_cond_1_);

      updateInitialCond(k_initial_cond_1_);
    }

    mtx_initial_cond.lock();

    x0 = stateA_;
    mtx_initial_cond.unlock();
  }
  else
  {
    std::cout << bold << "X is not initialized yet" << reset << std::endl;
    x0 = state_local;
  }

  if (par_.visual)
  {
    A_.header.stamp = ros::Time::now();
    A_.pose.position.x = stateA_.pos.x();
    A_.pose.position.y = stateA_.pos.y();
    A_.pose.position.z = stateA_.pos.z();
    pub_point_A_.publish(A_);
  }

  Eigen::Vector3d A = x0.pos;

  double ra = std::min((dist_to_goal - 0.001), par_.Ra);

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Solve JPS //////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////
  bool solvedjpsa = false;
  MyTimer timer_jps(true);

  vec_Vecf<3> JPSk = jps_manager_.solveJPS3D(A, G, &solvedjpsa, 1);
  clearJPSPathVisualization(2);

  if (solvedjpsa == false)
  {
    ROS_ERROR("JPSa didn't find a solution");
    return;
  }

  log_.JPStotal_ms = timer_jps.ElapsedMs();

  // Find the elements of JPSk inside the shpere S
  ra = std::min((dist_to_goal - 0.001), par_.Ra_max);  // radius of the sphere S
  bool noPointsOutsideSphere1;
  int li1;  // last index inside the sphere of JPSk
  Eigen::Vector3d E = getFirstIntersectionWithSphere(JPSk, ra, JPSk[0], &li1, &noPointsOutsideSphere1);
  vec_Vecf<3> JPSk_inside_sphere(JPSk.begin(), JPSk.begin() + li1 + 1);
  if (noPointsOutsideSphere1 == false)
  {
    JPSk_inside_sphere.push_back(E);
  }
  // createMoreVertexes in case dist between vertexes is too big
  createMoreVertexes(JPSk_inside_sphere, par_.dist_max_vertexes);

  //////////////////////////////////////////////////////////////////////////
  ///////////////// Solve with GUROBI Whole trajectory /////////////////////
  //////////////////////////////////////////////////////////////////////////

  if (par_.use_faster == true)
  {
    // std::cout << bold << green << "***********WHOLE TRAJ*********************" << reset << std::endl;

    vec_Vecf<3> JPS_whole = JPSk_inside_sphere;
    deleteVertexes(JPS_whole, par_.max_poly_whole);
    E = JPS_whole[JPS_whole.size() - 1];

    // Convex Decomp around JPS_whole
    MyTimer cvx_ellip_decomp_t(true);
    jps_manager_.cvxEllipsoidDecomp(JPS_whole, OCCUPIED_SPACE, l_constraints_whole_, cvx_whole_pub_);
    log_.cvx_decomp_whole_ms = cvx_ellip_decomp_t.ElapsedMs();

    // Check if G is inside poly_whole
    bool isGinside_whole = l_constraints_whole_[l_constraints_whole_.size() - 1].inside(G);
    E = (isGinside_whole == true) ? G : E;
    // TODO hack for the jackal
    E = (optimized_ == false) ? state_local.pos : E;
    // End of hack

    // Set Initial cond, Final cond, and polytopes for the whole traj
    state xf;
    xf.setPos(E);
    sg_whole_.setXf(xf);
    sg_whole_.setX0(x0);
    sg_whole_.setPolytopes(l_constraints_whole_);

    // Solve with Gurobi
    MyTimer whole_gurobi_t(true);
    bool solved_whole = sg_whole_.genNewTraj();
    log_.gurobi_whole_ms = sg_whole_.runtime_ms_;
    log_.gurobi_whole_ms_mine = whole_gurobi_t.ElapsedMs();
    log_.gurobi_whole_trials = sg_whole_.trials_;
    log_.gurobi_whole_dt = sg_whole_.dt_;
    log_.gurobi_whole_factor = sg_whole_.factor_that_worked_;

    if (solved_whole == false)
    {
      ROS_ERROR("No solution found for the whole trajectory");
      return;
    }

    // Get Results
    MyTimer fill_whole_t(true);
    sg_whole_.fillXandU();

    if (par_.visual == true)
    {
      clearJPSPathVisualization(JPS_WHOLE);
      publishJPSPath(JPS_whole, JPS_WHOLE);
      E_.header.stamp = ros::Time::now();
      E_.pose.position.x = E(0);
      E_.pose.position.y = E(1);
      E_.pose.position.z = E(2);
      pub_point_E_.publish(E_);
    }

    // std::cout << bold << blue << "WholeGurobi:  " << std::fixed << whole_gurobi_t << "ms, (" << std::fixed
    //          << sg_whole_.runtime_ms_ << " ms), " << reset << sg_whole_.trials_ << " trials (dt=" << sg_whole_.dt_
    //          << "), f_worked=" << std::setprecision(2) << sg_whole_.factor_that_worked_ << std::endl;

    // std::cout << bold << blue << "Fill Whole:  " << std::fixed << fill_whole_t << "ms" << reset << std::endl;
  }
  else
  {  // Dummy whole trajectory
    sg_whole_.X_temp_ = Eigen::MatrixXd::Zero(2, 9);
    sg_whole_.U_temp_ = Eigen::MatrixXd::Zero(2, 3);
  }

  //////////////////////////////////////////////////////////////////////////
  ///////////////// Solve with GUROBI Safe trajectory /////////////////////
  //////////////////////////////////////////////////////////////////////////

  vec_Vecf<3> JPSk_inside_sphere_tmp = JPSk_inside_sphere;
  bool thereIsIntersection2;
  Eigen::Vector3d M = getFirstCollisionJPS(JPSk_inside_sphere_tmp, &thereIsIntersection2, UNKNOWN_MAP,
                                           RETURN_INTERSECTION);  // results saved in JPSk_inside_sphere_tmp

  B_ = M;
  // std::cout << "Point M is:" << M.transpose() << std::endl;
  if (par_.visual)
  {
    M_.header.stamp = ros::Time::now();
    M_.pose.position.x = M(0);
    M_.pose.position.y = M(1);
    M_.pose.position.z = M(2);
    pub_point_M_.publish(M_);
  }

  ////////////////
  bool needToComputeSafePath;
  int indexH = findIndexH(needToComputeSafePath);

  if (par_.use_faster == false)
  {
    needToComputeSafePath = true;
  }

  log_.needToComputeSafePath = (int)needToComputeSafePath;

  if (needToComputeSafePath == false and takeoff_done_ == true)
  {
    // std::cout << red << bold << "No Rescue path" << reset << std::endl;
    indexR_ = indexH;
    // No need of a rescue path
  }
  else
  {
    indexR_ = findIndexR(indexH);

    MyTimer otherStuff3_t(true);
    mtx_X_U_temp.lock();

    Eigen::Vector3d posH(sg_whole_.X_temp_(indexH, 0), sg_whole_.X_temp_(indexH, 1), sg_whole_.X_temp_(indexH, 2));

    // std::cout << red << bold << "Below2" << reset << std::endl;

    // Find the point R (init of the Safe trajectory)
    state R;
    Eigen::Matrix<double, 9, 1> tmp = X_temp_.block(indexR_, 0, 1, 9);
    R.setState(tmp);
    mtx_X_U_temp.unlock();

    /*    MyTimer check_collision_AR_t(true);
        if (ARisInFreeSpace(indexR_) == false and takeoff_done_ == true)
        {
          std::cout << red << bold << "The piece A-->R is not in Free Space" << std::endl;
          return;
        }

        std::cout << bold << blue << "Check collision with AR:  " << std::fixed << check_collision_AR_t << "ms" << reset
                  << std::endl;*/
    JPSk_inside_sphere_tmp[0] = R.pos;

    if (par_.use_faster == false)
    {
      JPSk_inside_sphere_tmp[0] = A;
    }

    vec_Vecf<3> JPS_safe = JPSk_inside_sphere_tmp;

    // delete extra vertexes
    deleteVertexes(JPS_safe, par_.max_poly_safe);
    M = JPS_safe[JPS_safe.size() - 1];

    // compute convex decomposition of JPS_safe
    MyTimer cvx_ellip_decomp2_t(true);
    jps_manager_.cvxEllipsoidDecomp(JPS_safe, UNKOWN_AND_OCCUPIED_SPACE, l_constraints_safe_, cvx_safe_pub_);
    publishJPSPath(JPS_safe, JPS_SAFE);

    log_.cvx_decomp_safe_ms = cvx_ellip_decomp2_t.ElapsedMs();

    if (par_.visual)
    {
      /*      R_.header.stamp = ros::Time::now();
            R_.pose.position.x = posR(0);
            R_.pose.position.y = posR(1);
            R_.pose.position.z = posR(2);
            pub_point_R_.publish(R_);*/

      H_.header.stamp = ros::Time::now();
      H_.pose.position.x = posH(0);
      H_.pose.position.y = posH(1);
      H_.pose.position.z = posH(2);
      pub_point_H_.publish(H_);
    }

    bool isGinside = l_constraints_safe_[l_constraints_safe_.size() - 1].inside(G);
    M = (isGinside == true) ? G : M;

    // TODO hack for the jackal
    M = (optimized_ == false) ? state_local.pos : M;
    // End of hack

    state x0_safe;
    x0_safe = R;

    state xf_safe;
    xf_safe.pos = M;  // only used to compute dt

    if (par_.use_faster == false)
    {
      x0_safe = stateA_;
    }

    // std::cout << "Polytopes set=" << l_constraints_safe_.size() << std::endl;

    bool isMinside = l_constraints_safe_[l_constraints_safe_.size() - 1].inside(M);

    bool shouldForceFinalConstraint_for_Safe =
        (par_.use_faster == false) ? true : false;  // hacktodo pero quiza ahora es mejor, era ((isMinside &&
                                                    // takeoff_done_) || par_.use_faster == false) ? true : false;

    std::cout << "shouldForceFinalConstraint_for_Safe=" << shouldForceFinalConstraint_for_Safe << std::endl;
    std::cout << "par_.use_faster=" << par_.use_faster << std::endl;
    std::cout << "takeoff_done_=" << takeoff_done_ << std::endl;
    std::cout << "isMinside=" << isMinside << std::endl;

    if (l_constraints_safe_[0].inside(x0_safe.pos) == false)
    {
      ROS_ERROR("First point of safe traj is outside");
    }

    sg_safe_.setXf(xf_safe);
    sg_safe_.setX0(x0_safe);
    sg_safe_.setPolytopes(l_constraints_safe_);
    sg_safe_.setForceFinalConstraint(shouldForceFinalConstraint_for_Safe);
    MyTimer safe_gurobi_t(true);
    bool solved_safe = sg_safe_.genNewTraj();

    if (solved_safe == false)
    {
      std::cout << red << "No solution found for the safe path" << reset << std::endl;
      return;
    }

    // Get the solution
    sg_safe_.fillXandU();

    /*    log_.gurobi_safe_ms = sg_safe_.runtime_ms_;
        log_.gurobi_safe_ms_mine = safe_gurobi_t.ElapsedMs();
        log_.gurobi_safe_trials = sg_safe_.trials_;
        log_.gurobi_safe_dt = sg_safe_.dt_;
        log_.gurobi_safe_factor = sg_safe_.factor_that_worked_;*/

    //    MyTimer fill_safe_t(true);
    // std::cout << bold << blue << "Fill Safe:  " << std::fixed << fill_safe_t << "ms" << reset << std::endl;

    // std::cout << bold << blue << "SafeGurobi:  " << std::fixed << safe_gurobi_t << "ms, (" << std::fixed
    //           << sg_safe_.runtime_ms_ << " ms), " << reset << sg_safe_.trials_ << " trials (dt=" << sg_safe_.dt_
    //           << "), f_worked=" << std::setprecision(2) << sg_safe_.factor_that_worked_ << std::endl;

  }  // End of need to compute the rescue path

  ///////////////////////////////////////////////////////////
  ///////////////       MERGE RESULTS    ////////////////////
  ///////////////////////////////////////////////////////////

  // std::cout << red << bold << "Merging Results" << reset << std::endl;

  MyTimer otherStuff2_t(true);

  mtx_X_U_temp.lock();

  //# of cols are the same for all the trajectories
  int colsU = sg_whole_.U_temp_.cols();
  int colsX = sg_whole_.X_temp_.cols();

  int rows_X_safe = (needToComputeSafePath == true) ? sg_safe_.X_temp_.rows() : 0;
  int rows_U_safe = (needToComputeSafePath == true) ? sg_safe_.U_temp_.rows() : 0;
  int rows_X_whole = sg_whole_.X_temp_.rows();
  int rows_U_whole = sg_whole_.U_temp_.rows();

  if (par_.use_faster == true)
  {
    U_temp_.conservativeResize(indexR_ + rows_U_safe + 1, colsU);  // Set the number of rows of U_temp_
    X_temp_.conservativeResize(indexR_ + rows_X_safe + 1, colsX);  // Set the number of rows of X_temp_

    U_temp_.block(0, 0, indexR_ + 1, colsU) = sg_whole_.U_temp_.block(0, 0, indexR_ + 1, colsU);
    X_temp_.block(0, 0, indexR_ + 1, colsX) = sg_whole_.X_temp_.block(0, 0, indexR_ + 1, colsX);

    if (needToComputeSafePath == true)  // If the safe path has been computed
    {
      U_temp_.block(indexR_ + 1, 0, rows_U_safe, colsU) = sg_safe_.U_temp_;
      X_temp_.block(indexR_ + 1, 0, rows_X_safe, colsX) = sg_safe_.X_temp_;
    }
  }
  else
  {  // Plan only in known space

    U_temp_.conservativeResize(rows_U_safe, colsU);  // Set the number of rows of U_temp_
    X_temp_.conservativeResize(rows_X_safe, colsX);  // Set the number of rows of X_temp_

    U_temp_ = sg_safe_.U_temp_;
    X_temp_ = sg_safe_.X_temp_;
  }

  mtx_X_U_temp.unlock();

  log_.indexR_percentage = indexR_ / (1.0 * indexH);
  log_.indexR = indexR_;
  log_.indexH = indexH;

  if (par_.visual == true)
  {
    if (needToComputeSafePath == true)
    {
      pubTraj(sg_safe_.X_temp_, SAFE);
    }
    else
    {  // publish only the last point of the whole trajectory as a "dummy" rescue path
      Eigen::MatrixXd tmp = sg_whole_.X_temp_.block(rows_X_whole - 1, 1, 1, colsX - 1);
      pubTraj(tmp, SAFE);
    }
    pubTraj(sg_whole_.X_temp_, WHOLE);
    pubTraj(X_temp_, COMMITTED_COLORED);
    pubTraj(sg_whole_.X_temp_, WHOLE_COLORED);
    if (needToComputeSafePath == true)
    {
      pubTraj(sg_safe_.X_temp_, SAFE_COLORED);
    }
  }

  ///////////////////////////////////////////////////////////
  ///////////////       OTHER STUFF    //////////////////////
  ///////////////////////////////////////////////////////////
  // std::cout << red << bold << "Above8" << reset << std::endl;
  JPS_old_ = JPSk;
  mtx_k.lock();
  k_initial_cond_ = k_initial_cond_1_;
  // printf("Ahora mismo, k_initial_cond_=%d and k_=%d\n", k_initial_cond_, k_);
  mtx_k.unlock();

  optimized_ = true;
  mtx_planner_status_.lock();
  planner_status_ = REPLANNED;

  /*  if (takeoff_done_ == true)
    {
      planner_status_ = START_REPLANNING;  // hacktodo
    }*/

  mtx_planner_status_.unlock();
  // printf("ReplanCB: planner_status_ = REPLANNED\n");
  // std::cout << red << bold << "Above9" << reset << std::endl;
  Eigen::Vector3d F;
  F << X_temp_(X_temp_.rows() - 1, 0), X_temp_(X_temp_.rows() - 1, 1),
      X_temp_(X_temp_.rows() - 1, 2);  // Final point of the safe path (\equiv final point of the comitted path)

  double dist = (G_term_ - F).norm();
  // std::cout << "******Distance=" << dist << std::endl;
  if (dist < par_.goal_radius)
  {
    std::cout << "Changed to GoalSeen" << std::endl;
    status_ = GOAL_SEEN;
  }

  // std::cout << bold << blue << "OtherStuff 2:  " << std::fixed << otherStuff2_t << "ms" << reset << std::endl;

  log_.total_replanning_ms = replanCB_t.ElapsedMs();
  // std::cout << bold << blue << "TOTAL REPLANNING CB:  " << std::fixed << replanCB_t << "ms" << reset << std::endl;

  mtx_offsets.lock();

  // deltaTp_old_ = deltaTp_;

  int states_last_replan = ceil(replanCB_t.ElapsedMs() / (par_.dc * 1000));  // Number of states that
                                                                             // would have been needed for
                                                                             // the last replan

  //  std::cout << "states_last_replan:  " << std::fixed << states_last_replan << " states" << std::endl;
  // std::cout << "min_states_deltaTp:  " << std::fixed << par_.min_states_deltaTp << " states" << std::endl;
  // deltaTp_ = std::max(par_.factor_deltaTp * states_last_replan, (double)par_.min_states_deltaTp);  // deltaTp
  // std::cout << "Next deltaTp_:  " << std::fixed << deltaTp_ << " states" << std::endl;

  if (planner_status_ != REPLANNED)  // If already have a solution, keep using the same deltaT_
  {
    deltaT_ = std::max(par_.factor_deltaT * states_last_replan,
                       (double)par_.min_states_deltaT);  // Delta_t

    deltaT_min_ = par_.factor_min_deltaT * states_last_replan;
  }

  mtx_offsets.unlock();

  // Time allocation
  double new_init_whole = std::max(sg_whole_.factor_that_worked_ - par_.gamma_whole, 1.0);  // 1;  // hacktodo
  double new_final_whole = sg_whole_.factor_that_worked_ + par_.gammap_whole;  // high end factor is not a problem
  sg_whole_.setFactorInitialAndFinalAndIncrement(new_init_whole, new_final_whole, par_.increment_whole);

  double new_init_safe = std::max(sg_safe_.factor_that_worked_ - par_.gamma_safe, 1.0);  // 1;  // hacktodo
  double new_final_safe = sg_safe_.factor_that_worked_ + par_.gammap_safe;  // high end factor is not a problem
  sg_safe_.setFactorInitialAndFinalAndIncrement(new_init_safe, new_final_safe, par_.increment_safe);

  // std::cout << "Next factors: W: " << std::fixed << std::setprecision(2) << new_init_whole << "-->" <<
  // new_final_whole
  //           << " R: " << new_init_safe << "-->" << new_final_safe << reset << std::endl;

  log_.header.stamp = ros::Time::now();

  if (status_ != GOAL_REACHED)
  {
    pub_log_.publish(log_);
  }
  return;
}

int CVX::findIndexR(int indexH)
{
  // Ignore z to obtain this heuristics (if not it can become VERY conservative)
  mtx_X_U_temp.lock();
  Eigen::Vector2d posHk;
  posHk << sg_whole_.X_temp_(indexH, 0), sg_whole_.X_temp_(indexH, 1);
  int indexR = indexH;
  for (int i = 0; i <= indexH; i = i + 1)  // Loop from A to H
  {
    Eigen::Vector2d vel;
    vel << sg_whole_.X_temp_(i, 3), sg_whole_.X_temp_(i, 4);

    Eigen::Vector2d pos;
    pos << sg_whole_.X_temp_(i, 0), sg_whole_.X_temp_(i, 1);

    Eigen::Vector2d braking_distance =
        (vel.array() * (posHk - pos).array()).sign() * vel.array().square() / (2 * par_.delta_a * par_.a_max);

    // std::cout << "braking_distance=" << braking_distance.transpose() << std::endl;
    // std::cout << "(posHk - pos).cwiseAbs().array())=" << (posHk - pos).cwiseAbs().array().transpose() << std::endl;

    bool thereWillBeCollision =
        (braking_distance.array() > (posHk - pos).cwiseAbs().array()).any();  // Any of the braking distances (in x, y,
                                                                              // z) is bigger than the distance to the
                                                                              // obstacle in that direction
    if (thereWillBeCollision)
    {
      indexR = i;

      if (indexR == 0)
      {
        std::cout << bold << red << "R was taken in A" << reset << std::endl;
      }

      break;
    }
  }
  std::cout << red << bold << "indexR=" << indexR << " /" << sg_whole_.X_temp_.rows() - 1 << reset << std::endl;
  // std::cout << red << bold << "indexH=" << indexH << " /" << sg_whole_.X_temp_.rows() - 1 << reset << std::endl;
  mtx_X_U_temp.unlock();

  return indexR;
}

int CVX::findIndexH(bool& needToComputeSafePath)
{
  int n = 1;  // find one neighbour
  std::vector<int> pointIdxNKNSearch(n);
  std::vector<float> pointNKNSquaredDistance(n);

  needToComputeSafePath = false;

  mtx_unk.lock();
  mtx_X_U_temp.lock();
  int indexH = sg_whole_.X_temp_.rows() - 1;

  for (int i = 0; i < sg_whole_.X_temp_.rows(); i = i + 10)
  {  // Sample points along the trajectory

    pcl::PointXYZ searchPoint(sg_whole_.X_temp_(i, 0), sg_whole_.X_temp_(i, 1), sg_whole_.X_temp_(i, 2));

    if (kdtree_unk_.nearestKSearch(searchPoint, n, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
      if (sqrt(pointNKNSquaredDistance[0]) < par_.drone_radius)
      {
        needToComputeSafePath = true;  // There is intersection, so there is need to compute rescue path
        indexH = (int)(par_.delta_H * i);
        break;
      }
    }
  }
  std::cout << red << bold << "indexH=" << indexH << " /" << sg_whole_.X_temp_.rows() - 1 << reset << std::endl;
  mtx_unk.unlock();
  mtx_X_U_temp.unlock();

  return indexH;
}

bool CVX::ARisInFreeSpace(int index)
{  // We have to check only against the unkown space (A-R won't intersect the obstacles for sure)

  // std::cout << "In ARisInFreeSpace, radius_drone= " << par_.drone_radius << std::endl;
  int n = 1;  // find one neighbour

  std::vector<int> pointIdxNKNSearch(n);
  std::vector<float> pointNKNSquaredDistance(n);

  bool isFree = true;

  // std::cout << "Before mtx_unk" << std::endl;
  mtx_unk.lock();
  mtx_X_U_temp.lock();
  // std::cout << "After mtx_unk. index=" << index << std::endl;
  for (int i = 0; i < index; i = i + 10)
  {  // Sample points along the trajectory
     // std::cout << "i=" << i << std::endl;
    pcl::PointXYZ searchPoint(sg_whole_.X_temp_(i, 0), sg_whole_.X_temp_(i, 1), sg_whole_.X_temp_(i, 2));

    Eigen::Vector3d novale;
    novale << sg_whole_.X_temp_(i, 0), sg_whole_.X_temp_(i, 1), sg_whole_.X_temp_(i, 2);
    // std::cout << "Point =" << novale.transpose() << std::endl;

    if (kdtree_unk_.nearestKSearch(searchPoint, n, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
      if (sqrt(pointNKNSquaredDistance[0]) < 0.2)
      {  // TODO: 0.2 is the radius of the drone.
        std::cout << "A->R collides, with d=" << sqrt(pointNKNSquaredDistance[0])
                  << ", radius_drone=" << par_.drone_radius << std::endl;
        isFree = false;
        break;
      }
    }
  }

  mtx_unk.unlock();
  mtx_X_U_temp.unlock();

  return isFree;
}

void CVX::print_status()
{
  switch (status_)
  {
    case YAWING:
      std::cout << bold << "status_=YAWING" << reset << std::endl;
      break;
    case TRAVELING:
      std::cout << bold << "status_=TRAVELING" << reset << std::endl;
      break;
    case GOAL_SEEN:
      std::cout << bold << "status_=GOAL_SEEN" << reset << std::endl;
      break;
    case GOAL_REACHED:
      std::cout << bold << "status_=GOAL_REACHED" << reset << std::endl;
      break;
  }

  switch (planner_status_)
  {
    case FIRST_PLAN:
      std::cout << bold << "planner_status_=FIRST_PLAN" << reset << std::endl;
      break;
    case START_REPLANNING:
      std::cout << bold << "planner_status_=START_REPLANNING" << reset << std::endl;
      break;
    case REPLANNED:
      std::cout << bold << "planner_status_=REPLANNED" << reset << std::endl;
      break;
  }

  switch (flight_mode_.mode)
  {
    case flight_mode_.NOT_FLYING:
      std::cout << bold << "flight_mode_=NOT_FLYING" << reset << std::endl;
      break;
    case flight_mode_.TAKEOFF:
      std::cout << bold << "flight_mode_=TAKEOFF" << reset << std::endl;
      break;
    case flight_mode_.LAND:
      std::cout << bold << "flight_mode_=LAND" << reset << std::endl;
      break;
    case flight_mode_.INIT:
      std::cout << bold << "flight_mode_=INIT" << reset << std::endl;
      break;
    case flight_mode_.GO:
      std::cout << bold << "flight_mode_=GO" << reset << std::endl;
      break;
    case flight_mode_.ESTOP:
      std::cout << bold << "flight_mode_=ESTOP" << reset << std::endl;
      break;
    case flight_mode_.KILL:
      std::cout << bold << "flight_mode_=KILL" << reset << std::endl;
      break;
  }

  std::cout << bold << "optimized_=" << optimized_ << reset << std::endl;
  std::cout << bold << "takeoff_done_=" << takeoff_done_ << reset << std::endl;
}

void CVX::pubCB(const ros::TimerEvent& e)
{
  // print_status();
  mtx_goals.lock();
  // printf("In PUBCB: GOing to publish\n");

  if (flight_mode_.mode == flight_mode_.LAND)
  {
    double d = sqrt(pow(quadGoal_.pos.z - par_.z_land, 2));
    if (d < 0.1)
    {
      ros::Duration(1.0).sleep();
      flight_mode_.mode = flight_mode_.NOT_FLYING;
    }
  }

  quadGoal_.header.stamp = ros::Time::now();
  quadGoal_.header.frame_id = "world";

  // Save previous dyaw:
  dyaw_filtered_ = quadGoal_.dyaw;

  quadGoal_.vel = vectorNull();
  quadGoal_.accel = vectorNull();
  quadGoal_.jerk = vectorNull();
  quadGoal_.dyaw = 0;

  if (quadGoal_.cut_power && (flight_mode_.mode == flight_mode_.TAKEOFF || flight_mode_.mode == flight_mode_.GO))
  {
    double then = ros::Time::now().toSec();
    double diff = 0;
    while (diff < spinup_time_)
    {
      quadGoal_.header.stamp = ros::Time::now();
      diff = ros::Time::now().toSec() - then;
      quadGoal_.cut_power = 0;
      ros::Duration(0.01).sleep();
      pub_goal_.publish(quadGoal_);
    }
  }

  if (optimized_ && flight_mode_.mode != flight_mode_.NOT_FLYING && flight_mode_.mode != flight_mode_.KILL)
  {
    quadGoal_.cut_power = false;

    mtx_k.lock();

    k_ = std::min(k_, (int)(X_.rows() - 1));
    // printf("planner_status_= %d\n", planner_status_);
    // printf("to_land_= %d\n", to_land_);

    // printf("k_ = %d\n", k_);
    // printf("k_initial_cond_ = %d\n", k_initial_cond_);

    if (k_ > k_initial_cond_ && status_ == TRAVELING)
    {  // The initial condition of the optimization was already sent to the drone!
       // ROS_WARN("Optimization took too long. Increase deltaT");
    }

    if (((planner_status_ == REPLANNED && (k_ == k_initial_cond_ || to_land_ == true)) ||  // Should be k_==
         (force_reset_to_0_ && planner_status_ == REPLANNED)))  //&& takeoff_done_ == false)  // hacktodo
    {
      to_land_ == false;
      printf("************Reseteando a 0!\n");
      // reset the current optimizations (not needed because I already have a solution)
      sg_whole_.StopExecution();
      sg_safe_.StopExecution();

      force_reset_to_0_ = false;
      mtx_X_U_temp.lock();
      mtx_X_U.lock();
      X_ = X_temp_;
      U_ = U_temp_;
      mtx_X_U.unlock();
      mtx_X_U_temp.unlock();
      X_initialized_ = true;
      k_ = 0;  // Start again publishing the waypoints in X_ from the first row
      mtx_planner_status_.lock();
      planner_status_ = START_REPLANNING;
      mtx_planner_status_.unlock();
      // printf("pucCB2: planner_status_=START_REPLANNING\n");
    }

    if ((planner_status_ == REPLANNED && (k_ > k_initial_cond_)))
    {  // I've published what I planned --> plan again
      std::cout << bold << magenta << "Rejecting current plan, planning again. Suggestion: Increase delta_t" << reset
                << std::endl;
      sg_whole_.StopExecution();
      sg_safe_.StopExecution();
      mtx_planner_status_.lock();
      planner_status_ = START_REPLANNING;
      status_ = TRAVELING;
      mtx_planner_status_.unlock();
    }

    k_ = std::min(k_, (int)(X_.rows() - 1));
    /*    printf("k_=%d\n", k_);
        printf("X_.rows() - 1=%d\n", (int)(X_.rows() - 1));
        std::cout << "PubCB: Esto es lo que tengo por delante, voy a publicar la 1a fila" << std::endl;
        std::cout << X_.block(k_, 0, 10, 1) << std::endl;*/

    mtx_k.unlock();
    // int kp1 = std::min(k_ + deltaT, (int)(X_.rows() - 1));  // k plus offset

    quadGoal_.pos = eigen2rosvector(getPos(k_));
    quadGoal_.vel = eigen2rosvector(getVel(k_));
    quadGoal_.accel = (par_.use_ff) ? eigen2rosvector(getAccel(k_)) : vectorNull();
    quadGoal_.jerk = (par_.use_ff) ? eigen2rosvector(getJerk(k_)) : vectorNull();
    quadGoal_.dyaw = 0;

    // heading_ = atan2(goal_(1) - X_(0, 1), goal_(0) - X_(0, 0));

    // std::cout << "status_= " << status_ << std::endl;

    if (status_ == YAWING)
    {
      // mtx_G.lock();
      double desired_yaw = atan2(G_[1] - quadGoal_.pos.y, G_[0] - quadGoal_.pos.x);
      // mtx_G.unlock();
      // std::cout << red << bold << std::setprecision(6) << "desired_yaw=" << desired_yaw << reset << std::endl;
      // std::cout << red << bold << std::setprecision(6) << "quadGoal_.yaw=" << quadGoal_.yaw << reset << std::endl;

      double diff = desired_yaw - quadGoal_.yaw;
      // std::cout << red << bold << std::setprecision(6) << "diff before wrappping=" << diff << reset << std::endl;

      angle_wrap(diff);

      // std::cout << red << bold << "diff after wrappping=" << diff << reset << std::endl;

      yaw(diff, quadGoal_);
      /*      printf("Inside, desired_yaw=%0.2f,quadGoal_.yaw=%0.2f, diff=%f , abs(diff)=%f\n", desired_yaw,
         quadGoal_.yaw, diff, fabs(diff));*/
      if (fabs(diff) < 0.04)
      {
        // printf("It's less than 0.2!!\n");
        status_ = TRAVELING;
      }
      else
      {
        // printf("Yawing\n");
      }
    }

    if ((status_ == TRAVELING || status_ == GOAL_SEEN))
    {
      // double desired_yaw = atan2(quadGoal_.vel.y, quadGoal_.vel.x);
      desired_yaw_B_ = atan2(B_[1] - quadGoal_.pos.y, B_[0] - quadGoal_.pos.x);

      /*      std::cout << red << bold << std::setprecision(6) << "B_=" << B_.transpose() << reset << std::endl;
            std::cout << red << bold << std::setprecision(6) << "quadGoal_=" << quadGoal_.pos.x << "  " <<
         quadGoal_.pos.y
                      << reset << std::endl;
            std::cout << red << bold << std::setprecision(6) << "desired_yaw=" << desired_yaw << reset << std::endl;
            std::cout << red << bold << std::setprecision(6) << "quadGoal_.yaw=" << quadGoal_.yaw << reset <<
         std::endl;*/

      double diff = desired_yaw_B_ - quadGoal_.yaw;
      // std::cout << red << bold << std::setprecision(6) << "diff before wrappping=" << diff << reset << std::endl;
      angle_wrap(diff);
      // std::cout << red << bold << std::setprecision(6) << "diff after wrappping=" << diff << reset << std::endl;
      if (JPSk_solved_ == true and takeoff_done_ == true and fabs(diff) > 0.04)  // only yaw if diff is big enough
      {
        yaw(diff, quadGoal_);
      }

      if (JPSk_solved_ == false)
      {
        quadGoal_.dyaw = 0;
      }
    }
    if (status_ == GOAL_REACHED || takeoff_done_ == false)
    {
      quadGoal_.dyaw = 0;
      quadGoal_.yaw = quadGoal_.yaw;
    }

    mtx_k.lock();
    k_++;

    mtx_k.unlock();
  }
  else
  {
    quadGoal_.cut_power = true;
  }

  /*  ROS_INFO("publishing quadGoal: %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", quadGoal_.pos.x, quadGoal_.pos.y,
             quadGoal_.pos.z, quadGoal_.vel.x, quadGoal_.vel.y, quadGoal_.vel.z);*/

  // std::cout << green << bold << std::setprecision(6) << "quadGoal_.yaw sent=" << quadGoal_.yaw << reset << std::endl;
  // std::cout << green << bold << std::setprecision(6) << "quadGoal_.dyaw sent=" << quadGoal_.dyaw << reset <<
  // std::endl;

  ////////////////////////////////
  // NOW generate all the things needed for the jackal

  geometry_msgs::Twist cmd_jackal;

  /*  double x_error = sqrt(pow(xd, 2) + pow(yd, 2));
    double y_error = state_.pos.y - quadGoal_.pos.y;

    double vx_input = quadGoal_.vel.x - 2 * x_error;
    double vy_input = quadGoal_.vel.y - 2 * y_error;*/

  // std::cout << "k_=" << k_ << std::endl;
  // std::cout << "(int)(X_.rows() - 1)=" << (int)(X_.rows() - 1) << std::endl;

  if (status_ == YAWING)
  {
    cmd_jackal.angular.z = quadGoal_.dyaw;
  }

  else if (status_ == GOAL_REACHED)
  {
    // don't send commands
  }
  else if (k_ >= (int)(X_.rows() - 1) && status_ != GOAL_SEEN)  // stopped at the end of a trajectory, but GOAL not
                                                                // REACHED --> yaw
  {
    double angle = current_yaw_ - desired_yaw_B_;  // quadGoal_.yaw;
    // std::cout << bold << blue << "Spinning:" << reset << std::endl;
    // std::cout << "current_yaw_=" << current_yaw_ << std::endl;
    // std::cout << "quadGoal_.yaw=" << quadGoal_.yaw << std::endl;
    // std::cout << "angle before=" << angle << std::endl;
    angle_wrap(angle);
    // std::cout << "angle after=" << angle << std::endl;
    int direction = (angle > 0) ? -1 : 1;
    // std::cout << "direction=" << direction << std::endl;
    cmd_jackal.angular.z = direction * par_.w_max;
  }
  else
  {
    double x = quadGoal_.pos.x;
    double y = quadGoal_.pos.y;
    double xd = quadGoal_.vel.x;
    double yd = quadGoal_.vel.y;
    double xd2 = quadGoal_.accel.x;
    double yd2 = quadGoal_.accel.y;

    double v_desired = sqrt(pow(xd, 2) + pow(yd, 2));
    double alpha = current_yaw_ - atan2(y - state_.pos.y(), x - state_.pos.x());
    angle_wrap(alpha);                                                    // wrap between -pi and pi
    int forward = (alpha <= 3.14 / 2.0 && alpha > -3.14 / 2.0) ? 1 : -1;  // 1 if forward, -1 if backwards
    double dist_error = forward * sqrt(pow(x - state_.pos.x(), 2) + pow(y - state_.pos.y(), 2));
    alpha = (fabs(dist_error) < 0.1) ? 0 : alpha;

    /*  std::cout << bold << "v_desired= " << v_desired << ", dist_error= " << dist_error << ", forward= " << forward
                << "alpha= " << alpha << reset << std::endl;*/

    // See http://mathworld.wolfram.com/Curvature.html (diff(phi)/diff(t))
    double numerator = xd * yd2 - yd * xd2;
    double denominator = xd * xd + yd * yd;
    double w_desired = (denominator > 0.01) ? numerator / denominator : 0;
    double desired_yaw = (fabs(xd) < 0.001 || fabs(dist_error) < 0.03) ? desired_yaw_old_ : atan2(yd, xd);
    // std::cout << "desired_yaw=" << desired_yaw << std::endl;
    // std::cout << "current_yaw_=" << desired_yaw << std::endl;
    desired_yaw_old_ = desired_yaw;
    double yaw_error = current_yaw_ - desired_yaw;
    angle_wrap(yaw_error);  // wrap between -pi and pi
    /*  std::cout << bold << "current_yaw_= " << current_yaw_ << ", desired_yaw= " << desired_yaw << reset << std::endl;
      std::cout << bold << "w_desired= " << w_desired << ", yaw_error= " << yaw_error << reset << std::endl;*/

    double alpha_dot = (alpha - alpha_before_) / par_.dc;
    alpha_before_ = alpha;

    if (fabs(dist_error) > 0.15)
    {
      cmd_jackal.linear.x = par_.kdist * dist_error;
      /*      if (forward < 0)
            {
              if (alpha >= 0)
              {
                alpha = alpha - 3.14;
              }
              else
              {  // alpha is negative
                alpha = alpha + 3.14;
              }
              angle_wrap(alpha);
            }*/
      cmd_jackal.angular.z = -par_.kalpha * alpha;

      /*      std::cout << bold << red << "In Mode outside Trajectory" << reset << std::endl;
            std::cout << "alpha=" << alpha << std::endl;
            std::cout << "dist_error" << dist_error << std::endl;*/
    }
    else
    {
      cmd_jackal.linear.x = par_.kv * v_desired;
      cmd_jackal.angular.z = par_.kw * w_desired - par_.kyaw * yaw_error;
      /*      std::cout << bold << red << "In Mode inside Trajectory" << reset << std::endl;
            std::cout << "desired_yaw=" << desired_yaw << std::endl;
            std::cout << "current_yaw_=" << desired_yaw << std::endl;
            std::cout << "yaw_error=" << yaw_error << std::endl;*/
    }

    /*    std::cout << bold << "Linear.x= " << cmd_jackal.linear.x << blue << "-->v_desired=" << v_desired
                  << ", dist_error= " << dist_error << reset << std::endl;

        std::cout << bold << "Angular.z= " << cmd_jackal.angular.z << blue << "-->w_desired=" << w_desired
                  << ", yaw_error= " << yaw_error << ", alpha= " << alpha << reset << std::endl;*/
  }
  // std::cout << "Publishing Jackal Goal" << std::endl;
  pub_goal_jackal_.publish(cmd_jackal);
  ///////////////////////////////////////////////

  // std::cout << "Publishing QUAD Goal" << std::endl;
  pub_goal_.publish(quadGoal_);
  // std::cout << "QUAD Goal published" << std::endl;

  // Pub setpoint maker.  setpoint_ is the last quadGoal sent to the drone
  setpoint_.header.stamp = ros::Time::now();
  setpoint_.pose.position.x = quadGoal_.pos.x;
  setpoint_.pose.position.y = quadGoal_.pos.y;
  setpoint_.pose.position.z = quadGoal_.pos.z;
  // printf("Publicando Goal=%f, %f, %f\n", quadGoal_.pos.x, quadGoal_.pos.y, quadGoal_.pos.z);
  pub_setpoint_.publish(setpoint_);
  // printf("End pubCB\n");
  // printf("#########Time in pubCB %0.2f ms\n", 1000 * (ros::Time::now().toSec() - t0pubCB));
  mtx_goals.unlock();
}

void CVX::modeCB(const acl_msgs::QuadFlightMode& msg)
{
  printf("*****In modeCB\n");
  if (msg.mode == msg.LAND)  //&& flight_mode_.mode != flight_mode_.LAND
  {
    printf("LANDING\n");

    mtx_goals.lock();
    mtx_state.lock();

    state x0 = state_;
    state xf = state_;
    xf.pos[2] = par_.z_land;

    mtx_state.unlock();
    mtx_goals.unlock();

    sg_whole_.setXf(xf);
    sg_whole_.setX0(x0);
    std::vector<LinearConstraint3D> l_constraints_empty;
    sg_whole_.setPolytopes(l_constraints_empty);
    bool solved_landing = false;
    solved_landing = sg_whole_.genNewTraj();

    if (solved_landing == false)
    {
      std::cout << bold << red << "No solution for landing" << reset << std ::endl;
    }
    else
    {
      std::cout << "solution found" << std::endl;
      sg_whole_.fillXandU();
      to_land_ = true;
      mtx_X_U_temp.lock();
      U_temp_ = sg_whole_.U_temp_;
      X_temp_ = sg_whole_.X_temp_;
      mtx_X_U_temp.unlock();
      mtx_planner_status_.lock();
      planner_status_ = REPLANNED;
      mtx_planner_status_.unlock();
    }
  }
  flight_mode_.mode = msg.mode;
}

// Odometry Callback (for the Jackal)
void CVX::odomCB(const nav_msgs::Odometry& msg)
{
  // ROS_ERROR("In state CB");
  // printf("(State): %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", msg.pos.x, msg.pos.y, msg.pos.z, msg.vel.x, msg.vel.y,
  //       msg.vel.z);

  mtx_state.lock();

  state state_;
  state_.setPos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
  state_.setPos(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
  state_.setAccel(0.0, 0.0, 0.0);

  double roll, pitch, yaw;
  quaternion2Euler(msg.pose.pose.orientation, roll, pitch, yaw);

  if (state_initialized_ == false)
  {
    quadGoal_.pos.x = msg.pose.pose.position.x;
    quadGoal_.pos.y = msg.pose.pose.position.y;
    quadGoal_.pos.z = msg.pose.pose.position.z;

    quadGoal_.vel.x = msg.twist.twist.linear.x;
    quadGoal_.vel.y = msg.twist.twist.linear.y;
    quadGoal_.vel.z = msg.twist.twist.linear.z;

    quadGoal_.yaw = yaw;
  }

  state_initialized_ = true;
  /*  printf("(State): %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", msg.pos.x, msg.pos.y, msg.pos.z, msg.vel.x,
     msg.vel.y, msg.vel.z);*/

  /*  std::cout << bold << red << "IN ODOM CB:" << msg.pose.pose.orientation << reset << std::endl;
    std::cout << bold << red << "Yaw=" << yaw * 180 / 3.14 << reset << std::endl;*/
  current_yaw_ = yaw;

  mtx_state.unlock();
  // Stop updating when we get GO
  if (flight_mode_.mode == flight_mode_.NOT_FLYING || flight_mode_.mode == flight_mode_.KILL)
  {
    quadGoal_.pos.x = msg.pose.pose.position.x;
    quadGoal_.pos.y = msg.pose.pose.position.y;
    quadGoal_.pos.z = msg.pose.pose.position.z;

    quadGoal_.vel.x = msg.twist.twist.linear.x;
    quadGoal_.vel.y = msg.twist.twist.linear.y;
    quadGoal_.vel.z = msg.twist.twist.linear.z;

    double roll, pitch, yaw;
    quaternion2Euler(msg.pose.pose.orientation, roll, pitch, yaw);
    current_yaw_ = yaw;
    quadGoal_.yaw = yaw;
    z_start_ = msg.pose.pose.position.z;
    z_start_ = std::max(0.0, z_start_);
    mtx_initial_cond.lock();
    stateA_.setPos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    mtx_initial_cond.unlock();
  }

  static int i = 0;
  i++;

  if (status_ != GOAL_REACHED && par_.visual == true)
  {
    pubActualTraj();
  }

  if (i % 10 == 0 && status_ != GOAL_REACHED && i != 0)
  {
    Eigen::Vector3d actual_pos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    // Don't use the state to compute the total distance (it's very noisy)
    // log_.total_dist = log_.total_dist + (actual_pos - pos_old_).norm();
    // pos_old_ = actual_pos;
  }
  Eigen::Vector3d vel(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
  log_.veloc_norm = vel.norm();
}

void CVX::stateCB(const acl_msgs::State& msg)
{
  // ROS_ERROR("In state CB");
  // printf("(State): %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", msg.pos.x, msg.pos.y, msg.pos.z, msg.vel.x, msg.vel.y,
  //       msg.vel.z);

  double roll, pitch, yaw;
  quaternion2Euler(msg.quat, roll, pitch, yaw);
  current_yaw_ = yaw;

  if (state_initialized_ == false)
  {
    quadGoal_.pos = msg.pos;
    quadGoal_.vel = msg.vel;
    quadGoal_.yaw = yaw;
  }

  mtx_state.lock();
  state state_;
  state_.setPos(msg.pos.x, msg.pos.y, msg.pos.z);
  state_.setVel(msg.vel.x, msg.vel.y, msg.vel.z);
  state_.setAccel(0.0, 0.0, 0.0);

  state_initialized_ = true;
  /*  printf("(State): %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", msg.pos.x, msg.pos.y, msg.pos.z, msg.vel.x,
     msg.vel.y, msg.vel.z);*/

  mtx_state.unlock();

  // Stop updating when we get GO
  if (flight_mode_.mode == flight_mode_.NOT_FLYING || flight_mode_.mode == flight_mode_.KILL)
  {
    quadGoal_.pos = msg.pos;
    quadGoal_.vel = msg.vel;

    double roll, pitch, yaw;
    quaternion2Euler(msg.quat, roll, pitch, yaw);
    quadGoal_.yaw = yaw;
    z_start_ = msg.pos.z;
    z_start_ = std::max(0.0, z_start_);
    mtx_initial_cond.lock();
    stateA_.pos = state_.pos;
    mtx_initial_cond.unlock();
  }

  static int i = 0;
  i++;

  if (status_ != GOAL_REACHED && par_.visual == true)
  {
    pubActualTraj();
  }

  if (i % 10 == 0 && status_ != GOAL_REACHED && i != 0)
  {
    Eigen::Vector3d actual_pos(msg.pos.x, msg.pos.y, msg.pos.z);
    // Don't use the state to compute the total distance (it's very noisy)
    // log_.total_dist = log_.total_dist + (actual_pos - pos_old_).norm();
    // pos_old_ = actual_pos;
  }
  Eigen::Vector3d vel(msg.vel.x, msg.vel.y, msg.vel.z);
  log_.veloc_norm = vel.norm();
}

void CVX::updateInitialCond(int i)
{
  mtx_initial_cond.lock();
  if (status_ != GOAL_REACHED)
  {
    stateA_.pos = getPos(i);
    stateA_.vel = getVel(i);
    stateA_.accel = (par_.use_ff) ? getAccel(i) : Eigen::Vector3d::Zero();
    // stateA_.jerk = (par_.use_ff) ? getJerk(i) : vectorNull();
  }
  else
  {
    mtx_state.unlock();
    stateA_.pos = state_.pos;
    stateA_.vel = Eigen::Vector3d::Zero();
    stateA_.accel = Eigen::Vector3d::Zero();
    // stateA_.jerk = vectorNull();
  }
  mtx_initial_cond.unlock();
}

Eigen::Vector3d CVX::getPos(int i)
{
  Eigen::Vector3d tmp;
  mtx_X_U.lock();
  tmp << X_(i, 0), X_(i, 1), X_(i, 2);
  mtx_X_U.unlock();
  return tmp;
}

Eigen::Vector3d CVX::getVel(int i)
{
  Eigen::Vector3d tmp;
  mtx_X_U.lock();
  tmp << X_(i, 3), X_(i, 4), X_(i, 5);
  mtx_X_U.unlock();
  return tmp;
}

Eigen::Vector3d CVX::getAccel(int i)
{
  Eigen::Vector3d tmp;
  mtx_X_U.lock();
  tmp << X_(i, 6), X_(i, 7), X_(i, 8);
  mtx_X_U.unlock();
  return tmp;
}
Eigen::Vector3d CVX::getJerk(int i)
{
  Eigen::Vector3d tmp;
  mtx_X_U.lock();
  tmp << U_(i, 0), U_(i, 1), U_(i, 2);
  mtx_X_U.unlock();
  return tmp;
}

void CVX::pubTraj(Eigen::MatrixXd& X, int type)
{
  // Trajectory
  nav_msgs::Path traj;
  traj.poses.clear();
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = "world";

  geometry_msgs::PoseStamped temp_path;

  for (int i = 0; i < X.rows(); i = i + 8)
  {
    temp_path.pose.position.x = X(i, 0);
    temp_path.pose.position.y = X(i, 1);
    temp_path.pose.position.z = X(i, 2);
    temp_path.pose.orientation.w = 1;
    temp_path.pose.orientation.x = 0;
    temp_path.pose.orientation.y = 0;
    temp_path.pose.orientation.z = 0;
    traj.poses.push_back(temp_path);
  }

  if (type == WHOLE)
  {
    pub_traj_whole_.publish(traj);
  }

  if (type == SAFE)
  {
    pub_traj_safe_.publish(traj);
  }

  clearMarkerColoredTraj();
  clearMarkerArray(&traj_committed_colored_, &pub_traj_committed_colored_);
  clearMarkerArray(&traj_whole_colored_, &pub_traj_whole_colored_);
  clearMarkerArray(&traj_safe_colored_, &pub_traj_safe_colored_);

  if (type == COMMITTED_COLORED)
  {
    traj_committed_colored_ = Matrix2ColoredMarkerArray(X, type, par_.v_max);
    pub_traj_committed_colored_.publish(traj_committed_colored_);
  }

  if (type == WHOLE_COLORED)
  {
    traj_whole_colored_ = Matrix2ColoredMarkerArray(X, type, par_.v_max);
    pub_traj_whole_colored_.publish(traj_whole_colored_);
  }

  if (type == SAFE_COLORED)
  {
    traj_safe_colored_ = Matrix2ColoredMarkerArray(X, type, par_.v_max);
    pub_traj_safe_colored_.publish(traj_safe_colored_);
  }
}

void CVX::clearMarkerActualTraj()
{
  // printf("In clearMarkerActualTraj\n");

  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::DELETEALL;
  m.id = 0;
  m.scale.x = 0.02;
  m.scale.y = 0.04;
  m.scale.z = 1;
  pub_actual_traj_.publish(m);
  actual_trajID_ = 0;
}

void CVX::clearMarkerColoredTraj()
{
  // printf("In clearMarkerActualTraj\n");

  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::DELETEALL;
  m.id = 0;
  m.scale.x = 0.02;
  m.scale.y = 0.04;
  m.scale.z = 1;
  pub_actual_traj_.publish(m);
  // actual_trajID_ = 0;
}

// Occupied CB
void CVX::mapCB(const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_map_ros,
                const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_unk_ros)
{
  MyTimer mapCB_t(true);
  mtx_map.lock();
  mtx_unk.lock();

  // Occupied Space Point Cloud
  pcl::fromROSMsg(*pcl2ptr_map_ros, *pclptr_map_);

  jps_manager_.updateJPSMap(pclptr_map_, state_.pos);  // Update even where there are no points

  if (pclptr_map_->width != 0 && pclptr_map_->height != 0)  // Point Cloud is not empty
  {
    kdtree_map_.setInputCloud(pclptr_map_);
    kdtree_map_initialized_ = 1;

    jps_manager_.vec_o_ = pclptr_to_vec(pclptr_map_);
  }
  else
  {
    ROS_WARN("Occupancy Grid received is empty, maybe map is too small?\n");
  }

  // Unknown Space Point Cloud
  pcl::fromROSMsg(*pcl2ptr_unk_ros, *pclptr_unk_);
  if (pcl2ptr_unk_ros->width != 0 && pcl2ptr_unk_ros->height != 0)
  {
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*pclptr_unk_, *pclptr_unk_, index);
    if (pclptr_unk_->points.size() == 0)
    {
      printf("Unkown cloud has 0 points\n");
      return;
    }

    kdtree_unk_.setInputCloud(pclptr_unk_);
    kdtree_unk_initialized_ = 1;
    jps_manager_.vec_uo_ = pclptr_to_vec(pclptr_unk_);  // insert unknown space
    jps_manager_.vec_uo_.insert(jps_manager_.vec_uo_.end(), jps_manager_.vec_o_.begin(),
                                jps_manager_.vec_o_.end());  // append known space
  }
  mtx_map.unlock();
  mtx_unk.unlock();
  // std::cout << bold << blue << "MapCB:  " << mapCB_t << "ms" << reset << std::endl;
}

// Returns the first collision of JPS with the map (i.e. with the known obstacles). Note that JPS will collide with a
// map B if JPS was computed using an older map A
// If type_return==Intersection, it returns the last point in the JPS path that is at least par_.inflation_jps from map
Eigen::Vector3d CVX::getFirstCollisionJPS(vec_Vecf<3>& path, bool* thereIsIntersection, int map, int type_return)
{
  vec_Vecf<3> original = path;

  Eigen::Vector3d first_element = path[0];
  Eigen::Vector3d last_search_point = path[0];
  Eigen::Vector3d inters = path[0];
  pcl::PointXYZ pcl_search_point = eigenPoint2pclPoint(path[0]);

  Eigen::Vector3d result;

  // occupied (map)
  int n = 1;
  std::vector<int> id_map(n);
  std::vector<float> dist2_map(n);  // squared distance
  double r = 1000000;
  // printElementsOfJPS(path);
  // printf("In 2\n");

  mtx_map.lock();
  mtx_unk.lock();

  // Find the next eig_search_point
  int last_id = -1;  // this is the last index inside the sphere
  int iteration = 0;
  while (path.size() > 0)
  {
    // std::cout<<red<<"New Iteration, iteration="<<iteration<<reset<<std::endl;
    // std::cout << red << "Searching from point=" << path[0].transpose() << reset << std::endl;
    pcl_search_point = eigenPoint2pclPoint(path[0]);

    int number_of_neigh;

    if (map == MAP)
    {
      number_of_neigh = kdtree_map_.nearestKSearch(pcl_search_point, n, id_map, dist2_map);
    }
    else  // map == UNKNOWN_MAP
    {
      number_of_neigh = kdtree_unk_.nearestKSearch(pcl_search_point, n, id_map, dist2_map);
      // std::cout << "In unknown_map, number of neig=" << number_of_neigh << std::endl;
    }
    // printf("************NearestSearch: TotalTime= %0.2f ms\n", 1000 * (ros::Time::now().toSec() - before));

    if (number_of_neigh > 0)
    {
      r = sqrt(dist2_map[0]);

      // std::cout << "r=" << r << std::endl;
      // std::cout << "Point=" << r << std::endl;

      if (r < par_.drone_radius)  // collision of the JPS path and an inflated obstacle --> take last search point
      {
        // std::cout << "Collision detected" << std::endl;  // We will return the search_point
        // pubJPSIntersection(inters);
        // inters = path[0];  // path[0] is the search_point I'm using.
        if (iteration == 0)
        {
          std::cout << red << bold << "The first point is in collision --> Hacking" << reset << std::endl;
        }
        switch (type_return)
        {
          case RETURN_LAST_VERTEX:
            result = last_search_point;
            break;
          case RETURN_INTERSECTION:
            if (iteration == 0)
            {  // Hacking (TODO)
              Eigen::Vector3d tmp;
              tmp << original[0](0) + 0.01, original[0](1), original[0](2);
              path.clear();
              path.push_back(original[0]);
              path.push_back(tmp);
              result = path[path.size() - 1];
              // result=original[original.size() - 1];
            }
            else
            {
              // std::cout << "In Return Intersection, last_id=" << last_id<<el_eliminated<< std::endl;
              int vertexes_eliminated_tmp = original.size() - path.size() + 1;
              // std::cout << "In Return Intersection, vertexes_eliminated_tmp=" << vertexes_eliminated_tmp <<
              // std::endl;
              original.erase(original.begin() + vertexes_eliminated_tmp,
                             original.end());  // Now original contains all the elements eliminated
              original.push_back(path[0]);

              /*              std::cout << "Result before reduceJPSbyDistance" << original[original.size() -
                 1].transpose()
                                      << std::endl;*/

              // This is to force the intersection point to be at least par_.drone_radius away from the obstacles
              reduceJPSbyDistance(original, par_.drone_radius);

              result = original[original.size() - 1];

              // std::cout<<"Result here is"<<result.transpose()<<std::endl;

              path = original;
            }
            // Copy the resulting path to the reference
            /*     std::reverse(original.begin(), original.end());  // flip all the vector
               result = getFirstIntersectionWithSphere(original, par_.inflation_jps, original[0]);*/
            break;
        }

        *thereIsIntersection = true;

        break;  // Leave the while loop
      }

      bool no_points_outside_sphere = false;

      inters = getFirstIntersectionWithSphere(path, r, path[0], &last_id, &no_points_outside_sphere);
      // printf("**********Found it*****************\n");
      if (no_points_outside_sphere == true)
      {  // JPS doesn't intersect with any obstacle
        *thereIsIntersection = false;
        /*        std::cout << "JPS provided doesn't intersect any obstacles, returning the first element of the path
           you gave " "me\n"
                          << std::endl;*/
        result = first_element;

        if (type_return == RETURN_INTERSECTION)
        {
          result = original[original.size() - 1];
          path = original;
        }

        break;  // Leave the while loop
      }
      // printf("In 4\n");

      last_search_point = path[0];
      // Remove all the points of the path whose id is <= to last_id:
      path.erase(path.begin(), path.begin() + last_id + 1);

      // and add the intersection as the first point of the path
      path.insert(path.begin(), inters);
    }
    else
    {  // There is no neighbours
      *thereIsIntersection = false;
      ROS_INFO("JPS provided doesn't intersect any obstacles, returning the first element of the path you gave me\n");
      result = first_element;

      if (type_return == RETURN_INTERSECTION)
      {
        result = original[original.size() - 1];
        path = original;
      }

      break;
    }
    iteration = iteration + 1;
  }
  mtx_map.unlock();
  mtx_unk.unlock();

  return result;
}

void CVX::pubJPSIntersection(Eigen::Vector3d& inters)
{
  geometry_msgs::PointStamped p;
  p.header.frame_id = "world";
  p.point = eigen2point(inters);
  pub_jps_inters_.publish(p);
}

void CVX::pubActualTraj()
{
  static geometry_msgs::Point p_last = pointOrigin();
  mtx_state.lock();
  Eigen::Vector3d act_pos = state_.pos;
  mtx_state.unlock();
  // mtx_G.lock();
  Eigen::Vector3d t_goal = G_;
  // mtx_G.unlock();
  float dist_to_goal = (t_goal - act_pos).norm();

  if (dist_to_goal < 2 * par_.goal_radius)
  {
    return;
  }

  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::ADD;
  m.id = actual_trajID_ % 3000;  // Start the id again after 300 points published (if not RVIZ goes very slow)
  actual_trajID_++;
  m.color = color(RED);
  m.scale.x = 0.15;
  m.scale.y = 0;
  m.scale.z = 0;
  m.header.stamp = ros::Time::now();
  m.header.frame_id = "world";

  geometry_msgs::Point p;
  p = eigen2point(act_pos);
  m.points.push_back(p_last);
  m.points.push_back(p);
  pub_actual_traj_.publish(m);
  p_last = p;
}

void CVX::pubTerminalGoal()
{
  geometry_msgs::PointStamped p;
  p.header.frame_id = "world";
  // mtx_G.lock();
  p.point = eigen2point(G_);
  // mtx_G.unlock();
  pub_point_G_.publish(p);
}
