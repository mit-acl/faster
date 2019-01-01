// Authors: Jesus Tordesillas
// Date: August 2018, December 2018

// TODO (from December 2018)
// Put several iterations to find dt (increasing dt in each iteration)
// right now the unkown space that cvx receives is only a sphere around the drone, not the whole real unknown
// space
// Set dt as a constant in the optimization problem (so that many constraints no se tengan que poner de nuevo) (what in
// Cvxgen is called parameter)
// Mirar a ver d'onde acaba el goal, parece que est'a acabando 0.2m del real goal.
// Al hacer shrink de los polyhedra, creo que se puede romper la continuidad (y que 2 polyhedra succesive no overlap
// between them)
// Al hacer shrink de los polyhedra, a veces la posición actual del dron queda fuera de los poliedros --> gurobi no
// encuentra solución

// TODOs antiguos:
// TODO: compile cvxgen with the option -03 (see
// https://stackoverflow.com/questions/19689014/gcc-difference-between-o3-and-os
// and
// https://cvxgen.com/docs/c_interface.html    )

// TODO: update gcc to the latest version (see https://cvxgen.com/docs/c_interface.html)

// TODO: use the gpu versions of the pcl functions
// TODO: https://eigen.tuxfamily.org/dox/TopicCUDA.html

// TODO: Check the offset or offset-1

#include "cvx.hpp"
#include "geometry_msgs/PointStamped.h"
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

#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>

using namespace JPS;

CVX::CVX(ros::NodeHandle nh, ros::NodeHandle nh_replan_CB, ros::NodeHandle nh_pub_CB)
  : nh_(nh), nh_replan_CB_(nh_replan_CB), nh_pub_CB_(nh_pub_CB)
{
  printf("Doing the setup\n");
  ros::param::param<bool>("~use_ff", par_.use_ff, 1);
  ros::param::param<bool>("~visual", par_.visual, true);
  ros::param::param<bool>("~use_vel", par_.use_vel, true);

  ros::param::param<double>("~wdx", par_.wdx, 20.0);
  ros::param::param<double>("~wdy", par_.wdy, 20.0);
  ros::param::param<double>("~wdz", par_.wdz, 4.0);
  ros::param::param<double>("~res", par_.res, 0.15);

  ros::param::param<double>("~dc", par_.dc, 0.01);
  ros::param::param<double>("~goal_radius", par_.goal_radius, 0.2);
  ros::param::param<double>("~drone_radius", par_.drone_radius, 0.15);

  ros::param::param<int>("~N", par_.N, 10);

  ros::param::param<int>("~offset", par_.offset, 5);

  ros::param::param<double>("~Ra", par_.Ra, 2.0);
  ros::param::param<double>("~Ra_max", par_.Ra_max, 2.5);
  ros::param::param<double>("~Rb", par_.Rb, 6.0);
  ros::param::param<double>("~w_max", par_.w_max, 1.0);
  ros::param::param<double>("~alpha_0", par_.alpha_0, 1.0);
  ros::param::param<double>("~z_ground", par_.z_ground, 0.0);
  ros::param::param<double>("~z_max", par_.z_max, 5.0);
  ros::param::param<double>("~inflation_jps", par_.inflation_jps, 0.8);
  ros::param::param<double>("~factor_jps", par_.factor_jps, 2);

  ros::param::param<double>("~v_max", par_.v_max, 2.0);
  ros::param::param<double>("~a_max", par_.a_max, 2.0);
  ros::param::param<double>("~j_max", par_.j_max, 10.0);
  ros::param::param<double>("~q", par_.q, 100000.0);

  ros::param::param<double>("~z_land", par_.z_land, 0.02);

  ros::param::param<double>("cntrl/spinup_time", spinup_time_, 0.5);

  optimized_ = false;
  flight_mode_.mode = flight_mode_.NOT_FLYING;

  pub_goal_ = nh_.advertise<acl_msgs::QuadGoal>("goal", 1);
  pub_term_goal_ = nh_.advertise<geometry_msgs::PointStamped>("term_goal_projected", 1);
  pub_traj_ = nh_.advertise<nav_msgs::Path>("traj", 1);
  pub_traj_rescue_ = nh_.advertise<nav_msgs::Path>("traj_rescue", 1);

  pub_setpoint_ = nh_.advertise<visualization_msgs::Marker>("setpoint", 1);
  pub_trajs_sphere_ = nh_.advertise<visualization_msgs::MarkerArray>("trajs_sphere", 1);
  pub_forces_ = nh_.advertise<visualization_msgs::MarkerArray>("forces", 1);
  pub_actual_traj_ = nh_.advertise<visualization_msgs::Marker>("actual_traj", 1);
  pub_path_jps1_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps1", 1);
  pub_path_jps2_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps2", 1);

  cvx_decomp_el_pub_ = nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
  cvx_decomp_poly_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);

  pub_jps_inters_ = nh_.advertise<geometry_msgs::PointStamped>("jps_intersection", 1);

  pub_intersec_points_ = nh_.advertise<visualization_msgs::MarkerArray>("intersection_points", 1);

  pub_planning_vis_ = nh_.advertise<visualization_msgs::MarkerArray>("planning_vis", 1);

  pub_samples_rescue_path_ = nh_.advertise<visualization_msgs::MarkerArray>("samples_rescue_path", 1);

  pub_log_ = nh_.advertise<acl_msgs::Cvx>("log_topic", 1);

  sub_goal_ = nh_.subscribe("term_goal", 1, &CVX::goalCB, this);
  sub_mode_ = nh_.subscribe("flightmode", 1, &CVX::modeCB, this);
  sub_state_ = nh_.subscribe("state", 1, &CVX::stateCB, this);
  sub_map_ = nh_.subscribe("occup_grid", 1, &CVX::mapCB, this);
  sub_unk_ = nh_.subscribe("unknown_grid", 1, &CVX::unkCB, this);
  sub_frontier_ = nh_.subscribe("frontier_grid", 1, &CVX::frontierCB, this);
  sub_pcl_ = nh_.subscribe("pcloud", 1, &CVX::pclCB, this);

  pubCBTimer_ = nh_pub_CB_.createTimer(ros::Duration(par_.dc), &CVX::pubCB, this);

  replanCBTimer_ = nh_replan_CB.createTimer(ros::Duration(par_.dc), &CVX::replanCB, this);

  // Initialize setpoint marker
  setpoint_.header.frame_id = "world";
  setpoint_.id = 0;
  setpoint_.type = visualization_msgs::Marker::SPHERE;
  setpoint_.scale.x = 0.35;
  setpoint_.scale.y = 0.35;
  setpoint_.scale.z = 0.35;
  setpoint_.color = color(ORANGE_TRANS);
  // mtx_term_goal.lock();
  term_goal_ << 0, 0, 0;
  // mtx_term_goal.unlock();
  term_term_goal_ << 0, 0, 0;

  quadGoal_.pos = vectorNull();
  quadGoal_.vel = vectorNull();
  quadGoal_.accel = vectorNull();
  quadGoal_.jerk = vectorNull();

  mtx_initial_cond.lock();
  initialCond_.pos = vectorNull();
  initialCond_.vel = vectorNull();
  initialCond_.accel = vectorNull();
  initialCond_.jerk = vectorNull();
  mtx_initial_cond.unlock();

  log_.total_dist = 0;

  markerID_ = 0;

  cells_x_ = (int)par_.wdx / par_.res;
  cells_y_ = (int)par_.wdy / par_.res;
  cells_z_ = (int)par_.wdz / par_.res;

  solver_vel_.setDC(par_.dc);
  solver_accel_.setDC(par_.dc);
  solver_jerk_.setDC(par_.dc);

  solver_vel_.setq(par_.q);
  solver_accel_.setq(par_.q);
  solver_jerk_.setq(par_.q);

  double max_values[3] = { par_.v_max, par_.a_max, par_.j_max };
  solver_jerk_.set_max(max_values);

  solver_gurobi_.setN(par_.N);
  solver_gurobi_.createVars();
  solver_gurobi_.setDC(par_.dc);
  solver_gurobi_.set_max(max_values);
  solver_gurobi_.setQ(par_.q);

  double max_values_vel[1] = { par_.v_max };
  solver_vel_.set_max(max_values_vel);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
  // pclptr_map_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pclptr_unk_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  name_drone_ = ros::this_node::getNamespace();
  name_drone_.erase(0, 2);  // Erase slashes

  map_util_ = std::make_shared<VoxelMapUtil>();
  planner_ptr_ = std::unique_ptr<JPSPlanner3D>(new JPSPlanner3D(false));

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
  if (i == 1)
  {
    // printf("going to clear MarkerArray");
    clearMarkerArray(&path_jps1_, &pub_path_jps1_);
  }
  else
  {
    clearMarkerArray(&path_jps2_, &pub_path_jps2_);
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
  // path_jps_ = clearArrows();
  if (i == 1)
  {
    vectorOfVectors2MarkerArray(path, &path_jps1_, color(BLUE));
    pub_path_jps1_.publish(path_jps1_);
  }
  else
  {
    vectorOfVectors2MarkerArray(path, &path_jps2_, color(RED));
    pub_path_jps2_.publish(path_jps2_);
  }
}

void CVX::publishJPS2handIntersection(vec_Vecf<3> JPS2, vec_Vecf<3> JPS2_fix, Eigen::Vector3d inter1,
                                      Eigen::Vector3d inter2, bool solvedFix)
{
  // printf("Going to publish\n");
  /*vec_Vecf<3> traj, visualization_msgs::MarkerArray* m_array*/
  clearJPSPathVisualization(2);
  // path_jps_ = clearArrows();

  // vectorOfVectors2MarkerArray(JPS2, &path_jps2_, color(RED));
  if (solvedFix == true)
  {
    vectorOfVectors2MarkerArray(JPS2_fix, &path_jps2_, color(GREEN));
  }

  visualization_msgs::Marker m1;
  m1.header.frame_id = "world";
  m1.id = 19865165;
  m1.type = visualization_msgs::Marker::SPHERE;
  m1.scale = vectorUniform(0.3);
  m1.color = color(BLUE_TRANS);
  m1.pose.position = eigen2point(inter1);
  path_jps2_.markers.push_back(m1);

  visualization_msgs::Marker m2;
  m2.header.frame_id = "world";
  m2.id = 19865166;
  m2.type = visualization_msgs::Marker::SPHERE;
  m2.scale = vectorUniform(0.3);
  m2.color = color(RED_TRANS);
  m2.pose.position = eigen2point(inter2);
  path_jps2_.markers.push_back(m2);

  pub_path_jps2_.publish(path_jps2_);
}

void CVX::updateJPSMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr)
{
  mtx_state.lock();
  Vec3f center_map(state_.pos.x, state_.pos.y, state_.pos.z);  // center of the map
  mtx_state.unlock();
  Vec3i dim(cells_x_, cells_y_, cells_z_);  //  number of cells in each dimension
  // printf("Before reader\n");
  // printf("Sending dim=\n");
  std::cout << dim.transpose() << std::endl;
  MapReader<Vec3i, Vec3f> reader(pclptr, cells_x_, cells_y_, cells_z_, par_.factor_jps * par_.res, center_map,
                                 par_.z_ground, par_.z_max,
                                 par_.inflation_jps);  // Map read
  // std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();
  // printf("Before setMap\n");
  mtx_jps_map_util.lock();
  map_util_->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());
  // printf("After setMap\n");
  planner_ptr_->setMapUtil(map_util_);  // Set collision checking function
  mtx_jps_map_util.unlock();
  // printf("After setMapUtil\n");
}

vec_Vecf<3> CVX::solveJPS3D(Vec3f& start_sent, Vec3f& goal_sent, bool* solved, int i)
{
  Eigen::Vector3d start(start_sent[0], start_sent[1], start_sent[2]);
  Eigen::Vector3d goal(goal_sent[0], goal_sent[1], goal_sent[2]);
  /*  std::cout << "IN JPS3d" << std::endl;*/
  /*  std::cout << "start=" << start.transpose() << std::endl;
    std::cout << "goal=" << goal.transpose() << std::endl;*/

  Vec3f originalStart = start;
  if (flight_mode_.mode != flight_mode_.GO)
  {
    vec_Vecf<3> solution;
    solution.push_back(start);
    solution.push_back(goal);
    *solved = true;
    return solution;
  }

  pcl::PointXYZ pcl_start = eigenPoint2pclPoint(start);
  pcl::PointXYZ pcl_goal = eigenPoint2pclPoint(goal);

  mtx_map.lock();
  std::vector<int> id_map1(1);
  std::vector<float> dist2_map1(1);  // squared distance
  // printf("solveJPS3D3\n");

  if (kdtree_map_.nearestKSearch(pcl_start, 1, id_map1, dist2_map1) > 0)
  {
    double r = sqrt(dist2_map1[0]);
    // printf("nearest obstacle for start is at d=%f\n", r);

    Eigen::Vector3d n_obs;  // nearest obstacle
    pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr = kdtree_map_.getInputCloud();
    n_obs << ptr->points[id_map1[0]].x, ptr->points[id_map1[0]].y, ptr->points[id_map1[0]].z;

    Eigen::Array3d dist = ((start - n_obs).array()).abs();  // distances in each component to the nearest obstacle
    /*    std::cout << "n_obs" << n_obs.transpose() << std::endl;
        std::cout << "start" << start.transpose() << std::endl;*/

    // std::cout << "distances" << distances << std::endl;
    // printf("solveJPS3D3.5\n");
    if ((dist < par_.inflation_jps).all() == true)
    {
      /*      printf("The START is hitting an inflated obstacle or the ground, JPS won't work\n");
            printf("I'm going to try to fix it for you: Start= near point in the opposite direction\n");*/

      Eigen::Array3d signed_dists = start - n_obs;
      double signx = copysign(1, signed_dists[0]);
      double signy = copysign(1, signed_dists[1]);  // sign of signed_dists[1]
      double signz = copysign(1, signed_dists[2]);  // sign of signed_dists[2]

      /*      std::cout << "Obstaculo" << n_obs.transpose() << std::endl;
            std::cout << "Start antes" << start.transpose() << std::endl;
            std::cout << "Dist" << dist.transpose() << std::endl;*/

      start[0] =
          (dist[0] < par_.inflation_jps) ? start[0] + signx * (par_.inflation_jps + 2 * par_.res - dist[0]) : start[0];
      start[1] =
          (dist[1] < par_.inflation_jps) ? start[1] + signy * (par_.inflation_jps + 2 * par_.res - dist[1]) : start[1];
      start[2] =
          (dist[2] < par_.inflation_jps) ? start[2] + signz * (par_.inflation_jps + 2 * par_.res - dist[2]) : start[2];

      /*      std::cout << "Start despues" << start.transpose() << std::endl;

            std::cout << "otros valores" << std::endl;
            std::cout << "signx=" << signx << std::endl;
            std::cout << "signy=" << signy << std::endl;
            std::cout << "signz=" << signz << std::endl;

            std::cout << "1=" << start[0] + signx * (par_.inflation_jps + 2 * par_.res - dist[0]) << std::endl;
            std::cout << "2=" << signy * (par_.inflation_jps + 1 * par_.res - dist[1]) << std::endl;
            std::cout << "3=" << signz * (par_.inflation_jps + 1 * par_.res - dist[2]) << std::endl;
      */
      pcl::PointXYZ pcl_start2 = eigenPoint2pclPoint(start);
      std::vector<int> idxR;
      std::vector<float> r2D;  // squared distance
      Eigen::Vector3d force(0, 0, 0);
      pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr = kdtree_map_.getInputCloud();

      if (kdtree_map_.radiusSearch(pcl_start2, 1.5 * par_.inflation_jps, idxR, r2D) > 0)  // This is an
      {
        for (size_t i = 0; i < idxR.size(); ++i)
        {
          Eigen::Vector3d obs(ptr->points[idxR[i]].x, ptr->points[idxR[i]].y, ptr->points[idxR[i]].z);
          force = force + (start - obs).normalized();
        }
        start = start + (par_.inflation_jps + 2 * par_.res - sqrt(r2D[0])) * (force.normalized());
      }

      // std::cout << "force=" << force.transpose() << std::endl;

      if (start[2] < par_.z_ground)
      {
        start[2] = par_.z_ground + 2 * par_.res;
      }
    }
  }

  // printf("solveJPS3D4\n");
  std::vector<int> id_map2(1);
  std::vector<float> dist2_map2(1);  // squared distance
  if (kdtree_map_.nearestKSearch(pcl_goal, 1, id_map2, dist2_map2) > 0)
  {
    double r = sqrt(dist2_map2[0]);
    // printf("nearest obstacle for goal is at d=%f\n", r);
    Eigen::Vector3d n_obs;  // nearest obstacle
    pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr = kdtree_map_.getInputCloud();
    n_obs << ptr->points[id_map2[0]].x, ptr->points[id_map2[0]].y, ptr->points[id_map2[0]].z;

    Eigen::Array3d distances = ((n_obs - goal).array()).abs();  // distances in each component to the nearest obstacle

    if ((distances < par_.inflation_jps).all() == true)
    {
      /*      std::cout << "Goal=" << goal.transpose() << std::endl;
            std::cout << "n_obs=" << n_obs.transpose() << std::endl;
            std::cout << "distances=" << distances.transpose() << std::endl;

            printf("The GOAL is hitting an inflated obstacle, JPS won't work\n");
            printf("I'm going to try to fix it for you: GOAL= near point in the opposite direction\n");
            printf("If the new goal falls outside the map, it won't work neither\n");*/

      std::vector<int> idxR;
      std::vector<float> r2D;  // squared distance
      Eigen::Vector3d force(0, 0, 0);
      pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr = kdtree_map_.getInputCloud();

      if (kdtree_map_.radiusSearch(pcl_goal, 2 * par_.inflation_jps, idxR, r2D) > 0)  // This is an approximation
      {
        for (size_t i = 0; i < idxR.size(); ++i)
        {
          Eigen::Vector3d obs(ptr->points[idxR[i]].x, ptr->points[idxR[i]].y, ptr->points[idxR[i]].z);
          force = force + (start - obs).normalized();
        }
      }
      goal = goal + (par_.inflation_jps + 2 * par_.res) * (force.normalized());

      // Check if it falls outside the map
      mtx_state.lock();
      Eigen::Vector3d rel(goal[0] - state_.pos.x, goal[1] - state_.pos.y, goal[2] - state_.pos.z);
      goal[0] = (rel[0] > par_.wdx / 2 + 4 * par_.res) ? state_.pos.x + par_.wdx / 2 + 4 * par_.res : goal[0];
      goal[1] = (rel[1] > par_.wdy / 2 + 4 * par_.res) ? state_.pos.y + par_.wdy / 2 + 4 * par_.res : goal[1];
      goal[2] = (rel[2] > par_.wdz / 2 + 4 * par_.res) ? state_.pos.z + par_.wdz / 2 + 4 * par_.res : goal[2];
      goal[0] = (rel[0] < -par_.wdx / 2 - 4 * par_.res) ? state_.pos.x - par_.wdx / 2 - 4 * par_.res : goal[0];
      goal[1] = (rel[1] > -par_.wdy / 2 - 4 * par_.res) ? state_.pos.y - par_.wdy / 2 - 4 * par_.res : goal[1];
      goal[2] = (rel[2] > -par_.wdz / 2 - 4 * par_.res) ? state_.pos.z - par_.wdz / 2 - 4 * par_.res : goal[2];
      mtx_state.unlock();

      if (goal[2] < par_.z_ground)
      {
        goal[2] = par_.z_ground + 2 * par_.res;
      }

      if (goal[2] < par_.z_max)
      {
        goal[2] = par_.z_max;
      }
    }
  }

  // printf("       JPS check takes: %f ms\n", (double)time_solve_jps_check.Elapsed().count());
  // printf("Out3\n");

  /*  std::cout << "start despues=" << start.transpose() << std::endl;
    std::cout << "goal despues=" << goal.transpose() << std::endl;*/

  mtx_map.unlock();

  mtx_jps_map_util.lock();
  planner_ptr_->updateMap();
  // printf("Out4\n");

  // Timer time_jps(true);
  // std::cout << "Planning from start=" << start << std::endl;

  double t0JPS1 = ros::Time::now().toSec();
  bool valid_jps = planner_ptr_->plan(
      start, goal, 1, true);  // Plan from start to goal with heuristic weight=1, and using JPS (if false --> use A*)
                              // printf("       JPS solver takes: %f ms\n", (double)time_jps.Elapsed().count());
  if (i == 1)
  {
    log_.JPS1_ms = 1000 * (ros::Time::now().toSec() - t0JPS1);
  }

  vec_Vecf<3> path;
  path.clear();

  if (valid_jps == true)  // There is a solution
  {
    // double dt_jps = time_jps.Elapsed().count();
    // printf("JPS Planner takes: %f ms\n", dt_jps);
    // printf("JPS Path Distance: %f\n", total_distance3f(planner_ptr->getPath()));  // getpar_.RawPath() if you want
    // the path with more corners (not "cleaned") printf("JPS Path: \n");

    // printf("after cleaning:\n");
    // printElementsOfJPS(path);
    path = planner_ptr_->getPath();  // getpar_.RawPath() if you want the path with more corners (not "cleaned")
    path[0] = start;
    path[path.size() - 1] = goal;  // force to start and end in the start and goal (and not somewhere in the voxel)
    // path.insert(path.begin(), originalStart);
    /*    printf("First point in path_jps_vector_:\n");
        std::cout << path_jps_vector_[0].transpose() << std::endl;*/
    // directionJPS_ = path_jps_vector_[1] - path_jps_vector_[0];
    // printf("Estoy aqui: \n");
    /*    for (const auto& it : path_jps_vector)
        {
          std::cout << it.transpose() << std::endl;
        }*/
  }
  else
  {
    printf("Solution not found\n");
  }
  mtx_jps_map_util.unlock();

  *solved = valid_jps;
  /*
 Timer time_astar(true);
 bool valid_astar = planner_ptr->plan(start, goal, 1, false);  // Plan from start to goal using A*
 double dt_astar = time_astar.Elapsed().count();
 printf("AStar Planner takes: %f ms\n", dt_astar);
 printf("AStar Path Distance: %f\n", total_distance3f(planner_ptr->getpar_.RawPath()));
 printf("AStar Path: \n");
 auto path_astar = planner_ptr->getpar_.RawPath();
 for (const auto& it : path_astar)
   std::cout << it.transpose() << std::endl;
*/
  // printf("Out of solveJPSD\n");
  // printf("Out6\n");
  return path;
}

/*visualization_msgs::MarkerArray CVX::clearArrows()
{
  visualization_msgs::MarkerArray tmp;
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::DELETEALL;
  m.id = 0;
  tmp.markers.push_back(m);
  pub_path_jps_.publish(tmp);
  visualization_msgs::MarkerArray new_array;
  return new_array;
}*/

void CVX::vectorOfVectors2MarkerArray(vec_Vecf<3> traj, visualization_msgs::MarkerArray* m_array,
                                      std_msgs::ColorRGBA color, int type)
{
  // printf("In vectorOfVectors2MarkerArray\n");
  geometry_msgs::Point p_last = eigen2point(traj[0]);

  bool skip = false;
  int i = 50000;  // large enough to prevent conflict with other markers

  for (const auto& it : traj)
  {
    i++;
    if (skip and type == visualization_msgs::Marker::ARROW)  // skip the first element
    {
      skip = true;
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
    /*    p.x = traj[i][0];
        p.y = X(i, 1);
        p.z = X(i, 2);*/
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
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.1;
      m.pose.position = p;
    }
    (*m_array).markers.push_back(m);
  }
}

void CVX::goalCB(const acl_msgs::TermGoal& msg)
{
  printf("NEW GOAL************************************************\n");
  mtx_term_term_goal.lock();
  term_term_goal_ = Eigen::Vector3d(msg.pos.x, msg.pos.y, msg.pos.z);
  mtx_term_term_goal.unlock();
  // std::cout << "term_term_goal_=\n" << term_term_goal_ << std::endl;
  mtx_term_goal.lock();
  mtx_state.lock();
  Eigen::Vector3d temp(state_.pos.x, state_.pos.y, state_.pos.z);
  term_goal_ = projectClickedGoal(temp);
  mtx_state.unlock();
  mtx_term_goal.unlock();
  // std::cout << "term_goal_=\n" << term_goal_ << std::endl;

  status_ = (status_ == GOAL_REACHED) ? YAWING : TRAVELING;
  if (status_ == YAWING)
  {
    // printf("GCB: status_ = YAWING\n");
  }
  if (status_ == TRAVELING)
  {
    // printf("GCB: status_ = TRAVELING\n");
  }
  mtx_planner_status_.lock();
  planner_status_ = START_REPLANNING;
  mtx_planner_status_.unlock();
  force_reset_to_0_ = true;
  printf("GCB: planner_status_ = START_REPLANNING\n");
  goal_click_initialized_ = true;
  clearMarkerActualTraj();
  // printf("Exiting from goalCB\n");
}

void CVX::yaw(double diff, acl_msgs::QuadGoal& quad_goal)
{
  saturate(diff, -par_.dc * par_.w_max, par_.dc * par_.w_max);
  if (diff > 0)
    quad_goal.dyaw = par_.w_max;
  else
    quad_goal.dyaw = -par_.w_max;
  quad_goal.yaw += diff;
}

vec_Vecf<3> CVX::fix(vec_Vecf<3> JPS_old, Eigen::Vector3d start, Eigen::Vector3d goal, bool* solved)
{
  vec_Vecf<3> fix;
  bool thereIsIntersection = false;
  /*  vec_Vecf<3> null1(1, Eigen::Vector3d::Zero());
    vec_Vecf<3> null2(1, Eigen::Vector3d::Zero());
    vec_Vecf<3>& path_start2fix(null1);  // references has to be initialized
    vec_Vecf<3>& path_fix2goal(null2);*/
  vec_Vecf<3> path_start2fix;  // referenceFs has to be initialized
  vec_Vecf<3> path_fix2goal;
  path_start2fix.clear();
  path_fix2goal.clear();
  vec_Vecf<3> path_fixed;

  int el_eliminated;  // not used

  Eigen::Vector3d inters1 =
      getFirstCollisionJPS(JPS_old, &thereIsIntersection, el_eliminated);  // intersection starting from start

  if (thereIsIntersection)
  {
    clearJPSPathVisualization(2);
    vec_Vecf<3> tmp = JPS_old_;
    std::reverse(tmp.begin(), tmp.end());  // flip all the vector
    Eigen::Vector3d inters2 =
        getFirstCollisionJPS(tmp, &thereIsIntersection, el_eliminated);  // intersection starting from the goal

    // std::reverse(path_fix2goal.begin(), path_fix2goal.end());
    bool solvedFix, solvedStart2Fix, solvedFix2Goal;

    // printf("Calling to fix from\n");
    // std::cout << inters1.transpose() << std::endl << "to" << inters2.transpose() << std::endl;
    fix = solveJPS3D(inters1, inters2, &solvedFix, 2);
    // printf("AQUI2\n");

    path_start2fix = solveJPS3D(start, inters1, &solvedStart2Fix, 2);

    // printf("AQUI3\n");
    path_fix2goal = solveJPS3D(inters2, goal, &solvedFix2Goal, 2);

    // printf("AQUI4\n");

    // printf("After calling solveJPSD\n");
    bool solved_complete_fix = solvedFix && solvedStart2Fix && solvedFix2Goal;
    if (solved_complete_fix == false)
    {
      printf("**************Couldn't find some part of the fixed path**********\n");
      *solved = false;
    }

    else
    {
      *solved = true;
      // printf("AQUI1\n");
      /*printf("solution found!\n");
      printf("El path deberia ir\n");
      std::cout << JPS_old[0].transpose() << "--->" << JPS_old[JPS_old.size() - 1].transpose();
      std::cout << "Pasando por" << inters1.transpose() << "y  " << inters2.transpose() << std::endl;*/
      path_fixed.clear();
      path_fixed.insert(path_fixed.end(), path_start2fix.begin(), path_start2fix.end());
      path_fixed.insert(path_fixed.end(), fix.begin(), fix.end());
      path_fixed.insert(path_fixed.end(), path_fix2goal.begin(), path_fix2goal.end());
      /*printf("***************Start to fix******************\n");
      printElementsOfJPS(path_start2fix);
      printf("***************Fix***************************\n");
      printElementsOfJPS(fix);
      printf("***************Fix to Goal***************************\n");
      printElementsOfJPS(path_fix2goal);
      printf("***************Everything***************************\n");
      printElementsOfJPS(path_fixed);*/
      if (par_.visual == true)
      {
        publishJPS2handIntersection(JPS_old, path_fixed, inters1, inters2, solved_complete_fix);
      }
    }

    // printf("Before publishing\n");

    // printf("published\n");

    // printf("Solved, fix=:\n");
    // printElementsOfJPS(fix);
  }

  else
  {
    /*    printf("there is no intersection\n");
        std::cout << "the start is " << start.transpose() << std::endl;
        std::cout << "the goal is " << goal.transpose() << std::endl;*/

    *solved = true;
    JPS_old[0] = start;
    JPS_old[JPS_old.size() - 1] = goal;
    fix = JPS_old;
  }
  // printf("finisshing fix\n");

  return fix;
}

void CVX::replanCB(const ros::TimerEvent& e)
{
  // printf("IN replan CB!!!\n");

  log_.cvx_jerk_total_ms = 0;
  log_.cvx_vel_total_ms = 0;

  log_.loops_jerk = 0;
  log_.loops_vel = 0;

  log_.coll_total_ms = 0;
  log_.loops_col = 0;

  log_.computed_both = 0;

  // Timer time_init(true);
  // printf("In replanCB0\n");
  double t0replanCB = ros::Time::now().toSec();
  mtx_state.lock();
  Eigen::Vector3d state_pos(state_.pos.x, state_.pos.y, state_.pos.z);  // Local copy of state
  mtx_state.unlock();

  mtx_term_goal.lock();
  term_goal_ = projectClickedGoal(state_pos);
  // std::cout << "Projected Goal" << term_goal_.transpose() << std::endl;
  mtx_term_goal.unlock();
  // printf("In replanCB0.1\n");
  if (par_.visual == true)
  {
    clearMarkerSetOfArrows();
    pubintersecPoint(Eigen::Vector3d::Zero(), false);  // Clear the intersection points markers
  }
  if (!goal_click_initialized_)
  {
    ROS_WARN("Click a goal to start replanning");
    return;
  }

  if (!kdtree_map_initialized_)
  {
    ROS_WARN("Waiting to initialize kdTree_map");
    return;
  }

  // printf("In replanCB0.3\n");
  mtx_term_goal.lock();
  Eigen::Vector3d term_goal = term_goal_;  // Local copy of the terminal goal
  mtx_term_goal.unlock();
  double dist_to_goal = (term_goal - state_pos).norm();

  /*  std::cout << "rb=" << rb << std::endl;*/
  // std::cout << "dist_to_goal=" << dist_to_goal << std::endl;

  if (dist_to_goal < par_.goal_radius && status_ != GOAL_REACHED)
  {
    status_ = GOAL_REACHED;
    // printf("STATUS=GOAL_REACHED\n");
  }
  // printf("Entering in replanCB, planner_status_=%d\n", planner_status_);
  // printf("In replanCB0.4s\n");
  if (status_ == GOAL_SEEN || status_ == GOAL_REACHED || planner_status_ == REPLANNED || status_ == YAWING)
  {
    // printf("No replanning needed because planner_status_=%d\n", planner_status_);
    // printf("or because status_=%d\n", status_);
    return;
  }

  // printf("ReplanCB: Init takes %f ms\n", (double)time_init.Elapsed().count());

  // Timer time_med(true);
  ///////////////////////////////////////////////////////////////////////////////

  bool have_seen_the_goal1 = false, have_seen_the_goal2 = false;
  bool found_one_1 = false, found_one_2 = false;  // found at least one free trajectory
  bool need_to_decide = true;
  bool solvedjps1 = false, solvedjps2 = false;

  int li1;    // last index inside the sphere of JPS1
  int li2;    // last index inside the sphere of JPS2
  int liold;  // last index inside the sphere of JPS2

  double dist1 = 0, dist2 = 0, J1 = 0, J2 = 0, JPrimj1 = 0, JPrimj2 = 0, JPrimv1 = 0, JPrimv2 = 0, JDist1 = 0,
         JDist2 = 0;
  J1 = std::numeric_limits<double>::max();
  J2 = std::numeric_limits<double>::max();
  Eigen::MatrixXd U_temp1, U_temp2, X_temp1, X_temp2;

  Eigen::Vector3d B1, B2, C1, C2, B_old;
  vec_Vecf<3> WP1, WP2;

  vec_Vecf<3> JPS1;
  vec_Vecf<3> JPS2;

  // printf("init2\n");
  // std::cout << "Running JPS3d from=" << state_pos.transpose() << std::endl;
  // std::cout << "Running JPS3d to terminal goal=" << term_goal.transpose() << std::endl;

  static bool first_time = true;  // how many times I've solved JPS1

  printf("Running JPS1!!!\n");
  JPS1 = solveJPS3D(state_pos, term_goal, &solvedjps1, 1);  // Solution is in JPS1
                                                            // printf("Aqui89\n");
  printf("Solved JPS1!!!\n");

  // std::cout << "solvedjps1= " << solvedjps1 << std::endl;

  bool previous = solvedjps1;

  // 0.96 and 0.98 are to ensure that ra<rb<dist_to_goal always
  double ra = std::min(0.96 * dist_to_goal, par_.Ra);
  double rb = std::min(0.98 * dist_to_goal, par_.Rb);  // radius of the sphere Sbl

  if (solvedjps1 == true && flight_mode_.mode == flight_mode_.GO)
  {
    // ra = (JPS1[1] - JPS1[0]).norm();
    saturate(ra, par_.Ra, par_.Ra_max);
  }

  ra = std::min(0.96 * dist_to_goal, ra);  // radius of the sphere Sa

  /*  if (par_.use_vel == false)
    {
      rb = 1.001 * ra;
    }*/

  if (flight_mode_.mode != flight_mode_.GO)
  {
    mtx_state.lock();
    ra = term_goal[2] - state_.pos.z;
    mtx_state.unlock();
  }

  if (first_time == true)
  {
    first_time = false;
    solvedjps2 = solvedjps1;
    JPS2 = JPS1;
    JPS_old_ = JPS1;
    solvedjps2 = true;  // only the first time
  }
  else
  {
  }

  if (solvedjps1 == true)
  {
    JPS1_solved_ = true;
    if (par_.visual == true)
    {
      clearJPSPathVisualization(1);
      publishJPSPath(JPS1, 1);
    }
    // printf("ReplanCB: Elements of JPS1 are...\n");
    // printElementsOfJPS(JPS1);
    /*
            printf("Elements of JPS_old_ are...:\n");
            printElementsOfJPS(JPS_old_);*/
  }
  else
  {
    printf("JPS1 didn't find a solution\n");
    JPS1_solved_ = false;
    return;
  }

  // std::cout << state_pos.transpose() << std::endl;
  bool noPointsOutsideSphere1;
  bool noPointsOutsideSphere2;
  B1 = getFirstIntersectionWithSphere(JPS1, ra, state_pos, &li1, &noPointsOutsideSphere1);

  /*  printf("ReplanCB: Elements of JPS1 are...\n");
    printElementsOfJPS(JPS1);
    printf("B1 is:");
    std::cout << B1.transpose() << std::endl;*/
  vec_Vecf<3> JPS1_inside_sphere(JPS1.begin(), JPS1.begin() + li1 + 1);  // Elements of JPS that are inside the sphere
  JPS1_inside_sphere.push_back(B1);
  // printf("ReplanCB: Elements of JPS1_inside_sphere are...\n");
  // printElementsOfJPS(JPS1_inside_sphere);

  B_old = getFirstIntersectionWithSphere(JPS_old_, ra, state_pos, &liold, &noPointsOutsideSphere2);

  Eigen::Vector3d v1 = B1 - state_pos;  // point i expressed with origin=origin sphere

  //////////////////////////////////////////////////////////////////////////
  //////////// Solve with GUROBI Whole trajectory //////////////////////////
  /////////////////////////////////////////////////////////////////////////
  printf("Running Gurobi!!!\n");
  if (X_initialized_)  // Needed to skip the first time (X_ still not initialized)
  {
    mtx_k.lock();
    k_initial_cond_1_ = std::min(k_ + par_.offset, (int)(X_.rows() - 1));
    // printf("Ahora mismo, k_initial_cond_1=%d\n", k_initial_cond_1_);
    mtx_k.unlock();
    updateInitialCond(k_initial_cond_1_);
  }

  mtx_initial_cond.lock();
  double x0[9] = { initialCond_.pos.x,   initialCond_.pos.y,   initialCond_.pos.z,
                   initialCond_.vel.x,   initialCond_.vel.y,   initialCond_.vel.z,
                   initialCond_.accel.x, initialCond_.accel.y, initialCond_.accel.z };
  mtx_initial_cond.unlock();
  double xf[9] = { B1[0], B1[1], B1[2], 0, 0, 0, 0, 0, 0 };
  // printf("Running CVXDecomp!!!\n");
  double before = ros::Time::now().toSec();
  cvxDecomp(JPS1_inside_sphere);  // result saved in l_constraints_
  ROS_WARN("CVXDecomp time: %0.2f ms", 1000 * (ros::Time::now().toSec() - before));
  printf("Solved CVXDecomp!!!\n");

  solver_gurobi_.setPolytopes(l_constraints_);
  vec_Vecf<3> samples_empty;
  std::vector<double> dist_empty;
  solver_gurobi_.setDistances(samples_empty, dist_empty);  // No distance constraints for the normal path
  solver_gurobi_.setXf(xf);
  solver_gurobi_.setX0(x0);
  bool solved_whole = solver_gurobi_.genNewTraj();

  if (solved_whole == false)
  {
    ROS_ERROR("No solution found for the whole trajectory");
    return;
  }

  if (par_.visual == true)
  {
    pubTraj(X_temp_, WHOLE);
  }

  printf("Solved Gurobi!!!\n");

  mtx_X_U_temp.lock();
  U_temp_ = solver_gurobi_.getU();
  X_temp_ = solver_gurobi_.getX();
  mtx_X_U_temp.unlock();

  ///////////////////////////////////////////////////////////
  ///////////////       RESCUE PATH    //////////////////////
  ///////////////////////////////////////////////////////////

  int index = ceil(0.04 / par_.dc);  // R is the point of the trajectory 40 ms after the start of the trajectory
                                     // TODO: change 0.04 to parameter
  Eigen::Vector3d posR(X_temp_(index, 0), X_temp_(index, 1), X_temp_(index, 2));
  Eigen::Vector3d velR(X_temp_(index, 3), X_temp_(index, 4), X_temp_(index, 5));
  Eigen::Vector3d accelR(X_temp_(index, 6), X_temp_(index, 7), X_temp_(index, 8));

  vec_Vecf<3> JPS1r = JPS1_inside_sphere;  // JPS1 used for the rescue path
  JPS1r[0] = posR;

  printf("Going to compute rescue path");
  bool thereIsIntersection = false;
  Eigen::Vector3d I;  // I is the first intersection of JPS with the unknown space

  // Part of JPS1 in known space
  int el_eliminated;
  I = getFirstCollisionJPS(JPS1r, &thereIsIntersection, el_eliminated, UNKNOWN_MAP);  // Intersection with unkown space

  // printf("Elements_eliminated=%d\n", el_eliminated);

  vec_Vecf<3> JPS1_in_known_space(JPS1r.begin(),
                                  JPS1r.begin() + el_eliminated +
                                      1);  // Element of JPS that are inside the unkown space

  if (thereIsIntersection == true)
  {
    JPS1_in_known_space.push_back(I);  // Plus the intersection with the unkown space
  }
  else
  {
    JPS1_in_known_space.push_back(JPS1r[JPS1r.size() - 1]);  // Plus the last element of JPS
  }

  // printf("These is the part of JPS that is in known space:\n");
  // printElementsOfJPS(JPS1_in_known_space);

  vec_Vecf<3> samples = sampleJPS(JPS1_in_known_space, par_.N + 1);  // Take N +1 samples along JPS1_in_known_space; IT
                                                                     // has to be N+1, because N is the number of
                                                                     // segments
  std::vector<double> dist_near_neig = getDistToNearestObs(samples);

  /*  printf("These are the samples along the path:\n");
    printElementsOfJPS(samples);
    printf("**********************");*/

  clearMarkerArray(&samples_rescue_path_, &pub_samples_rescue_path_);
  vectorOfVectors2MarkerArray(samples, &samples_rescue_path_, color(BLUE), visualization_msgs::Marker::SPHERE);
  pub_samples_rescue_path_.publish(samples_rescue_path_);

  double x0_rescue[9] = { posR[0], posR[1], posR[2], velR[0], velR[1], velR[2], accelR[0], accelR[1], accelR[2] };

  Eigen::Vector3d F = samples[samples.size() - 1];  // F is the final point of the rescue path (will be near I)
  double xf_rescue[9] = { F[0], F[1], F[2], 0, 0, 0, 0, 0, 0 };

  std::vector<LinearConstraint3D> empty;
  solver_gurobi_.setPolytopes(empty);  // No Polytopes constraints for the rescue path
  solver_gurobi_.setDistances(samples, dist_near_neig);
  solver_gurobi_.setXf(xf_rescue);
  solver_gurobi_.setX0(x0_rescue);
  bool solved_rescue = solver_gurobi_.genNewTraj();

  if (solved_rescue == false)
  {
    ROS_ERROR("No solution found for the rescue path");
    return;
  }

  mtx_X_U_rescue.lock();
  U_rescue_ = solver_gurobi_.getU();
  X_rescue_ = solver_gurobi_.getX();
  mtx_X_U_rescue.unlock();

  // And now the trajectory saved is X_temp_ will be the first part corresponding to the X_temp_, and the second part is
  // X_rescue_

  mtx_X_U_temp.lock();
  mtx_X_U_rescue.lock();

  // printf("Going to resize\n");
  U_temp_.conservativeResize(index + U_rescue_.rows(), U_temp_.cols());     // Increase the number of rows of U_temp_
  X_temp_.conservativeResize(index + X_rescue_.rows(), X_temp_.cols());     // Increase the number of rows of X_temp_
  U_temp_.block(index, 0, U_rescue_.rows(), U_rescue_.cols()) = U_rescue_;  // and copy the part of the rescue path
  X_temp_.block(index, 0, X_rescue_.rows(), X_rescue_.cols()) = X_rescue_;  // and copy the part of the rescue path

  // printf("Copied block\n");
  mtx_X_U_temp.unlock();
  mtx_X_U_rescue.unlock();

  if (par_.visual == true)
  {
    pubTraj(X_rescue_, RESCUE);
  }

  ///////////////////////////////////////////////////////////
  ///////////////       OTHER STUFF    //////////////////////
  ///////////////////////////////////////////////////////////
  JPS_old_ = JPS1;
  mtx_k.lock();
  k_initial_cond_ = k_initial_cond_1_;
  // printf("Ahora mismo, k_initial_cond_=%d and k_=%d\n", k_initial_cond_, k_);
  mtx_k.unlock();
  B_ = B1;
  optimized_ = true;
  mtx_planner_status_.lock();
  planner_status_ = REPLANNED;
  mtx_planner_status_.unlock();
  // printf("ReplanCB: planner_status_ = REPLANNED\n");

  double dist = (term_goal - B1).norm();
  bool have_seen_the_goal = (dist < par_.goal_radius) ? true : false;
  if (have_seen_the_goal)
  {
    status_ = GOAL_SEEN;
  }

  return;
  ///////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////
}

// Get the distance to the nearest obstacle for each point given in the vector "points"
std::vector<double> CVX::getDistToNearestObs(vec_Vecf<3>& points)
{
  mtx_unk.lock();
  mtx_map.lock();
  std::vector<double> distances;
  for (int i = 0; i < points.size(); i++)
  {
    int N = 1;
    pcl::PointXYZ searchPoint(points[i][0], points[i][1], points[i][2]);
    std::vector<int> id(N);
    std::vector<float> dist2(N);  // squared distance
    double distance_map, distance_unk;

    // Find nearest obstacles in Unkown space
    distance_unk = kdtree_unk_.nearestKSearch(searchPoint, N, id, dist2) > 0 ? sqrt(dist2[0]) : 100;
    // printf("Distance unkown=%f\n", distance_unk);

    // Find nearest obstacles in MAP
    distance_map = kdtree_map_.nearestKSearch(searchPoint, N, id, dist2) > 0 ? sqrt(dist2[0]) : 100;
    // printf("Distance map=%f\n", distance_map);
    // printf("******");

    distances.push_back(std::min(distance_map, distance_unk));
  }
  // std::cout << "Distances computed are: " << distances << std::endl;
  mtx_unk.unlock();
  mtx_map.unlock();
  return distances;
}

// Sample n points along the path
vec_Vecf<3> CVX::sampleJPS(vec_Vecf<3>& path, int n)
{
  std::vector<double> distance;  // vector that contains the distance (going along the path) of each point in the path
                                 // to the first point of the path
  distance.push_back(0);
  for (int i = 1; i < path.size(); i++)
  {
    double distance_so_far = distance[distance.size() - 1];
    distance.push_back(distance_so_far + (path[i] - path[i - 1]).norm());
  }

  // note that distance and path have the same number of elements

  double total_distance = getDistancePath(path);
  double d = total_distance / n;  // distance between samples

  vec_Vecf<3> samples;
  samples.push_back(path[0]);
  double dist_next_sample = 0;  // Distancia a la que quiero poner el next sample

  Eigen::Vector3d next_peak, previous_peak;
  double difference;
  for (int n_samples = 1; n_samples < n; n_samples++)
  {
    dist_next_sample = dist_next_sample + d;

    Eigen::Vector3d previous_peak;
    for (int i = 1; i < distance.size(); i++)
    {
      if (distance[i] > dist_next_sample)
      {
        previous_peak = path[i - 1];
        next_peak = path[i];
        difference = dist_next_sample - distance[i - 1];
        break;
      }
    }

    Eigen::Vector3d v = (next_peak - previous_peak).normalized();

    Eigen::Vector3d last_sample = samples[samples.size() - 1];
    Eigen::Vector3d new_sample = previous_peak + difference * v;
    samples.push_back(new_sample);
  }
  return samples;
}

double CVX::solveVelAndGetCost(vec_Vecf<3> path)
{
  double cost = 0;
  /*  printf("Points in the path VEL\n");
    for (int i = 0; i < path.size() - 1; i++)
    {
      std::cout << path[i].transpose(s) << std::endl;
    }*/

  for (int i = 0; i < path.size() - 1; i++)
  {
    double xf[3] = { path[i + 1][0], path[i + 1][1], path[i + 1][2] };

    double x0[3] = { path[i][0], path[i][1], path[i][2] };
    // double u0[3] = { initialCond_.vel.x, initialCond_.vel.y, initialCond_.vel.z };
    solver_vel_.set_xf(xf);
    solver_vel_.set_x0(x0);
    // solver_vel_.set_u0(u0);
    double t0cvxgen_vel = ros::Time::now().toSec();
    solver_vel_.genNewTraj();
    log_.cvx_vel_total_ms = log_.cvx_vel_total_ms + 1000 * (ros::Time::now().toSec() - t0cvxgen_vel);
    log_.loops_vel = log_.loops_vel + 1;
    cost = cost + solver_vel_.getCost();
  }
  return cost;
}

void CVX::modeCB(const acl_msgs::QuadFlightMode& msg)
{
  // printf("In modeCB\n");
  if (msg.mode == msg.LAND && flight_mode_.mode != flight_mode_.LAND)
  {
    printf("LANDING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    // Solver Vel
    /*    double xf[6] = { quadGoal_.pos.x, quadGoal_.pos.y, z_land_ };
        solver_vel_.set_xf(xf);
        solver_vel_.genNewTraj();*/

    // Solver Accel
    /*    double xf[6] = { quadGoal_.pos.x, quadGoal_.pos.y, z_land_, 0, 0, 0 };
        double max_values[2] = { par_.v_max, par_.a_max };
        solver_accel_.set_max(max_values);  // TODO: To land, I use u_min_
        solver_accel_.set_xf(xf);
        solver_accel_.genNewTraj();*/
    mtx_state.lock();
    double x0[9] = { state_.pos.x, state_.pos.y, state_.pos.z, state_.vel.x, state_.vel.y, state_.vel.z, 0, 0, 0 };

    double xf[9] = { state_.pos.x, state_.pos.y, par_.z_land, 0, 0, 0, 0, 0, 0 };
    mtx_state.unlock();
    mtx_goals.unlock();
    // printf("Hola6.7\n");
    solver_jerk_.set_xf(xf);
    solver_jerk_.set_x0(x0);

    k_initial_cond_ = std::min(k_ + par_.offset, (int)(X_.rows() - 1));
    // Solver Jerk
    // double xf[9] = { quadGoal_.pos.x, quadGoal_.pos.y, par_.z_land, 0, 0, 0, 0, 0, 0 };
    // TODO: To land, I use u_min_
    solver_jerk_.set_x0(x0);
    solver_jerk_.set_xf(xf);
    solver_jerk_.genNewTraj();
    to_land_ = true;
    mtx_X_U_temp.lock();
    U_temp_ = solver_jerk_.getU();
    X_temp_ = solver_jerk_.getX();
    mtx_X_U_temp.unlock();
    mtx_planner_status_.lock();
    planner_status_ = REPLANNED;
    mtx_planner_status_.unlock();
  }
  flight_mode_.mode = msg.mode;
}

void CVX::stateCB(const acl_msgs::State& msg)
{
  // ROS_ERROR("In state CB");
  // printf("(State): %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", msg.pos.x, msg.pos.y, msg.pos.z, msg.vel.x, msg.vel.y,
  //       msg.vel.z);
  mtx_state.lock();
  state_ = msg;
  mtx_state.unlock();
  // Stop updating when we get GO
  if (flight_mode_.mode == flight_mode_.NOT_FLYING || flight_mode_.mode == flight_mode_.KILL)
  {
    quadGoal_.pos = msg.pos;
    quadGoal_.vel = msg.vel;
    z_start_ = msg.pos.z;
    z_start_ = std::max(0.0, z_start_);
    mtx_initial_cond.lock();
    initialCond_.pos = msg.pos;
    mtx_initial_cond.unlock();
  }

  static int i = 0;
  i++;

  if (status_ != GOAL_REACHED && par_.visual == true)
  {
    pubActualTraj();
  }

  if (i % 10 == 0)
  {
    Eigen::Vector3d actual_pos(msg.pos.x, msg.pos.y, msg.pos.z);
    log_.total_dist = log_.total_dist + (actual_pos - pos_old_).norm();
    pos_old_ = actual_pos;
  }
  Eigen::Vector3d vel(msg.vel.x, msg.vel.y, msg.vel.z);
  log_.veloc_norm = vel.norm();
}

void CVX::updateInitialCond(int i)
{
  if (status_ != GOAL_REACHED)
  {
    mtx_initial_cond.lock();
    initialCond_.pos = getPos(i);
    initialCond_.vel = getVel(i);
    initialCond_.accel = (par_.use_ff) ? getAccel(i) : vectorNull();
    initialCond_.jerk = (par_.use_ff) ? getJerk(i) : vectorNull();
    mtx_initial_cond.unlock();
  }

  else
  {
    mtx_initial_cond.lock();
    mtx_state.lock();
    initialCond_.pos = state_.pos;
    mtx_state.unlock();
    initialCond_.vel = vectorNull();
    initialCond_.accel = vectorNull();
    initialCond_.jerk = vectorNull();
    mtx_initial_cond.unlock();
  }
}

void CVX::pubCB(const ros::TimerEvent& e)
{
  mtx_goals.lock();
  // printf("GOing to publish\n");

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

  quadGoal_.vel = vectorNull();
  quadGoal_.accel = vectorNull();
  quadGoal_.jerk = vectorNull();
  quadGoal_.dyaw = 0;

  mtx_initial_cond.lock();
  initialCond_.vel = vectorNull();
  initialCond_.accel = vectorNull();
  initialCond_.jerk = vectorNull();
  mtx_initial_cond.unlock();

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
      ROS_WARN("Optimization took too long. Increase par_.offset");
    }

    if ((planner_status_ == REPLANNED && (k_ >= k_initial_cond_ || to_land_ == true)) ||  // Should be k_==
        (force_reset_to_0_ && planner_status_ == REPLANNED))
    {
      to_land_ == false;
      printf("************Reseteando a 0!\n");
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
       // printf("Rejecting current plan, planning again. Suggestion: Increase the offset\n");
      mtx_planner_status_.lock();
      planner_status_ = START_REPLANNING;
      mtx_planner_status_.unlock();
    }

    k_ = std::min(k_, (int)(X_.rows() - 1));
    /*    printf("k_=%d\n", k_);
        printf("X_.rows() - 1=%d\n", (int)(X_.rows() - 1));
        std::cout << "PubCB: Esto es lo que tengo por delante, voy a publicar la 1a fila" << std::endl;
        std::cout << X_.block(k_, 0, 10, 1) << std::endl;*/

    mtx_k.unlock();
    // int kp1 = std::min(k_ + par_.offset, (int)(X_.rows() - 1));  // k plus offset

    quadGoal_.pos = getPos(k_);
    quadGoal_.vel = getVel(k_);
    quadGoal_.accel = (par_.use_ff) ? getAccel(k_) : vectorNull();
    quadGoal_.jerk = (par_.use_ff) ? getJerk(k_) : vectorNull();
    quadGoal_.dyaw = 0;

    // heading_ = atan2(goal_(1) - X_(0, 1), goal_(0) - X_(0, 0));

    if (status_ == YAWING)
    {
      // mtx_term_goal.lock();
      double desired_yaw = atan2(term_goal_[1] - quadGoal_.pos.y, term_goal_[0] - quadGoal_.pos.x);
      // mtx_term_goal.unlock();
      double diff = desired_yaw - quadGoal_.yaw;
      angle_wrap(diff);
      yaw(diff, quadGoal_);
      /*      printf("Inside, desired_yaw=%0.2f,quadGoal_.yaw=%0.2f, diff=%f , abs(diff)=%f\n", desired_yaw,
         quadGoal_.yaw, diff, fabs(diff));*/
      if (fabs(diff) < 0.2)
      {
        // printf("It's less than 0.2!!\n");
        status_ = TRAVELING;
        printf("status_=TRAVELING\n");
      }
      else
      {
        printf("Yawing\n");
      }
    }

    if ((status_ == TRAVELING || status_ == GOAL_SEEN))
    {
      // double desired_yaw = atan2(quadGoal_.vel.y, quadGoal_.vel.x);
      double desired_yaw = atan2(B_[1] - quadGoal_.pos.y, B_[0] - quadGoal_.pos.x);
      double diff = desired_yaw - quadGoal_.yaw;
      angle_wrap(diff);
      if (JPS1_solved_ == true)
      {
        yaw(diff, quadGoal_);
      }

      if (JPS1_solved_ == false)
      {
        quadGoal_.dyaw = 0;
      }
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
  // printf("(initialCond_): %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", initialCond_.pos.x, initialCond_.pos.y,
  //       initialCond_.pos.z, initialCond_.vel.x, initialCond_.vel.y, initialCond_.vel.z);

  pub_goal_.publish(quadGoal_);

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

geometry_msgs::Vector3 CVX::getPos(int i)
{
  int input_order = solver_jerk_.getOrder();
  geometry_msgs::Vector3 tmp;
  mtx_X_U.lock();
  tmp.x = X_(i, 0);
  tmp.y = X_(i, 1);
  tmp.z = X_(i, 2);
  mtx_X_U.unlock();
  return tmp;
}

geometry_msgs::Vector3 CVX::getVel(int i)
{
  int input_order = solver_jerk_.getOrder();
  geometry_msgs::Vector3 tmp;
  mtx_X_U.lock();
  switch (input_order)
  {
    case VEL:
      tmp.x = U_(i, 0);
      tmp.y = U_(i, 1);
      tmp.z = U_(i, 2);
      break;
    case ACCEL:
      tmp.x = X_(i, 3);
      tmp.y = X_(i, 4);
      tmp.z = X_(i, 5);
      break;
    case JERK:
      tmp.x = X_(i, 3);
      tmp.y = X_(i, 4);
      tmp.z = X_(i, 5);
      break;
  }
  mtx_X_U.unlock();
  return tmp;
}

geometry_msgs::Vector3 CVX::getAccel(int i)
{
  int input_order = solver_jerk_.getOrder();
  geometry_msgs::Vector3 tmp;
  mtx_X_U.lock();
  switch (input_order)
  {
    case VEL:
      tmp.x = U_(i, 3);
      tmp.y = U_(i, 4);
      tmp.z = U_(i, 5);
      break;
    case ACCEL:
      tmp.x = U_(i, 0);
      tmp.y = U_(i, 1);
      tmp.z = U_(i, 2);
    case JERK:
      tmp.x = X_(i, 6);
      tmp.y = X_(i, 7);
      tmp.z = X_(i, 8);
      break;
  }
  mtx_X_U.unlock();

  return tmp;
}

geometry_msgs::Vector3 CVX::getJerk(int i)
{
  int input_order = solver_jerk_.getOrder();
  geometry_msgs::Vector3 tmp;

  mtx_X_U.lock();
  switch (input_order)
  {
    case VEL:
      printf("Input is Vel --> returning jerk=0\n");
      tmp.x = 0;
      tmp.y = 0;
      tmp.z = 0;
      break;
    case ACCEL:
      tmp.x = U_(i, 3);
      tmp.y = U_(i, 4);
      tmp.z = U_(i, 5);
    case JERK:
      tmp.x = U_(i, 0);
      tmp.y = U_(i, 1);
      tmp.z = U_(i, 2);
      break;
  }
  mtx_X_U.unlock();
  return tmp;
}

void CVX::cvxDecomp(vec_Vecf<3> path)
{
  // std::cout << "In cvxDecomp 0!" << std::endl;
  if (kdtree_map_initialized_ == false)
  {
    return;
  }
  // std::cout << "In cvxDecomp 1!" << std::endl;
  pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr_cloud = kdtree_map_.getInputCloud();

  // Read the point cloud from bag
  // sensor_msgs::PointCloud cloud = read_bag<sensor_msgs::PointCloud>(file_name, topic_name);

  vec_Vec3f obs = kdtree_to_vec(ptr_cloud);

  // Read path from txt
  // vec_Vec3f path;

  /*  nav_msgs::Path path_msg = DecompROS::vec_to_path(path);
    path_msg.header.frame_id = "map";
    path_pub.publish(path_msg);*/

  // Using ellipsoid decomposition
  EllipsoidDecomp3D decomp_util;
  decomp_util.set_obs(obs);
  decomp_util.set_local_bbox(
      Vec3f(2, 2, 1));       // Only try to find cvx decomp in the Mikowsski sum of JPS and this box (I think)
  decomp_util.dilate(path);  // Find convex polyhedra
  decomp_util.shrink_polyhedrons(par_.drone_radius);  // Shrink polyhedra by the drone radius

  // Publish visualization msgs
  decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(decomp_util.get_ellipsoids());
  es_msg.header.frame_id = "world";
  cvx_decomp_el_pub_.publish(es_msg);

  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
  poly_msg.header.frame_id = "world";
  cvx_decomp_poly_pub_.publish(poly_msg);

  // Convert to inequality constraints Ax < b
  // std::vector<polytope> polytopes;
  auto polys = decomp_util.get_polyhedrons();

  // std::cout << "In cvxDecomp 3!" << std::endl;
  l_constraints_.clear();

  // std::cout << "In cvxDecomp, el path llegado es:" << std::endl;
  // printElementsOfJPS(path);
  for (size_t i = 0; i < path.size() - 1; i++)
  {
    // std::cout << "Inserting constraint" << std::endl;
    const auto pt_inside = (path[i] + path[i + 1]) / 2;
    LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());
    l_constraints_.push_back(cs);
    /*      printf("i: %zu\n", i);
              std::cout << "A: " << cs.A() << std::endl;
              std::cout << "b: " << cs.b() << std::endl;
              std::cout << "point: " << path[i].transpose();
              if (cs.inside(path[i]))
                std::cout << " is inside!" << std::endl;
              else
                std::cout << " is outside!" << std::endl;

              std::cout << "point: " << path[i + 1].transpose();
              if (cs.inside(path[i + 1]))
                std::cout << " is inside!" << std::endl;
              else
                std::cout << " is outside!" << std::endl;*/
  }
}

/*void CVX::pubTraj(double** x)
{

  // Trajectory
  nav_msgs::Path traj;
  traj.poses.clear();
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = "world";

  geometry_msgs::PoseStamped temp_path;
  int N = solver_accel_.getN();
  for (int i = 1; i < N; i++)
  {
    temp_path.pose.position.x = x[i][0];
    temp_path.pose.position.y = x[i][1];
    temp_path.pose.position.z = x[i][2];
    temp_path.pose.orientation.w = 1;
    temp_path.pose.orientation.x = 0;
    temp_path.pose.orientation.y = 0;
    temp_path.pose.orientation.z = 0;
    traj.poses.push_back(temp_path);
  }
  pub_traj_.publish(traj);
}*/

void CVX::pubTraj(Eigen::MatrixXd X, int type)
{
  // Trajectory
  nav_msgs::Path traj;
  traj.poses.clear();
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = "world";

  geometry_msgs::PoseStamped temp_path;

  for (int i = 0; i < X.rows(); i++)
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
    pub_traj_.publish(traj);
  }

  if (type == RESCUE)
  {
    pub_traj_rescue_.publish(traj);
  }
}

void CVX::createMarkerSetOfArrows(Eigen::MatrixXd X, bool isFree)
{
  // printf("In createMarkerSetOfArrows, X=\n");

  /*  if (X.rows() == 0 || X.cols() == 0)
    {
      return;
    }*/

  geometry_msgs::Point p_last;

  p_last.x = X(0, 0);
  p_last.y = X(0, 1);
  p_last.z = X(0, 2);
  // TODO: change the 10 below
  for (int i = 1; i < X.rows(); i = i + 10)  // Push (a subset of) the points in the trajectory
  {
    markerID_++;
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.id = markerID_;
    m.ns = "ns";
    if (isFree)
    {
      m.color = color(BLUE_LIGHT);
    }
    else
    {
      m.color = color(RED);
    }

    m.scale.x = 0.02;
    m.scale.y = 0.04;
    // m.scale.z = 1;

    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    geometry_msgs::Point p;
    p.x = X(i, 0);
    p.y = X(i, 1);
    p.z = X(i, 2);
    m.points.push_back(p_last);
    m.points.push_back(p);
    // std::cout << "pushing marker\n" << m << std::endl;
    trajs_sphere_.markers.push_back(m);
    p_last = p;
  }
  // m.lifetime = ros::Duration(5);  // 3 second duration
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

void CVX::clearMarkerSetOfArrows()
{
  // printf("In clearMarkerSetOfArrows\n");

  trajs_sphere_.markers.clear();  // trajs_sphere_ has no elements now

  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::DELETEALL;
  m.id = 0;

  trajs_sphere_.markers.push_back(m);

  pub_trajs_sphere_.publish(trajs_sphere_);
  trajs_sphere_.markers.clear();  // trajs_sphere_ is ready to insert in it the "add" markers
  markerID_ = 0;
}

void CVX::frontierCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg)
{
  // printf("****In FrontierCB\n");
  if (pcl2ptr_msg->width == 0 || pcl2ptr_msg->height == 0)  // Point Cloud is empty (this happens at the beginning)
  {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_frontier(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_msg, *pclptr_frontier);

  // printf("In mapCB2\n");
  mtx_frontier.lock();
  // printf("Before setting InputCloud\n");
  kdtree_frontier_.setInputCloud(pclptr_frontier);
  mtx_frontier.unlock();
  // printf("In mapCB3\n");
  kdtree_frontier_initialized_ = 1;
}

void CVX::pclCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg)
{
  // printf("In pclCB\n");

  if (pcl2ptr_msg->width == 0 || pcl2ptr_msg->height == 0)  // Point Cloud is empty
  {
    return;
  }
  geometry_msgs::TransformStamped transformStamped;
  sensor_msgs::PointCloud2Ptr pcl2ptr_msg_transformed(new sensor_msgs::PointCloud2());
  try
  {
    transformStamped = tf_buffer_.lookupTransform("world", name_drone_ + "/camera", ros::Time(0));
    tf2::doTransform(*pcl2ptr_msg, *pcl2ptr_msg_transformed, transformStamped);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pcl2ptr_msg_transformed, *pclptr);

    std::vector<int> index;
    // TODO: there must be a better way to check this. It's here because (in the simulation) sometimes all the points
    // are NaN (when the drone is on the ground and stuck moving randomly). If this is not done, the program breaks. I
    // think it won't be needed in the real drone
    pcl::removeNaNFromPointCloud(*pclptr, *pclptr, index);
    if (pclptr->size() == 0)
    {
      return;
    }

    kdTreeStamped my_kdTreeStamped;
    my_kdTreeStamped.kdTree.setInputCloud(pclptr);
    my_kdTreeStamped.time = pcl2ptr_msg->header.stamp;
    mtx_inst.lock();
    // printf("pclCB: MTX is locked\n");
    v_kdtree_new_pcls_.push_back(my_kdTreeStamped);
    // printf("pclCB: MTX is unlocked\n");
    mtx_inst.unlock();
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

// Occupied CB
void CVX::mapCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg)
{
  double before = ros::Time::now().toSec();

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_msg, *pclptr_map);

  // printf("updating JSPMAP\n");
  updateJPSMap(pclptr_map);  // UPDATE EVEN WHEN THERE ARE NO POINTS!!
  // printf("updated!!\n");

  if (pcl2ptr_msg->width == 0 || pcl2ptr_msg->height == 0)  // Point Cloud is empty (this happens at the beginning)
  {
    return;
  }

  // printf("***********************************In mapCB abajo\n");
  // printf("In mapCB2\n");
  mtx_map.lock();
  // printf("Before setting InputCloud\n");
  kdtree_map_.setInputCloud(pclptr_map);
  mtx_map.unlock();
  // printf("In mapCB3\n");
  kdtree_map_initialized_ = 1;

  // printf("mapCB: MTX is locked\n");
  // printf("In mapCB4\n");
  // pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr(new pcl::PointCloud<pcl::PointXYZ>);

  // std::vector<int> index;
  // TODO: there must be a better way to check this. It's here because (in the simulation) sometimes all the points
  // are NaN (when the drone is on the ground and stuck moving randomly). If this is not done, the program breaks. I
  // think it won't be needed in the real drone
  // printf("In mapCB3\n");
  // pcl::removeNaNFromPointCloud(*pclptr_map_, *pclptr_map_, index);
  /*  if (pclptr_map_->size() == 0)
    {
      return;
    }*/

  // printf("In mapCB4, size=%d\n", v_kdtree_new_pcls_.size());
  mtx_inst.lock();
  for (unsigned i = 0; i < v_kdtree_new_pcls_.size(); ++i)
  {
    // printf("antes i=%d\n", i);
    if (v_kdtree_new_pcls_[i].time <= pcl2ptr_msg->header.stamp)  // if the map already contains that point cloud...
    {
      v_kdtree_new_pcls_.erase(v_kdtree_new_pcls_.begin() + i);  // ...erase that point cloud from the vector
      i = i - 1;  // Needed because I'm changing the vector inside the loop
    }
    // printf("despues i=%d\n", i);
  }
  mtx_inst.unlock();
  // printf("***********************************OUT mapCB\n");
  // printf("below\n");

  // mtx.unlock();
  ROS_WARN("MapCB takes: %0.2f ms", 1000 * (ros::Time::now().toSec() - before));
}

// Unkwown  CB
void CVX::unkCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg)
{
  // ROS_WARN("*************************IN unkCB\n");
  if (pcl2ptr_msg->width == 0 || pcl2ptr_msg->height == 0)  // Point Cloud is empty (this happens at the beginning)
  {
    printf("Cloud In has 0 points\n");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_msg, *cloudIn);
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*cloudIn, *cloudIn, index);
  if (cloudIn->points.size() == 0)
  {
    printf("Cloud In has 0 points\n");
    return;
  }

  // printf("Waiting for lock!");
  mtx_unk.lock();
  /*  // If the drone is nos flying/taking of/..., let's remove a box around it so that it can take off.
    if (flight_mode_.mode == flight_mode_.LAND || flight_mode_.mode == flight_mode_.TAKEOFF ||
        flight_mode_.mode == flight_mode_.NOT_FLYING || flight_mode_.mode == flight_mode_.INIT)
    {*/
  /*  printf("removing unkown space to allow takeoff\n");
    // printf("inside\n");
    // TODO A box filter would be better, I don't know why it doesn't work...

    // Note that this is not visualized in the point cloud
    float l = par_.drone_radius;
    Eigen::Vector4f minPoint;
    minPoint[0] = state_.pos.x - l;  // define minimum point x
    minPoint[1] = state_.pos.y - l;  // define minimum point y
    minPoint[2] = state_.pos.z - l;  // define minimum point z
    minPoint[3] = 1;
    Eigen::Vector4f maxPoint;
    maxPoint[0] = state_.pos.x + l;  // define maximum point x
    maxPoint[1] = state_.pos.y + l;  // define maximum point y
    maxPoint[2] = state_.pos.z + l;  // define maximum point z
    maxPoint[3] = 1;

    std::cout << minPoint << std::endl;
    std::cout << maxPoint << std::endl;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::CropBox<pcl::PointXYZ> cropFilter;

    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);
    cropFilter.setInputCloud(cloudIn);
    cropFilter.filter(*cloudFiltered);
    //I think I'm missing sth here

    if (cloudFiltered->points.size() == 0)
    {
      printf("Cloud filtered has 0 points\n");
      return;
    }
  */
  /*    // printf("Size before=%d", cloudIn->points.size());
     pcl::PassThrough<pcl::PointXYZ> pass;
     pass.setInputCloud(cloudIn);
     pass.setFilterFieldName("z");
     pass.setFilterLimits(-1, 1.5);       // TODO: Change this hand-coded values
     pass.setFilterLimitsNegative(true);  // Forget the unknown space between z=-1 and z=3
     pass.filter(*pclptr_unk_);

     // printf("Size after=%d", pclptr_unk_->points.size());
     if (pclptr_unk_->points.size() != 0)
     {
       kdtree_unk_.setInputCloud(pclptr_unk_);
       kdtree_unk_initialized_ = 1;
     }
   }
   else
   {  // when the drone is flying*/

  kdtree_unk_.setInputCloud(cloudIn);
  kdtree_unk_initialized_ = 1;
  //}
  mtx_unk.unlock();
  // printf("*************************OUT unkCB\n");
}

// TODO: check also against unkown space? Be careful because with the new points cloud I may have information of
// previously-unknown voxels
bool CVX::trajIsFree(Eigen::MatrixXd X)
{
  double t0coll = ros::Time::now().toSec();

  /*    if (flight_mode_.mode == flight_mode_.LAND || flight_mode_.mode == flight_mode_.TAKEOFF ||
        flight_mode_.mode == flight_mode_.NOT_FLYING || flight_mode_.mode == flight_mode_.INIT)*/

  if (flight_mode_.mode != flight_mode_.GO)
  {
    return true;
  }

  // printf("********In trajIsFree\n");
  // std::cout << X << std::endl;

  // printf("before ground\n");
  // TODO: maybe there is a more efficient way to do this (sampling only some points of X?)
  if (((X.col(2)).array() < 0).any() == true)  // If there is some z < 0. Note that in eigen, first_index=0
  {
    printf("Collision with the ground \n");
    // std::cout << X.col(3) << std::endl << std::endl;
    return false;  // There is a collision with the ground
  }
  // printf("later\n");

  int n = 1;  // Find nearest element

  Eigen::Vector3d eig_search_point(X(0, 0), X(0, 1), X(0, 2));
  pcl::PointXYZ pcl_search_point = eigenPoint2pclPoint(eig_search_point);
  double r = 100000;
  int last_i = 0;
  // printf("later2\n");

  // printf("later3\n");
  bool isfree = true;

  double t0mutex = ros::Time::now().toSec();
  mtx_map.lock();
  mtx_unk.lock();
  mtx_inst.lock();
  // printf("************Time waiting for mutex %0.2f ms\n", 1000 * (ros::Time::now().toSec() - t0mutex));
  // printf("*************************************\n");
  while (last_i < X.rows() - 1)
  {
    // printf("Inside the loop, last_i=%d\n", last_i);
    // last point clouds
    std::vector<int> id_inst(n);
    std::vector<float> dist2_inst(n);  // squared distance
    pcl_search_point = eigenPoint2pclPoint(eig_search_point);
    Eigen::Vector3d intersectionPoint;

    // printf("**********before the lock inst\n");

    // printf("after the lock inst\n");
    // printf("TisFree: MTX is locked\n");

    for (unsigned i = v_kdtree_new_pcls_.size() - 5; i < v_kdtree_new_pcls_.size() && i >= 0; i = i + 2)
    {
      if (i < 0)
      {
        continue;
      }
      // printf("Inside the loop\n");
      if (v_kdtree_new_pcls_[i].kdTree.nearestKSearch(pcl_search_point, n, id_inst, dist2_inst) > 0)
      {
        // printf("Below\n");
        if (sqrt(dist2_inst[0]) < r)
        {
          r = sqrt(dist2_inst[0]);
          pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr = v_kdtree_new_pcls_[i].kdTree.getInputCloud();
          intersectionPoint << ptr->points[id_inst[0]].x, ptr->points[id_inst[0]].y, ptr->points[id_inst[0]].z;
        }
        // r = std::min(r, sqrt(dist2_inst[0]));
      }
    }
    // printf("In 1\n");
    // double r_map = 100000;
    // occupied (map)
    std::vector<int> id_map(n);
    std::vector<float> dist2_map(n);  // squared distance

    if (kdtree_map_.nearestKSearch(pcl_search_point, n, id_map, dist2_map) > 0)
    {
      if (sqrt(dist2_map[0]) < r)
      {
        r = sqrt(dist2_map[0]);
        pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr = kdtree_map_.getInputCloud();
        intersectionPoint << ptr->points[id_map[0]].x, ptr->points[id_map[0]].y, ptr->points[id_map[0]].z;
      }
    }
    // printf("In 2\n");

    /*    printf("TisFree MTX is unlocked\n");
        mtx.unlock();*/

    // unknwown
    std::vector<int> id_unk(n);
    std::vector<float> dist2_unk(n);  // squared distance

    // printf("In 3\n");
    if (kdtree_unk_.nearestKSearch(pcl_search_point, n, id_unk, dist2_unk) > 0)
    {
      if (sqrt(dist2_unk[0]) < r)
      {
        r = sqrt(dist2_unk[0]);
        pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr = kdtree_unk_.getInputCloud();
        intersectionPoint << ptr->points[id_unk[0]].x, ptr->points[id_unk[0]].y, ptr->points[id_unk[0]].z;
      }
    }

    // printf("In 4\n");

    // Now r is the distance to the nearest obstacle (considering unknown space as obstacles)
    if (r < par_.drone_radius)  // || r_map < par_.inflation_jps
    {
      // printf("There is a collision with i=%d out of X.rows=%d\n", last_i, X.rows() - 1);
      if (par_.visual == true)
      {
        pubintersecPoint(intersectionPoint, true);
      }
      isfree = false;
      break;
    }

    // printf("In 5\n");
    // Find the next eig_search_point
    for (int i = last_i; i < X.rows(); i = i + 1)
    {
      last_i = i;
      Eigen::Vector3d traj_point(X(i, 0), X(i, 1), X(i, 2));
      if ((traj_point - eig_search_point).norm() > r)
      {  // There may be a collision here
        eig_search_point << X(i - 1, 0), X(i - 1, 1), X(i - 1, 2);
        break;
      }
    }
  }
  // printf("In 6\n");
  mtx_inst.unlock();
  mtx_map.unlock();
  mtx_unk.unlock();
  // printf("In 7\n");
  // printf("returning isFree=%d\n", isfree);
  // mtx.unlock();

  log_.coll_total_ms = log_.coll_total_ms + 1000 * (ros::Time::now().toSec() - t0coll);
  printf("Time in col checking: %f\n", 1000 * (ros::Time::now().toSec() - t0coll));
  log_.loops_col = log_.loops_col + 1;

  return isfree;  // It's free!
}

// Returns the first collision of JPS with the obstacles
double CVX::getDistanceToFirstCollisionJPSwithUnkonwnspace(vec_Vecf<3> path, bool* thereIsIntersection)
{
  if (kdtree_unk_initialized_ == false)
  {
    return par_.Ra;
  }
  vec_Vecf<3> original = path;
  // printf("*****ORIGINAL******");
  // printElementsOfJPS(original);

  Eigen::Vector3d first_element = path[0];
  Eigen::Vector3d last_search_point = path[0];
  Eigen::Vector3d inters = path[0];
  Eigen::Vector3d obst;  // intersection point
  pcl::PointXYZ pcl_search_point = eigenPoint2pclPoint(path[0]);

  double result;

  // occupied (map)
  int n = 1;
  std::vector<int> id_map(n);
  std::vector<float> dist2_map(n);  // squared distance
  double r = 1000000;
  // printElementsOfJPS(path);
  // printf("In 2\n");

  mtx_unk.lock();
  int el_eliminated = 0;  // number of elements eliminated
  while (path.size() > 0)
  {
    pcl_search_point = eigenPoint2pclPoint(path[0]);

    // printf("hola1\n");
    if (kdtree_unk_.nearestKSearch(pcl_search_point, n, id_map, dist2_map) > 0)
    {
      // printf("hola2\n");
      r = sqrt(dist2_map[0]);

      // if (r < par_.drone_radius)  // collision of the JPS path and an inflated obstacle --> take last search point
      if (r < 0.01)
      {
        // printf("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWw\n");
        pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr = kdtree_unk_.getInputCloud();
        obst << ptr->points[id_map[0]].x, ptr->points[id_map[0]].y, ptr->points[id_map[0]].z;
        // std::cout << "original=" << original[0].transpose() << std::endl;
        // std::cout << "obst=" << obst.transpose() << std::endl;
        result = (original[0] - obst).norm();
        *thereIsIntersection = true;

        break;
      }
      // printf("hola2\n");

      // Find the next eig_search_point
      int last_id;  // this is the last index inside the sphere

      bool no_points_outside_sphere = false;

      inters = getFirstIntersectionWithSphere(path, r, path[0], &last_id, &no_points_outside_sphere);
      // printf("**********Found it*****************\n");
      if (no_points_outside_sphere == true)
      {  // JPS doesn't intersect with any obstacle
        result = par_.Ra;

        break;
      }
      last_search_point = path[0];
      // Remove all the points of the path whose id is <= to last_id:
      // printf("last_id=%d\n", last_id);
      path.erase(path.begin(), path.begin() + last_id + 1);
      el_eliminated = el_eliminated + last_id;
      // and add the intersection as the first point of the path
      path.insert(path.begin(), inters);
    }
    else
    {
      // printf("JPS provided doesn't intersect any obstacles\n");
      // printf("Returning the first element of the path you gave me\n");
      result = par_.Ra;
      break;
    }
  }
  mtx_unk.unlock();

  // printf("returning l=%f\n", result);

  return result;
}

// Returns the first collision of JPS with the map (i.e. with the known obstacles). Note that JPS will collide with a
// map B if JPS was computed using an older map A
Eigen::Vector3d CVX::getFirstCollisionJPS(vec_Vecf<3> path, bool* thereIsIntersection, int& el_eliminated, int map)
{
  vec_Vecf<3> original = path;
  /*  printf("*****ORIGINAL******");
    printElementsOfJPS(original);*/
  // vec_Vecf<3> path_behind;
  // printf("In getFirstCollisionJPS\n");
  Eigen::Vector3d first_element = path[0];
  Eigen::Vector3d last_search_point = path[0];
  Eigen::Vector3d inters = path[0];
  Eigen::Vector3d n_obst;  // nearest obstacle
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

  el_eliminated = 0;  // number of elements eliminated
  while (path.size() > 0)
  {
    pcl_search_point = eigenPoint2pclPoint(path[0]);

    int number_of_neigh;

    double before = ros::Time::now().toSec();

    if (map == MAP)
    {
      number_of_neigh = kdtree_map_.nearestKSearch(pcl_search_point, n, id_map, dist2_map);
    }
    else  // map == UNKNOWN_MAP
    {
      number_of_neigh = kdtree_unk_.nearestKSearch(pcl_search_point, n, id_map, dist2_map);
    }

    printf("************NearestSearch: TotalTime= %0.2f ms\n", 1000 * (ros::Time::now().toSec() - before));

    if (number_of_neigh > 0)
    {
      r = sqrt(dist2_map[0]);

      if (r < par_.inflation_jps)  // collision of the JPS path and an inflated obstacle --> take last search point
      {
        // printf("Collision detected");  // We will return the search_point
        // pubJPSIntersection(inters);
        // inters = path[0];  // path[0] is the search_point I'm using.
        result = last_search_point;
        *thereIsIntersection = true;

        break;
      }

      // Find the next eig_search_point
      int last_id;  // this is the last index inside the sphere

      bool no_points_outside_sphere = false;

      inters = getFirstIntersectionWithSphere(path, r, path[0], &last_id, &no_points_outside_sphere);
      // printf("**********Found it*****************\n");
      if (no_points_outside_sphere == true)
      {  // JPS doesn't intersect with any obstacle
        // printf("JPS provided doesn't intersect any obstacles, returning the first element of the path you gave
        // me\n");
        result = first_element;
        break;
      }
      // printf("In 4\n");

      last_search_point = path[0];
      // Remove all the points of the path whose id is <= to last_id:
      path.erase(path.begin(), path.begin() + last_id + 1);
      el_eliminated = el_eliminated + last_id;
      // and add the intersection as the first point of the path
      path.insert(path.begin(), inters);
    }
    else
    {
      // printf("JPS provided doesn't intersect any obstacles, returning the first element of the path you gave me\n");
      result = first_element;
      break;
    }
  }
  mtx_map.unlock();
  mtx_unk.unlock();

  return result;
}

void CVX::pubJPSIntersection(Eigen::Vector3d inters)
{
  geometry_msgs::PointStamped p;
  p.header.frame_id = "world";
  p.point = eigen2point(inters);
  pub_jps_inters_.publish(p);
}

// TODO: the mapper receives a depth map and converts it to a point cloud. Why not receiving directly the point cloud?

// TODO: maybe clustering the point cloud is better for the potential field (instead of adding an obstacle in every
// point of the point cloud)

// g is the goal, x is the point on which I'd like to compute the force=-gradient(potential)
Eigen::Vector3d CVX::computeForce(Eigen::Vector3d x, Eigen::Vector3d g)
{
  // printf("In computeForce\n");

  double k_att = 2;
  double k_rep = 2;
  double d0 = 5;   // (m). Change between quadratic and linear potential (between linear and constant force)
  double rho = 5;  // (m). If the obstacle is further than rho, its f_rep=0

  Eigen::Vector3d f_rep(0, 0, 0);
  Eigen::Vector3d f_att(0, 0, 0);

  // Compute attractive force
  float d_goal = (g - x).norm();  // distance to the goal
  if (d_goal <= d0)
  {
    f_att = -k_att * (x - g);
  }
  else
  {
    f_att = -d0 * k_att * (x - g) / d_goal;
  }

  // Compute repulsive force
  std::vector<int> id;                 // pointIdxpar_.RadiusSearch
  std::vector<float> sd;               // pointpar_.RadiusSquaredDistance
  pcl::PointXYZ sp(x[0], x[1], x[2]);  // searchPoint=x
  mtx_map.lock();
  pcl::KdTree<pcl::PointXYZ>::PointCloudConstPtr cloud_ptr = kdtree_map_.getInputCloud();

  if (kdtree_map_.radiusSearch(sp, rho, id, sd) > 0)  // if further, f_rep=f_rep+0
  {
    for (size_t i = 0; i < id.size(); ++i)
    {
      if (cloud_ptr->points[id[i]].z < 0.2)  // Ground is not taken into account
      {
        continue;
      }
      Eigen::Vector3d obs(cloud_ptr->points[id[i]].x, cloud_ptr->points[id[i]].y, cloud_ptr->points[id[i]].z);
      double d_obst = sqrt(sd[i]);
      f_rep = f_rep + k_rep * (1 / d_obst - 1 / rho) * (pow(1 / d_obst, 2)) * (x - obs) / d_obst;
    }
  }
  mtx_map.unlock();

  visualization_msgs::MarkerArray forces;
  createForceArrow(x, f_att, f_rep, &forces);

  pub_forces_.publish(forces);
  return f_att + f_rep;
}

// x is the position where the force f=f_att+f_rep is
Eigen::Vector3d CVX::createForceArrow(Eigen::Vector3d x, Eigen::Vector3d f_att, Eigen::Vector3d f_rep,
                                      visualization_msgs::MarkerArray* forces)
{
  //  printf("In createForceArrow\n");

  const int ATT = 0;
  const int REP = 1;
  const int TOTAL = 2;
  for (int i = 0; i < 3; i++)  // Attractive, Repulsive, Total
  {
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.id = 100000000 + i;  // Large enough to not interfere with the arrows of the trajectories

    m.scale.x = 0.02;
    m.scale.y = 0.04;
    // m.scale.z = 1;

    float s = 2;  // scale factor
    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    geometry_msgs::Point p_start;
    p_start.x = x[0];
    p_start.y = x[1];
    p_start.z = x[2];
    geometry_msgs::Point p_end;
    switch (i)
    {
      case ATT:
        m.color = color(GREEN);
        p_end.x = x[0] + s * f_att[0];
        p_end.y = x[1] + s * f_att[1];
        p_end.z = x[2] + s * f_att[2];
        break;
      case REP:
        m.color = color(RED);
        p_end.x = x[0] + s * f_rep[0];
        p_end.y = x[1] + s * f_rep[1];
        p_end.z = x[2] + s * f_rep[2];
        break;
      case TOTAL:
        m.color = color(BLUE);
        p_end.x = x[0] + s * (f_att + f_rep)[0];
        p_end.y = x[1] + s * (f_att + f_rep)[1];
        p_end.z = x[2] + s * (f_att + f_rep)[2];
        break;
    }
    m.points.push_back(p_start);
    m.points.push_back(p_end);

    (*forces).markers.push_back(m);
  }
}

void CVX::pubActualTraj()
{
  // ROS_ERROR("In pubActualTraj\n");

  static geometry_msgs::Point p_last = pointOrigin();
  mtx_state.lock();
  Eigen::Vector3d act_pos(state_.pos.x, state_.pos.y, state_.pos.z);
  mtx_state.unlock();
  // mtx_term_goal.lock();
  Eigen::Vector3d t_goal = term_goal_;
  // mtx_term_goal.unlock();
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
  // mtx_term_goal.lock();
  p.point = eigen2point(term_goal_);
  // mtx_term_goal.unlock();

  pub_term_goal_.publish(p);
}

void CVX::pubPlanningVisual(Eigen::Vector3d center, double ra, double rb, Eigen::Vector3d B1, Eigen::Vector3d C1)
{
  visualization_msgs::MarkerArray tmp;

  int start = 2200;  // Large enough to prevent conflict with other markers
  visualization_msgs::Marker sphere_Sa;
  sphere_Sa.header.frame_id = "world";
  sphere_Sa.id = start;
  sphere_Sa.type = visualization_msgs::Marker::SPHERE;
  sphere_Sa.scale = vectorUniform(2 * ra);
  sphere_Sa.color = color(BLUE_TRANS);
  sphere_Sa.pose.position = eigen2point(center);
  tmp.markers.push_back(sphere_Sa);

  visualization_msgs::Marker sphere_Sb;
  sphere_Sb.header.frame_id = "world";
  sphere_Sb.id = start + 1;
  sphere_Sb.type = visualization_msgs::Marker::SPHERE;
  sphere_Sb.scale = vectorUniform(2 * rb);
  sphere_Sb.color = color(RED_TRANS_TRANS);
  sphere_Sb.pose.position = eigen2point(center);
  tmp.markers.push_back(sphere_Sb);

  visualization_msgs::Marker B1_marker;
  B1_marker.header.frame_id = "world";
  B1_marker.id = start + 2;
  B1_marker.type = visualization_msgs::Marker::SPHERE;
  B1_marker.scale = vectorUniform(0.1);
  B1_marker.color = color(BLUE_LIGHT);
  B1_marker.pose.position = eigen2point(B1);
  tmp.markers.push_back(B1_marker);

  visualization_msgs::Marker C1_marker;
  C1_marker.header.frame_id = "world";
  C1_marker.id = start + 3;
  C1_marker.type = visualization_msgs::Marker::SPHERE;
  C1_marker.scale = vectorUniform(0.1);
  C1_marker.color = color(BLUE_LIGHT);
  C1_marker.pose.position = eigen2point(C1);
  tmp.markers.push_back(C1_marker);

  pub_planning_vis_.publish(tmp);
}

void CVX::pubintersecPoint(Eigen::Vector3d p, bool add)
{
  static int i = 0;
  static int last_id = 0;
  int start = 300000;  // large enough to prevent conflict with others
  if (add)
  {
    visualization_msgs::Marker tmp;
    tmp.header.frame_id = "world";
    tmp.id = start + i;
    tmp.type = visualization_msgs::Marker::SPHERE;
    tmp.scale = vectorUniform(0.1);
    tmp.color = color(BLUE_LIGHT);
    tmp.pose.position = eigen2point(p);
    intersec_points_.markers.push_back(tmp);
    last_id = start + i;
    i = i + 1;
  }
  else
  {  // clear everything
    intersec_points_.markers.clear();
    /*    for (int j = start; j <= start; j++)
        {*/
    visualization_msgs::Marker tmp;
    tmp.header.frame_id = "world";
    tmp.id = start;
    tmp.type = visualization_msgs::Marker::SPHERE;
    tmp.action = visualization_msgs::Marker::DELETEALL;
    intersec_points_.markers.push_back(tmp);
    /*    }
     */
    last_id = 0;
    i = 0;
  }
  pub_intersec_points_.publish(intersec_points_);
  if (!add)
  {
    intersec_points_.markers.clear();
  }
}

// If you want the force to be the direction selector
// Eigen::Vector3d force = computeForce(curr_pos, term_goal);
// double x = force[0], y = force[1], z = force[2];
// If you want the JPS3D solution to be the direction selector
// double x = directionJPS_[0], y = directionJPS_[1], z = directionJPS_[2];

/*      // Solver VEL
      double xf_sphere[3] = { p2[0], p2[1], p2[2] };
      mtx_goals.lock();
      double x0[3] = { initialCond_.pos.x, initialCond_.pos.y, initialCond_.pos.z };
      //double u0[3] = { initialCond_.vel.x, initialCond_.vel.y, initialCond_.vel.z };
      mtx_goals.unlock();
      solver_vel_.set_xf(xf_sphere);
      solver_vel_.set_x0(x0);
      printf("x0 is %f, %f, %f\n", x0[0], x0[1], x0[2]);
      printf("xf is %f, %f, %f\n", xf_sphere[0], xf_sphere[1], xf_sphere[2]);
      double max_values[1] = { par_.v_max };
      solver_vel_.set_max(max_values);
      //solver_vel_.set_u0(u0);
      solver_vel_.genNewTraj();
      printf("generated new traj\n");
      U_temp_ = solver_vel_.getU();
      printf("got U\n");
      X_temp_ = solver_vel_.getX();
      printf("X_temp=\n");
      std::cout << X_temp_ << std::endl;*/

/*      // Solver ACCEL
      double xf_sphere[6] = { p2[0], p2[1], p2[2], 0, 0, 0 };
      mtx_goals.lock();
      double x0[6] = { initialCond_.pos.x, initialCond_.pos.y, initialCond_.pos.z,
                       initialCond_.vel.x, initialCond_.vel.y, initialCond_.vel.z };
      //double u0[3] = { initialCond_.accel.x, initialCond_.accel.y, initialCond_.accel.z };
      mtx_goals.unlock();
      solver_accel_.set_xf(xf_sphere);
      solver_accel_.set_x0(x0);
      double max_values[2] = { par_.v_max, par_.a_max };
      solver_accel_.set_max(max_values);
      //solver_accel_.set_u0(u0);
      solver_accel_.genNewTraj();
      U_temp_ = solver_accel_.getU();
      X_temp_ = solver_accel_.getX();*/

/*      printf("(Optimizando desde): %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", initialCond_.pos.x,
   initialCond_.pos.y, initialCond_.pos.z, initialCond_.vel.x, initialCond_.vel.y, initialCond_.vel.z);

      printf("(State): %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", state_.pos.x, state_.pos.y, state_.pos.z,
             state_.vel.x, state_.vel.y, state_.vel.z);*/

// printf("Current state=%0.2f, %0.2f, %0.2f\n", state_pos[0], state_pos[1], state_pos[2]);
// printf("Before going to get C1, Points in JPS1\n");
/*        for (int i = 0; i < path_jps_vector_.size(); i++)
        {
          std::cout << path_jps_vector_[i].transpose() << std::endl;
        }*/

// P1-P2 is the direction used for projection. P2 is the gal clicked
Eigen::Vector3d CVX::projectClickedGoal(Eigen::Vector3d& P1)
{
  //[px1, py1, pz1] is inside the map (it's the center of the map, where the drone is)
  //[px2, py2, pz2] is outside the map
  mtx_term_term_goal.lock();
  Eigen::Vector3d P2 = term_term_goal_;
  mtx_term_term_goal.unlock();
  // return P2;  // TODO: Comment this line after the HW experiments!
  double x_max = P1[0] + par_.wdx / 2;
  double x_min = P1[0] - par_.wdx / 2;
  double y_max = P1[1] + par_.wdy / 2;
  double y_min = P1[1] - par_.wdy / 2;
  double z_max = P1[2] + par_.wdz / 2;
  double z_min = P1[2] - par_.wdz / 2;

  if ((P2[0] < x_max && P2[0] > x_min) && (P2[1] < y_max && P2[1] > y_min) && (P2[2] < z_max && P2[2] > z_min))
  {
    // Clicked goal is inside the map
    return P2;
  }
  Eigen::Vector3d inters;
  std::vector<Eigen::Vector4d> all_planes = {
    Eigen::Vector4d(1, 0, 0, -x_max),  // Plane X right
    Eigen::Vector4d(-1, 0, 0, x_min),  // Plane X left
    Eigen::Vector4d(0, 1, 0, -y_max),  // Plane Y right
    Eigen::Vector4d(0, -1, 0, y_min),  // Plane Y left
    Eigen::Vector4d(0, 0, 1, -z_max),  // Plane Z up
    Eigen::Vector4d(0, 0, -1, z_min)   // Plane Z down
  };

  /*  std::cout << "The planes" << std::endl;
    for (int i = 0; i < 6; i++)
    {
      std::cout << all_planes[i].transpose() << std::endl;
    }*/
  int axis;  // 1 is x, 2 is y, 3 is z
  for (int i = 0; i < 6; i++)
  {
    if (getIntersectionWithPlane(P1, P2, all_planes[i], &inters) == true)
    {
      axis = (int)(i / 2);
      int N = 1;
      pcl::PointXYZ searchPoint(inters[0], inters[1], inters[2]);
      std::vector<int> id(N);
      std::vector<float> pointNKNSquaredDistance(N);

      // intersectionPoint << ptr->points[id_map[0]].x, ptr->points[id_map[0]].y, ptr->points[id_map[0]].z;

      // Let's find now the nearest free or unkown point to the intersection
      if (kdtree_frontier_initialized_)
      {
        mtx_frontier.lock();
        pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr = kdtree_frontier_.getInputCloud();
        if (kdtree_frontier_.nearestKSearch(searchPoint, N, id, pointNKNSquaredDistance) > 0)
        {
          inters = Eigen::Vector3d(ptr->points[id[0]].x, ptr->points[id[0]].y, ptr->points[id[0]].z);

          // printf("Found nearest neighbour\n");
        }
        mtx_frontier.unlock();
        pubTerminalGoal();
      }
      else
      {
        printf("Run the mapper, returning Clicked Goal\n");
        return P2;
      }

      // Now let's put the intersection 1.8 * par_.inflation_jps meters away from the end of the map.
      Eigen::Vector3d sign(inters[0] / fabs(inters[0]), inters[1] / fabs(inters[1]), inters[2] / fabs(inters[2]));

      // std::cout << "sign=" << sign.transpose() << std::endl;

      inters[0] = (axis == 0) ? inters[0] + sign[0] * 1.5 * par_.inflation_jps : inters[0];
      inters[1] = (axis == 1) ? inters[1] + sign[1] * 1.5 * par_.inflation_jps : inters[1];
      inters[2] = (axis == 2) ? inters[2] + sign[2] * 1.5 * par_.inflation_jps : inters[2];

      // inters = inters + sign * 1.5 * par_.inflation_jps;

      return inters;
    }
  }
  printf("Neither the goal is inside the map nor it has a projection into it, this is impossible");
}
