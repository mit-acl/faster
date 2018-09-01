// Authors: Jesus Tordesillas and Brett T. Lopez
// Date: August 2018

// TODO: compile cvxgen with the option -03 (see
// https://stackoverflow.com/questions/19689014/gcc-difference-between-o3-and-os
// and
// https://cvxgen.com/docs/c_interface.html    )

// TODO: update gcc to the latest version (see https://cvxgen.com/docs/c_interface.html)

// TODO: use the gpu versions of the pcl functions
// TODO: https://eigen.tuxfamily.org/dox/TopicCUDA.html

// TODO: how to tackle unknown space

// TODO: Quiz'a puedo anadir al potential field otra fuerza que me aleje de los sitios por los cuales ya he pasado?

// TODO: Check the offset or offset-1

// TODO: I'm using only the direction of the force, but not the magnitude. Could I use the magnitude?
// TODO: If I'm only using the direction of the force, not sure if quadratic+linear separation is needed in the
// attractive force

#include "cvx.hpp"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/MarkerArray.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/filter.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>

#define OFFSET                                                                                                         \
  10  // Replanning offset (the initial conditions are taken OFFSET states farther from the last published goal)

#define Ra 4.0      // [m] Radius of the first sphere
#define Rb 6.0      // [m] Radius of the second sphere
#define W_MAX 1     // [rd/s] Maximum angular velocity
#define alpha_0 1   //[rd] threshold to ignore current JPS solution, and consider the old one
#define Z_ground 0  //[m] points below this are considered ground

using namespace JPS;

CVX::CVX(ros::NodeHandle nh, ros::NodeHandle nh_replan_CB, ros::NodeHandle nh_pub_CB)
  : nh_(nh), nh_replan_CB_(nh_replan_CB), nh_pub_CB_(nh_pub_CB)
{
  optimized_ = false;
  flight_mode_.mode = flight_mode_.NOT_FLYING;

  pub_goal_ = nh_.advertise<acl_msgs::QuadGoal>("goal", 1);
  pub_traj_ = nh_.advertise<nav_msgs::Path>("traj", 1);
  pub_setpoint_ = nh_.advertise<visualization_msgs::Marker>("setpoint", 1);
  pub_trajs_sphere_ = nh_.advertise<visualization_msgs::MarkerArray>("trajs_sphere", 1);
  pub_forces_ = nh_.advertise<visualization_msgs::MarkerArray>("forces", 1);
  pub_actual_traj_ = nh_.advertise<visualization_msgs::Marker>("actual_traj", 1);
  pub_path_jps1_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps1", 1);
  pub_path_jps2_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps2", 1);

  pub_intersec_points_ = nh_.advertise<visualization_msgs::MarkerArray>("intersection_points", 1);

  pub_planning_vis_ = nh_.advertise<visualization_msgs::MarkerArray>("planning_vis", 1);

  sub_goal_ = nh_.subscribe("term_goal", 1, &CVX::goalCB, this);
  sub_mode_ = nh_.subscribe("flightmode", 1, &CVX::modeCB, this);
  sub_state_ = nh_.subscribe("state", 1, &CVX::stateCB, this);
  sub_map_ = nh_.subscribe("occup_grid", 1, &CVX::mapCB, this);
  sub_unk_ = nh_.subscribe("unknown_grid", 1, &CVX::unkCB, this);
  sub_pcl_ = nh_.subscribe("pcloud", 1, &CVX::pclCB, this);

  pubCBTimer_ = nh_pub_CB_.createTimer(ros::Duration(DC), &CVX::pubCB, this);

  replanCBTimer_ = nh_replan_CB.createTimer(ros::Duration(2 * DC), &CVX::replanCB, this);

  bool ff;
  ros::param::param<bool>("~use_ff", ff, false);
  ros::param::param<double>("~u_min", u_min_, 0.2);
  ros::param::param<double>("~u_max", u_max_, 1.0);
  ros::param::param<double>("~z_land", z_land_, 0.05);
  ros::param::param<double>("cntrl/spinup_time", spinup_time_, 0.5);

  if (ff)
    use_ff_ = 1;
  else
    use_ff_ = 0;

  // Initialize setpoint marker
  setpoint_.header.frame_id = "world";
  setpoint_.id = 0;
  setpoint_.type = visualization_msgs::Marker::SPHERE;
  setpoint_.scale.x = 0.35;
  setpoint_.scale.y = 0.35;
  setpoint_.scale.z = 0.35;
  setpoint_.color = color(ORANGE_TRANS);

  term_goal_.pos = vectorNull();

  quadGoal_.pos = vectorNull();
  quadGoal_.vel = vectorNull();
  quadGoal_.accel = vectorNull();
  quadGoal_.jerk = vectorNull();

  initialCond_.pos = vectorNull();
  initialCond_.vel = vectorNull();
  initialCond_.accel = vectorNull();
  initialCond_.jerk = vectorNull();

  markerID_ = 0;

  cells_x_ = (int)WDX / RES;
  cells_y_ = (int)WDY / RES;
  cells_z_ = (int)WDZ / RES;

  // pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
  pclptr_map_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pclptr_unk_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  name_drone_ = ros::this_node::getNamespace();
  name_drone_.erase(0, 2);  // Erase slashes

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

void CVX::publishJPSPath(vec_Vecf<3> path, int i)
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

bool CVX::solveJPS3D(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr, Vec3f start, Vec3f goal, vec_Vecf<3>& path)
{
  // Create a map
  /*  std::cout << "Solving JPS from start\n" << start << std::endl;
    std::cout << "To goal\n" << goal << std::endl;*/
  Vec3i dim(cells_x_, cells_y_, cells_z_);  //  number of cells in each dimension
  Vec3f center_map = start;                 // position of the drone
  // Read the pointcloud

  MapReader<Vec3i, Vec3f> reader(pclptr, dim, RES, center_map, Z_ground);  // Map read

  std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();
  map_util->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());

  std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(false));  // Declare a planner

  planner_ptr->setMapUtil(map_util);  // Set collision checking function
  planner_ptr->updateMap();

  Timer time_jps(true);
  bool valid_jps = planner_ptr->plan(
      start, goal, 1, true);  // Plan from start to goal with heuristic weight=1, and using JPS (if false --> use A*)

  if (valid_jps == true)  // There is a solution
  {
    double dt_jps = time_jps.Elapsed().count();
    // printf("JPS Planner takes: %f ms\n", dt_jps);
    // printf("JPS Path Distance: %f\n", total_distance3f(planner_ptr->getPath()));  // getRawPath() if you want the
    // path with more corners (not "cleaned")
    // printf("JPS Path: \n");
    path.clear();
    path = planner_ptr->getPath();  // getRawPath() if you want the path with more corners (not "cleaned")

    /*    printf("First point in path_jps_vector_:\n");
        std::cout << path_jps_vector_[0].transpose() << std::endl;*/
    // directionJPS_ = path_jps_vector_[1] - path_jps_vector_[0];
    // printf("Estoy aqui: \n");
    /*    for (const auto& it : path_jps_vector)
        {
          std::cout << it.transpose() << std::endl;
        }*/
  }
  /*
 Timer time_astar(true);
 bool valid_astar = planner_ptr->plan(start, goal, 1, false);  // Plan from start to goal using A*
 double dt_astar = time_astar.Elapsed().count();
 printf("AStar Planner takes: %f ms\n", dt_astar);
 printf("AStar Path Distance: %f\n", total_distance3f(planner_ptr->getRawPath()));
 printf("AStar Path: \n");
 auto path_astar = planner_ptr->getRawPath();
 for (const auto& it : path_astar)
   std::cout << it.transpose() << std::endl;
*/
  // printf("Out of solveJPSD\n");
  return valid_jps;
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
                                      std_msgs::ColorRGBA color)
{
  // printf("In vectorOfVectors2MarkerArray\n");
  geometry_msgs::Point p_last = eigen2point(traj[0]);

  bool skip = false;
  int i = 50000;  // large enough to prevent conflict with other markers

  for (const auto& it : traj)
  {
    i++;
    if (skip)  // skip the first element
    {
      skip = true;
      continue;
    }

    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.id = i;
    m.color = color;
    m.scale.x = 0.02;
    m.scale.y = 0.04;
    // m.scale.z = 1;

    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    geometry_msgs::Point p = eigen2point(it);
    /*    p.x = traj[i][0];
        p.y = X(i, 1);
        p.z = X(i, 2);*/
    m.points.push_back(p_last);
    m.points.push_back(p);
    // std::cout << "pushing marker\n" << m << std::endl;
    (*m_array).markers.push_back(m);
    p_last = p;
  }
}

void CVX::goalCB(const acl_msgs::TermGoal& msg)
{
  printf("NEW GOAL************************************************\n");
  term_goal_ = msg;

  status_ = (status_ == GOAL_REACHED) ? YAWING : TRAVELING;
  if (status_ == YAWING)
  {
    printf("GCB: status_ = YAWING\n");
  }
  if (status_ == TRAVELING)
  {
    printf("GCB: status_ = TRAVELING\n");
  }

  planner_status_ = START_REPLANNING;
  force_reset_to_0_ = true;
  // printf("GCB: planner_status_ = START_REPLANNING\n");
  goal_click_initialized_ = true;
  clearMarkerActualTraj();
  // printf("Exiting from goalCB\n");
}

void CVX::yaw(double diff, acl_msgs::QuadGoal& quad_goal)
{
  saturate(diff, -DC * W_MAX, DC * W_MAX);
  if (diff > 0)
    quad_goal.dyaw = W_MAX;
  else
    quad_goal.dyaw = -W_MAX;
  quad_goal.yaw += diff;
}

void CVX::replanCB(const ros::TimerEvent& e)
{
  // printf("In replanCB\n");
  // double t0replanCB = ros::Time::now().toSec();
  Eigen::Vector3d state_pos(state_.pos.x, state_.pos.y, state_.pos.z);  // Local copy of state

  clearMarkerSetOfArrows();
  pubintersecPoint(Eigen::Vector3d::Zero(), false);  // Clear the intersection points markers
  if (!kdtree_map_initialized_ && !kdtree_unk_initialized_)
  {
    ROS_WARN("Run the mapper");
    return;
  }
  if (!goal_click_initialized_)
  {
    ROS_WARN("Click a goal to start replanning");
    return;
  }

  Eigen::Vector3d term_goal(term_goal_.pos.x, term_goal_.pos.y, term_goal_.pos.z);  // Local copy of the terminal goal
  double dist_to_goal = (term_goal - state_pos).norm();

  // 0.96 and 0.98 are to ensure that ra<rb<dist_to_goal always
  double ra = std::min(0.96 * dist_to_goal, Ra);  // radius of the sphere Sa
  double rb = std::min(0.98 * dist_to_goal, Rb);  // radius of the sphere Sb

  /*  std::cout << "rb=" << rb << std::endl;
    std::cout << "dist_to_goal=" << dist_to_goal << std::endl;*/

  if (dist_to_goal < GOAL_RADIUS && status_ != GOAL_REACHED)
  {
    status_ = GOAL_REACHED;
    printf("STATUS=GOAL_REACHED\n");
  }
  // printf("Entering in replanCB, planner_status_=%d\n", planner_status_);
  if (status_ == GOAL_SEEN || status_ == GOAL_REACHED || planner_status_ == REPLANNED || status_ == YAWING)
  {
    // printf("No replanning needed because planner_status_=%d\n", planner_status_);
    return;
  }

  ///////////////////////////////////////////////////////////////////////////////

  bool have_seen_the_goal1 = false, have_seen_the_goal2 = false;
  bool found_one_1 = false, found_one_2 = false;  // found at least one free trajectory
  bool need_to_decide = true;
  bool solvedjps1 = false, solvedjps2 = false;

  double J1 = 0, J2 = 0, JPrimj1 = 0, JPrimj2 = 0, JPrimv1 = 0, JPrimv2 = 0, JDist1 = 0, JDist2 = 0;
  J1 = std::numeric_limits<double>::max();
  J2 = std::numeric_limits<double>::max();
  Eigen::MatrixXd U_temp1, U_temp2, X_temp1, X_temp2;

  // printf("hola1\n");

  static Eigen::Vector3d B1km1;  // B1km1 is B1 in t=t_k-1 (previous iteration)
  Eigen::Vector3d B1, B2, C1, C2;
  vec_Vecf<3> WP1, WP2;

  // printf("hola2\n");

  vec_Vecf<3> null1(1, Eigen::Vector3d::Zero());
  vec_Vecf<3> null2(1, Eigen::Vector3d::Zero());
  vec_Vecf<3>& JPS1(null1);  // references HAVE to be initialized
  vec_Vecf<3>& JPS2(null2);
  static bool first_time = true;                                     // how many times I've solved JPS1
  solvedjps1 = solveJPS3D(pclptr_map_, state_pos, term_goal, JPS1);  // Solution is in JPS1
  JPS1[JPS1.size() - 1] = term_goal;  // JPS ends in the voxel of the goal, but not exactly in the goal--> force it;

  /*  printf("Points in JPS1 before\n");
    for (int i = 0; i < JPS1.size(); i++)
    {
      std::cout << JPS1[i].transpose() << std::endl;
    }*/

  if (first_time == true)
  {  // B1km1 is still not initialized --> Run from
    first_time = false;
    solvedjps2 = solvedjps1;
    JPS2 = JPS1;
    solvedjps2 = true;  // only the first time
    // printf("I'm in first time\n");
  }
  else
  {
    // In general, solve from B1km1
    // std::cout << "B1km1 before is" << B1km1 << std::endl;
    solvedjps2 = solveJPS3D(pclptr_map_, B1km1, term_goal, JPS2);  // Solution is in JPS2
    JPS2[JPS2.size() - 1] = term_goal;  // JPS ends in the voxel of the goal, but not exactly in the goal--> force it;
  }

  /*  if (solvedjps1 == false)
    {
      printf("JPS1 didn't find a solution\n");
      return;
    }
    if (solvedjps2 == false)
    {
      printf("JPS2 didn't find a solution");
      return;
    }*/
  // std::cout << "state" << state_pos << std::endl;
  /*  printf("Points in JPS1 despues\n");
    for (int i = 0; i < JPS1.size(); i++)
    {
      std::cout << JPS1[i].transpose() << std::endl;
    }*/

  if (solvedjps1 == true)
  {
    // printf("clearing\n");
    clearJPSPathVisualization(1);
    // printf("publishing");
    publishJPSPath(JPS1, 1);
    // printf("after\n");
    B1 = getFirstIntersectionWithSphere(JPS1, ra, state_pos);
    B1km1 = B1;
  }

  if (solvedjps2 == true)
  {
    clearJPSPathVisualization(2);
    publishJPSPath(JPS2, 2);
    B2 = getFirstIntersectionWithSphere(JPS2, ra, state_pos);
  }

  if (solvedjps1 == false || solvedjps2 == false)
  {
    printf("JPS didn't find a solution\n");
    return;
  }

  // std::cout << "B1km1 after is" << B1km1 << std::endl;

  printf("hola4\n");

  std::vector<Eigen::Vector3d> K = samplePointsSphere(B1, ra, state_pos);  // radius, center and point

  for (int i = 0; i < K.size(); i++)
  {
    printf("hola5\n");
    if (optimized_)  // Needed to skip the first time (X_ still not initialized)
    {
      k_initial_cond_ = std::min(k_ + OFFSET, (int)(X_.rows() - 1));
      updateInitialCond(k_initial_cond_);
    }
    //////SOLVING FOR JERK
    Eigen::Vector3d p = K[i];
    double xf[9] = { p[0], p[1], p[2], 0, 0, 0, 0, 0, 0 };
    mtx_goals.lock();
    double x0[9] = { initialCond_.pos.x,   initialCond_.pos.y,   initialCond_.pos.z,
                     initialCond_.vel.x,   initialCond_.vel.y,   initialCond_.vel.z,
                     initialCond_.accel.x, initialCond_.accel.y, initialCond_.accel.z };
    mtx_goals.unlock();
    solver_jerk_.set_xf(xf);
    solver_jerk_.set_x0(x0);
    // printf("hola6\n");
    double max_values[3] = { V_MAX, A_MAX, J_MAX };
    solver_jerk_.set_max(max_values);
    solver_jerk_.genNewTraj();
    U_temp1 = solver_jerk_.getU();
    X_temp1 = solver_jerk_.getX();
    //////SOLVED FOR JERK
    printf("hola7\n");

    bool isFree = trajIsFree(X_temp1);
    createMarkerSetOfArrows(X_temp1, isFree);

    printf("hola8\n");

    if (isFree)
    {
      found_one_1 = true;
      double dist = (term_goal - p).norm();
      have_seen_the_goal1 = (dist < GOAL_RADIUS) ? true : false;

      C1 = getLastIntersectionWithSphere(JPS1, rb, state_pos, &JDist1);
      pubPlanningVisual(state_pos, ra, rb, B1, C1);

      /*      printf("Angle=%f\n", angleBetVectors(B1 - state_pos, B2 - state_pos));
            std::cout << "uno\n" << B1 - state_pos << std::endl;
            std::cout << "dos\n" << B2 - state_pos << std::endl;*/

      if (angleBetVectors(B1 - state_pos, B2 - state_pos) <= alpha_0)
      {
        need_to_decide = false;
      }
      break;
    }
  }

  printf("hola9\n");
  //  if (need_to_decide == true && have_seen_the_goal1 == false)
  if (1)
  {
    printf("**************Computing costs to decide between 2 jerk trajectories\n");
    if (found_one_1)
    {
      printf("hola10\n");
      JPrimj1 = solver_jerk_.getCost();
      printf("hola11\n");
      WP1 = getPointsBw2Spheres(JPS1, ra, rb, state_pos);
      WP1.insert(WP1.begin(), B1);
      printf("hola12\n");
      WP1.push_back(C1);
      JPrimv1 = solveVelAndGetCost(WP1);
      printf("hola13\n");
      J1 = JPrimj1 + JPrimv1 + JDist1;
    }
    printf("hola14\n");
    K = samplePointsSphere(B2, ra, state_pos);  // radius, center and point
    printf("hola15\n");
    for (int i = 0; i < K.size(); i++)
    {
      printf("hola16\n");
      if (optimized_)  // Needed to skip the first time (X_ still not initialized)
      {
        printf("hola16.5\n");
        k_initial_cond_ = std::min(k_ + OFFSET, (int)(X_.rows() - 1));
        printf("hola16.7\n");
        updateInitialCond(k_initial_cond_);
        printf("hola16.8\n");
      }

      printf("hola17\n");
      //////SOLVING FOR JERK
      Eigen::Vector3d p = K[i];
      double xf[9] = { p[0], p[1], p[2], 0, 0, 0, 0, 0, 0 };
      mtx_goals.lock();
      double x0[9] = { initialCond_.pos.x,   initialCond_.pos.y,   initialCond_.pos.z,
                       initialCond_.vel.x,   initialCond_.vel.y,   initialCond_.vel.z,
                       initialCond_.accel.x, initialCond_.accel.y, initialCond_.accel.z };
      mtx_goals.unlock();
      solver_jerk_.set_xf(xf);
      solver_jerk_.set_x0(x0);
      double max_values[3] = { V_MAX, A_MAX, J_MAX };
      solver_jerk_.set_max(max_values);
      solver_jerk_.genNewTraj();
      U_temp2 = solver_jerk_.getU();
      X_temp2 = solver_jerk_.getX();
      //////SOLVED FOR JERK

      printf("hola18\n");
      bool isFree = trajIsFree(X_temp1);
      createMarkerSetOfArrows(X_temp1, isFree);
      if (isFree)
      {
        printf("hola19\n");
        found_one_2 = true;
        double dist = (term_goal - p).norm();
        have_seen_the_goal1 = (dist < GOAL_RADIUS) ? true : false;

        C2 = getLastIntersectionWithSphere(JPS2, rb, state_pos, &JDist2);
        pubPlanningVisual(state_pos, ra, rb, B2, C2);
        printf("hola20\n");
        pub_trajs_sphere_.publish(trajs_sphere_);
        printf("hola21\n");
        JPrimj2 = solver_jerk_.getCost();
        WP2 = getPointsBw2Spheres(JPS2, ra, rb, state_pos);
        WP2.insert(WP2.begin(), B2);
        WP2.push_back(C2);
        JPrimv2 = solveVelAndGetCost(WP2);
        J2 = JPrimj2 + JPrimv2 + JDist2;

        printf("  JPrimj1    JPrimj2    JPrimv1    JPrimv2    JDista1    JDista2  \n");
        printf("%10.1f,%10.1f,%10.1f,%10.1f,%10.1f,%10.1f\n", JPrimj1, JPrimj2, JPrimv1, JPrimv2, JDist1, JDist2);

        break;
      }
    }
  }
  pub_trajs_sphere_.publish(trajs_sphere_);

  if (found_one_1 == false && found_one_2 == false)  // J1=J2=infinity
  {
    ROS_ERROR("Unable to find a free traj");
    return;
  }

  ////DECISION: Choose 1 or 2?
  if (have_seen_the_goal1 || have_seen_the_goal2)
  {
    status_ = GOAL_SEEN;  // I've found a free path that ends in the goal
    printf("CHANGED TO GOAL_SEEN********\n");
  }

  if (have_seen_the_goal1 || !need_to_decide)
  {
    U_temp_ = U_temp1;
    X_temp_ = X_temp1;
  }
  else if (have_seen_the_goal2)
  {
    U_temp_ = U_temp2;
    X_temp_ = X_temp2;
  }
  else  // I reach this point also when need_to_decide==false, and noone has seen the goal
  {
    U_temp_ = (J1 <= J2) ? U_temp1 : U_temp2;
    X_temp_ = (J1 <= J2) ? X_temp1 : X_temp2;
  }

  optimized_ = true;
  planner_status_ = REPLANNED;
  printf("ReplanCB: planner_status_ = REPLANNED\n");
  pubTraj(X_temp_);
  printf("replanCB finished\n");
  // ROS_WARN("solve time: %0.2f ms", 1000 * (ros::Time::now().toSec() - then));
  // printf("Time in replanCB %0.2f ms\n", 1000 * (ros::Time::now().toSec() - t0replanCB));
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
    double max_values[1] = { V_MAX };
    solver_vel_.set_max(max_values);
    // solver_vel_.set_u0(u0);
    solver_vel_.genNewTraj();
    cost = cost + solver_vel_.getCost();
  }
  return cost;
}

void CVX::modeCB(const acl_msgs::QuadFlightMode& msg)
{
  // printf("In modeCB\n");
  if (msg.mode == msg.LAND && flight_mode_.mode != flight_mode_.LAND)
  {
    // Solver Vel
    /*    double xf[6] = { quadGoal_.pos.x, quadGoal_.pos.y, z_land_ };
        double max_values[1] = { V_MAX };
        solver_vel_.set_max(max_values);  // TODO: To land, I use u_min_
        solver_vel_.set_xf(xf);
        solver_vel_.genNewTraj();*/

    // Solver Accel
    /*    double xf[6] = { quadGoal_.pos.x, quadGoal_.pos.y, z_land_, 0, 0, 0 };
        double max_values[2] = { V_MAX, A_MAX };
        solver_accel_.set_max(max_values);  // TODO: To land, I use u_min_
        solver_accel_.set_xf(xf);
        solver_accel_.genNewTraj();*/

    // Solver Jerk
    double xf[9] = { quadGoal_.pos.x, quadGoal_.pos.y, z_land_, 0, 0, 0, 0, 0, 0 };
    double max_values[3] = { V_MAX, A_MAX, J_MAX };
    solver_jerk_.set_max(max_values);  // TODO: To land, I use u_min_
    solver_jerk_.set_xf(xf);
    solver_jerk_.genNewTraj();
  }
  flight_mode_.mode = msg.mode;
}

void CVX::stateCB(const acl_msgs::State& msg)
{
  // printf("(State): %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", msg.pos.x, msg.pos.y, msg.pos.z, msg.vel.x, msg.vel.y,
  //       msg.vel.z);
  state_ = msg;
  // Stop updating when we get GO
  if (flight_mode_.mode == flight_mode_.NOT_FLYING || flight_mode_.mode == flight_mode_.KILL)
  {
    quadGoal_.pos = msg.pos;
    quadGoal_.vel = msg.vel;
    z_start_ = msg.pos.z;
    z_start_ = std::max(0.0, z_start_);
  }

  if (status_ != GOAL_REACHED)
  {
    pubActualTraj();
  }
}

void CVX::updateInitialCond(int i)
{
  if (status_ != GOAL_REACHED)
  {
    initialCond_.pos = getPos(i);
    initialCond_.vel = getVel(i);
    initialCond_.accel = (use_ff_) ? getAccel(i) : vectorNull();
    initialCond_.jerk = (use_ff_) ? getJerk(i) : vectorNull();
  }

  else
  {
    initialCond_.pos = state_.pos;
    initialCond_.vel = vectorNull();
    initialCond_.accel = vectorNull();
    initialCond_.jerk = vectorNull();
  }
}

void CVX::pubCB(const ros::TimerEvent& e)
{
  mtx_goals.lock();

  if (flight_mode_.mode == flight_mode_.LAND)
  {
    double d = sqrt(pow(quadGoal_.pos.z - z_land_, 2));
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

  initialCond_.vel = vectorNull();
  initialCond_.accel = vectorNull();
  initialCond_.jerk = vectorNull();

  // printf("In pubCB3\n");
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
  // printf("In pubCB4\n");
  // static int k = 0;
  if (optimized_ && flight_mode_.mode != flight_mode_.NOT_FLYING && flight_mode_.mode != flight_mode_.KILL)
  {
    quadGoal_.cut_power = false;

    if ((planner_status_ == REPLANNED && (k_ == k_initial_cond_)) ||
        (force_reset_to_0_ && planner_status_ == REPLANNED))
    {
      force_reset_to_0_ = false;
      X_ = X_temp_;
      U_ = U_temp_;
      k_ = 0;  // Start again publishing the waypoints in X_ from the first row

      planner_status_ = START_REPLANNING;
      // printf("pucCB2: planner_status_=START_REPLANNING\n");
    }

    k_ = std::min(k_, (int)(X_.rows() - 1));
    int kp1 = std::min(k_ + OFFSET, (int)(X_.rows() - 1));  // k plus offset

    quadGoal_.pos = getPos(k_);
    quadGoal_.vel = getVel(k_);
    quadGoal_.accel = (use_ff_) ? getAccel(k_) : vectorNull();
    quadGoal_.jerk = (use_ff_) ? getJerk(k_) : vectorNull();
    quadGoal_.dyaw = 0;

    // heading_ = atan2(goal_(1) - X_(0, 1), goal_(0) - X_(0, 0));

    if (status_ == YAWING)
    {
      double desired_yaw = atan2(term_goal_.pos.y - quadGoal_.pos.y, term_goal_.pos.x - quadGoal_.pos.x);
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

    if (status_ == TRAVELING || status_ == GOAL_SEEN)
    {
      double desired_yaw = atan2(quadGoal_.vel.y, quadGoal_.vel.x);
      double diff = desired_yaw - quadGoal_.yaw;
      angle_wrap(diff);
      yaw(diff, quadGoal_);
    }

    k_++;
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
  tmp.x = X_(i, 0);
  tmp.y = X_(i, 1);
  tmp.z = X_(i, 2);
  return tmp;
}

geometry_msgs::Vector3 CVX::getVel(int i)
{
  int input_order = solver_jerk_.getOrder();
  geometry_msgs::Vector3 tmp;
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
  return tmp;
}

geometry_msgs::Vector3 CVX::getAccel(int i)
{
  int input_order = solver_jerk_.getOrder();
  geometry_msgs::Vector3 tmp;

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

  return tmp;
}

geometry_msgs::Vector3 CVX::getJerk(int i)
{
  int input_order = solver_jerk_.getOrder();
  geometry_msgs::Vector3 tmp;

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
  return tmp;
}

void CVX::pubTraj(double** x)
{
  // printf("In pubTraj\n");
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
}

void CVX::pubTraj(Eigen::MatrixXd X)
{
  // printf("In pubTraj\n");

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
  pub_traj_.publish(traj);
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
    mtx.lock();
    v_kdtree_new_pcls_.push_back(my_kdTreeStamped);
    mtx.unlock();
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

// Occupied CB
void CVX::mapCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg)
{
  // printf("In mapCB\n");

  if (pcl2ptr_msg->width == 0 || pcl2ptr_msg->height == 0)  // Point Cloud is empty (this happens at the beginning)
  {
    return;
  }
  mtx.lock();
  // printf("In mapCB2\n");
  // pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_msg, *pclptr_map_);
  std::vector<int> index;
  // TODO: there must be a better way to check this. It's here because (in the simulation) sometimes all the points
  // are NaN (when the drone is on the ground and stuck moving randomly). If this is not done, the program breaks. I
  // think it won't be needed in the real drone
  // printf("In mapCB3\n");
  pcl::removeNaNFromPointCloud(*pclptr_map_, *pclptr_map_, index);
  if (pclptr_map_->size() == 0)
  {
    return;
  }
  // printf("In mapCB4, size=%d\n", v_kdtree_new_pcls_.size());
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
  // printf("below\n");

  kdtree_map_.setInputCloud(pclptr_map_);

  kdtree_map_initialized_ = 1;
  // printf("pasado2\n");
  mtx.unlock();
  // printf("pasado esto\n");
}

// Unkwown  CB
void CVX::unkCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg)
{
  if (pcl2ptr_msg->width == 0 || pcl2ptr_msg->height == 0)  // Point Cloud is empty (this happens at the beginning)
  {
    return;
  }
  mtx_unk.lock();
  pcl::fromROSMsg(*pcl2ptr_msg, *pclptr_unk_);
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*pclptr_unk_, *pclptr_unk_, index);
  if (pclptr_unk_->size() == 0)
  {
    return;
  }
  kdtree_unk_.setInputCloud(pclptr_unk_);
  kdtree_unk_initialized_ = 1;
  mtx_unk.unlock();
}

// TODO: check also against unkown space? Be careful because with the new points cloud I may have information of
// previously-unknown voxels
bool CVX::trajIsFree(Eigen::MatrixXd X)
{
  // printf("********In trajIsFree\n");
  // std::cout << X << std::endl;
  mtx.lock();

  // TODO: maybe there is a more efficient way to do this (sampling only some points of X?)
  if (((X.col(2)).array() < 0).any() == true)  // If there is some z < 0. Note that in eigen, first_index=0
  {
    printf("Collision with the ground \n");
    std::cout << X.col(3) << std::endl << std::endl;
    mtx.unlock();
    return false;  // There is a collision with the ground
  }

  int n = 1;  // Find nearest element

  Eigen::Vector3d eig_search_point(X(0, 0), X(0, 1), X(0, 2));
  pcl::PointXYZ pcl_search_point = eigenPoint2pclPoint(eig_search_point);
  double r = 100000;
  int last_i = 0;

  while (last_i < X.rows() - 1)
  {
    // printf("Inside the loop, last_i=%d\n", last_i);
    // last point clouds
    std::vector<int> id_inst(n);
    std::vector<float> dist2_inst(n);  // squared distance
    pcl_search_point = eigenPoint2pclPoint(eig_search_point);
    Eigen::Vector3d intersectionPoint;
    for (unsigned i = v_kdtree_new_pcls_.size() - 1; i < v_kdtree_new_pcls_.size() && i >= 0; ++i)
    {
      if (v_kdtree_new_pcls_[i].kdTree.nearestKSearch(pcl_search_point, n, id_inst, dist2_inst) > 0)
      {
        if (sqrt(dist2_inst[0]) < r)
        {
          r = sqrt(dist2_inst[0]);
          pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr = v_kdtree_new_pcls_[i].kdTree.getInputCloud();
          intersectionPoint << ptr->points[id_inst[0]].x, ptr->points[id_inst[0]].y, ptr->points[id_inst[0]].z;
        }
        // r = std::min(r, sqrt(dist2_inst[0]));
      }
    }

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

    // unknwown
    std::vector<int> id_unk(n);
    std::vector<float> dist2_unk(n);  // squared distance

    mtx_unk.lock();
    if (kdtree_unk_.nearestKSearch(pcl_search_point, n, id_unk, dist2_unk) > 0)
    {
      if (sqrt(dist2_unk[0]) < r)
      {
        r = sqrt(dist2_unk[0]);
        pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr = kdtree_unk_.getInputCloud();
        intersectionPoint << ptr->points[id_unk[0]].x, ptr->points[id_unk[0]].y, ptr->points[id_unk[0]].z;
      }
    }
    mtx_unk.unlock();

    // Now r is the distance to the nearest obstacle (considering unknown space as obstacles)
    if (r < DRONE_RADIUS)
    {
      printf("There is a collision with i=%d out of X.rows=%d\n", last_i, X.rows() - 1);
      pubintersecPoint(intersectionPoint, true);
      mtx.unlock();
      return false;  // There is a collision
    }

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

  mtx.unlock();
  return true;  // It's free!
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
  std::vector<int> id;                 // pointIdxRadiusSearch
  std::vector<float> sd;               // pointRadiusSquaredDistance
  pcl::PointXYZ sp(x[0], x[1], x[2]);  // searchPoint=x
  mtx.lock();
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
  mtx.unlock();

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
  // printf("In pubActualTraj\n");

  static geometry_msgs::Point p_last = pointOrigin();

  Eigen::Vector3d act_pos(state_.pos.x, state_.pos.y, state_.pos.z);
  Eigen::Vector3d t_goal(term_goal_.pos.x, term_goal_.pos.y, term_goal_.pos.z);
  float dist_to_goal = (t_goal - act_pos).norm();

  if (dist_to_goal < 2 * GOAL_RADIUS)
  {
    return;
  }

  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::ADD;
  m.id = actual_trajID_;
  actual_trajID_++;
  m.color = color(GREEN);
  m.scale.x = 0.02;
  m.scale.y = 0.04;
  m.scale.z = 1;
  m.header.stamp = ros::Time::now();
  m.header.frame_id = "world";

  geometry_msgs::Point p;
  p = eigen2point(act_pos);
  m.points.push_back(p_last);
  m.points.push_back(p);
  pub_actual_traj_.publish(m);
  p_last = p;
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
      double max_values[1] = { V_MAX };
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
      double max_values[2] = { V_MAX, A_MAX };
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

/*Eigen::Vector3d CVX::projectClickedGoal(Eigen::Vector3d P1)
{
  //[px1, py1, pz1] is inside the map
  //[px2, py2, pz2] is outside the map
  P2 = term_term_goal_;
  double x_max = P1[0] + WDX / 2;
  double x_min = P1[0] - WDX / 2;
  double y_max = P1[1] + WDY / 2;
  double y_min = P1[1] - WDY / 2;
  double z_max = P1[2] + WDZ / 2;
  double z_min = P1[2] - WDZ / 2;

  if ((P1[0] < x_max && P1[0] > xmin) && (P1[1] < y_max && P1[1] > ymin) && (P1[2] < z_max && P1[2] > zmin))
  {
    // Clicked goal is inside the map
    return P1;
  }
  Eigen::Vector3d inters;
  std::vector<Eigen::Vector4d> all_planes = {
    Eigen::Vector4d(1, 0, 0, P1[0] + WDX / 2),   // Plane X right
    Eigen::Vector4d(-1, 0, 0, P1[0] - WDX / 2),  // Plane X left
    Eigen::Vector4d(0, 1, 0, P1[1] + WDY / 2),   // Plane Y right
    Eigen::Vector4d(0, -1, 0, P1[1] - WDY / 2),  // Plane Y left
    Eigen::Vector4d(0, 0, 1, P1[2] + WDZ / 2),   // Plane Z up
    Eigen::Vector4d(0, 0, -1, P1[2] - WDZ / 2)   // Plane Z down
  };
  for (int i = 0; i < 6; i++)
  {
    if (getIntersectionWithPlane(P1, P2, coeff, &inters) == true)
    {
      return inters;
    }
  }
  printf("Neither the goal is inside the map nor it has a projection into it, this is impossible");
}*/