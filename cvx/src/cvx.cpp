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

// TODO: yaw should be pointing to the velocity vector? Or maybe to the place with more unknown space?

// TODO: First of all, try with an straight line to the goal

// TODO: Quiz'a puedo anadir al potential field otra fuerza que me aleje de los sitios por los cuales ya he pasado?

// TODO: I think in CVXGEN it should be sum from 0 to..., instead of from 1 to... (in the cost function)

// TODO: Remove all the points below the ground in the point cloud

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

#define Ra 4.0  // [m] Radius of the first sphere
#define Rb 6.0  // [m] Radius of the second sphere

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
  pub_path_jps_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps", 1);

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

void CVX::solveJPS3D(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr, Vec3f start, Vec3f goal)
{
  // Create a map
  /*  std::cout << "Solving JPS from start\n" << start << std::endl;
    std::cout << "To goal\n" << goal << std::endl;*/
  const Vec3i dim(cells_x_, cells_y_, cells_z_);  //  number of cells in each dimension
  const Vec3f center_map = start;                 // position of the drone
  // Read the pointcloud

  MapReader<Vec3i, Vec3f> reader(pclptr, dim, RES, center_map);  // Map read

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
    path_jps_vector_ = planner_ptr->getPath();  // getRawPath() if you want the path with more corners (not "cleaned")

    /*    printf("First point in path_jps_vector_:\n");
        std::cout << path_jps_vector_[0].transpose() << std::endl;*/
    directionJPS_ = path_jps_vector_[1] - path_jps_vector_[0];
    // printf("Estoy aqui: \n");
    /*    for (const auto& it : path_jps_vector)
        {
          std::cout << it.transpose() << std::endl;
        }*/
    path_jps_ = clearArrows();
    vectorOfVectors2MarkerArray(path_jps_vector_, &path_jps_);
    pub_path_jps_.publish(path_jps_);
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
  return;
}

visualization_msgs::MarkerArray CVX::clearArrows()
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
}

void CVX::vectorOfVectors2MarkerArray(vec_Vecf<3> traj, visualization_msgs::MarkerArray* m_array)
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
    m.color = color(BLUE);
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
  status_ = TRAVELING;
  planner_status_ = START_REPLANNING;
  force_reset_to_0_ = true;
  printf("GCB: planner_status_ = START_REPLANNING\n");
  goal_click_initialized_ = true;
  clearMarkerActualTraj();
  printf("Exiting from goalCB\n");
}

void CVX::yaw(double diff, acl_msgs::QuadGoal& quad_goal)
{
  float plan_eval_time_ = 0.01;
  float r_max_ = 1;
  saturate(diff, -plan_eval_time_ * r_max_, plan_eval_time_ * r_max_);
  if (diff > 0)
    quad_goal.dyaw = r_max_;
  else
    quad_goal.dyaw = -r_max_;
  quad_goal.yaw += diff;
}

void CVX::replanCB(const ros::TimerEvent& e)
{
  // printf("In replanCB\n");
  // double t0replanCB = ros::Time::now().toSec();
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

  double dist_to_goal = sqrt(pow(term_goal_.pos.x - state_.pos.x, 2) + pow(term_goal_.pos.y - state_.pos.y, 2) +
                             pow(term_goal_.pos.z - state_.pos.z, 2));

  // 0.96 and 0.98 are to ensure that ra<rb<dist_to_goal always
  double ra = std::min(0.96 * dist_to_goal, Ra);                        // radius of the sphere Sa
  double rb = std::min(0.98 * dist_to_goal, Rb);                        // radius of the sphere Sa
  Eigen::Vector3d state_pos(state_.pos.x, state_.pos.y, state_.pos.z);  // Local copy of state
  Eigen::Vector3d curr_pos = state_pos;
  Eigen::Vector3d term_goal(term_goal_.pos.x, term_goal_.pos.y, term_goal_.pos.z);

  // TODO: I'm using only the direction of the force, but not the magnitude. Could I use the magnitude?
  // TODO: If I'm only using the direction of the force, not sure if quadratic+linear separation is needed in the
  // attractive force
  if (dist_to_goal < GOAL_RADIUS)
  {
    status_ = GOAL_REACHED;
  }
  // printf("Entering in replanCB, planner_status_=%d\n", planner_status_);
  if (status_ == GOAL_SEEN || status_ == GOAL_REACHED || planner_status_ == REPLANNED)
  {
    // printf("No replanning needed because planner_status_=%d\n", planner_status_);
    return;
  }

  // If you want the force to be the direction selector
  // Eigen::Vector3d force = computeForce(curr_pos, term_goal);
  // double x = force[0], y = force[1], z = force[2];
  // If you want the JPS3D solution to be the direction selector
  // double x = directionJPS_[0], y = directionJPS_[1], z = directionJPS_[2];

  Vec3f goal(term_goal_.pos.x, term_goal_.pos.y, term_goal_.pos.z);
  solveJPS3D(pclptr_map_, state_pos, goal);  // Solution is in path_jps_vector_

  Eigen::Vector3d center = state_pos;
  Eigen::Vector3d B1 = getFirstIntersectionWithSphere(path_jps_vector_, ra, center);

  // Direction from the center to that point:
  double x = B1[0] - center[0], y = B1[1] - center[1], z = B1[2] - center[2];

  double theta0 = acos(z / (sqrt(x * x + y * y + z * z)));
  double phi0 = atan2(y, x);

  bool found_it = 0;

  Eigen::AngleAxis<double> rot_z(phi0, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxis<double> rot_y(theta0, Eigen::Vector3d::UnitY());

  for (double theta = 0; theta <= 3.14 / 2 && !found_it; theta = theta + 3.14 / 10)
  {
    for (double phi = 0; phi <= 2 * 3.14 && !found_it; phi = phi + 3.14 / 10)
    {
      Eigen::Vector3d p1, p2;
      p1[0] = ra * sin(theta) * cos(phi);
      p1[1] = ra * sin(theta) * sin(phi);
      p1[2] = ra * cos(theta);
      Eigen::Vector3d trans = state_pos;
      p2 = rot_z * rot_y * p1 + trans;

      if (p2[2] < 0)  // If below the ground, discard
      {
        continue;
      }

      /*      // Solver VEL
            double xf_sphere[3] = { p2[0], p2[1], p2[2] };
            mtx_goals.lock();
            double x0[3] = { initialCond_.pos.x, initialCond_.pos.y, initialCond_.pos.z };
            double u0[3] = { initialCond_.vel.x, initialCond_.vel.y, initialCond_.vel.z };
            mtx_goals.unlock();
            solver_vel_.set_xf(xf_sphere);
            solver_vel_.set_x0(x0);
            printf("x0 is %f, %f, %f\n", x0[0], x0[1], x0[2]);
            printf("xf is %f, %f, %f\n", xf_sphere[0], xf_sphere[1], xf_sphere[2]);
            double max_values[1] = { V_MAX };
            solver_vel_.set_max(max_values);
            solver_vel_.set_u0(u0);
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
            double u0[3] = { initialCond_.accel.x, initialCond_.accel.y, initialCond_.accel.z };
            mtx_goals.unlock();
            solver_accel_.set_xf(xf_sphere);
            solver_accel_.set_x0(x0);
            double max_values[2] = { V_MAX, A_MAX };
            solver_accel_.set_max(max_values);
            solver_accel_.set_u0(u0);
            solver_accel_.genNewTraj();
            U_temp_ = solver_accel_.getU();
            X_temp_ = solver_accel_.getX();*/

      // Solver JERK
      if (optimized_)  // Needed to skip the first time (X_ still not initialized)
      {
        k_initial_cond_ = std::min(k_ + OFFSET, (int)(X_.rows() - 1));
        updateInitialCond(k_initial_cond_);
      }
      double xf_sphere[9] = { p2[0], p2[1], p2[2], 0, 0, 0, 0, 0, 0 };
      mtx_goals.lock();
      double x0[9] = { initialCond_.pos.x,   initialCond_.pos.y,   initialCond_.pos.z,
                       initialCond_.vel.x,   initialCond_.vel.y,   initialCond_.vel.z,
                       initialCond_.accel.x, initialCond_.accel.y, initialCond_.accel.z };

      /*      printf("(Optimizando desde): %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", initialCond_.pos.x,
         initialCond_.pos.y, initialCond_.pos.z, initialCond_.vel.x, initialCond_.vel.y, initialCond_.vel.z);

            printf("(State): %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", state_.pos.x, state_.pos.y, state_.pos.z,
                   state_.vel.x, state_.vel.y, state_.vel.z);*/

      double u0[3] = { initialCond_.jerk.x, initialCond_.jerk.y, initialCond_.jerk.z };
      mtx_goals.unlock();
      solver_jerk_.set_xf(xf_sphere);
      solver_jerk_.set_x0(x0);
      double max_values[3] = { V_MAX, A_MAX, J_MAX };
      solver_jerk_.set_max(max_values);
      solver_jerk_.set_u0(u0);
      solver_jerk_.genNewTraj();
      U_temp_ = solver_jerk_.getU();
      X_temp_ = solver_jerk_.getX();
      Timer time_coll_check(true);
      bool isFree = trajIsFree(X_temp_);
      double ms_ellapsed = time_coll_check.Elapsed().count();
      printf("Collision check takes: %f ms/traj\n", ms_ellapsed);
      createMarkerSetOfArrows(X_temp_, isFree);

      if (isFree)
      {
        double JPrimj1 = solver_jerk_.getCost();
        // printf("Current state=%0.2f, %0.2f, %0.2f\n", state_pos[0], state_pos[1], state_pos[2]);
        // printf("Before going to get C1, Points in JPS1\n");
        /*        for (int i = 0; i < path_jps_vector_.size(); i++)
                {
                  std::cout << path_jps_vector_[i].transpose() << std::endl;
                }*/
        Eigen::Vector3d C1 = getLastIntersectionWithSphere(path_jps_vector_, rb, center);
        pubPlanningVisual(center, ra, rb, B1, C1);
        vec_Vecf<3> WP = getPointsBw2Spheres(path_jps_vector_, ra, rb, center);
        WP.insert(WP.begin(), B1);
        WP.push_back(C1);
        double JPrimv1 = solveVelAndGetCost(WP);

        planner_status_ = REPLANNED;  // Don't replan again until start publishing current solution
        printf("ReplanCB: planner_status_ = REPLANNED\n");
        optimized_ = true;
        pubTraj(X_);
        found_it = 1;
        double dist_end_traj_to_goal =
            sqrt(pow(term_goal_.pos.x - xf_sphere[0], 2) + pow(term_goal_.pos.y - xf_sphere[1], 2) +
                 pow(term_goal_.pos.z - xf_sphere[2], 2));
        if (dist_end_traj_to_goal < GOAL_RADIUS)
        {  // I've found a free path that ends in the goal --> no more replanning (to avoid oscillations when reaching
          // the goal)
          status_ = GOAL_SEEN;
          printf("CHANGED TO GOAL_SEEN********\n");
        }
      }
    }
  }
  if (!found_it)
  {
    ROS_ERROR("Unable to find a free traj");
  }
  pub_trajs_sphere_.publish(trajs_sphere_);
  // ROS_WARN("solve time: %0.2f ms", 1000 * (ros::Time::now().toSec() - then));
  // printf("Time in replanCB %0.2f ms\n", 1000 * (ros::Time::now().toSec() - t0replanCB));
}

double CVX::solveVelAndGetCost(vec_Vecf<3> path)
{
  double cost = 0;
  /*  printf("Points in the path VEL\n");
    for (int i = 0; i < path.size() - 1; i++)
    {
      std::cout << path[i].transpose() << std::endl;
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

    /*    if ((planner_status_ == REPLANNED && (k > OFFSET - 1)) && status_ == TRAVELING)
        {
          printf("INCREASE OFFSET: the initial condition has already been published\n");
          planner_status_ = START_REPLANNING;  // Let's try to replan again...
          printf("pucCB1: planner_status_=START_REPLANNING\n");
        }*/

    // printf("k=%d\n", k);
    // printf("ROWS of X=%d\n", X_.rows());
    if ((planner_status_ == REPLANNED && (k_ == k_initial_cond_)) ||
        (force_reset_to_0_ && planner_status_ == REPLANNED))
    {
      // printf("Starting again\n");
      force_reset_to_0_ = false;
      X_ = X_temp_;
      U_ = U_temp_;
      k_ = 0;  // Start again publishing the waypoints in X_ from the first row

      planner_status_ = START_REPLANNING;
      printf("pucCB2: planner_status_=START_REPLANNING\n");
      // printf("%f, %f, %f, %f, %f, %f\n", X_(k, 0), X_(k, 1), X_(k, 2), X_(k, 3), X_(k, 4), X_(k, 5));
    }

    k_ = std::min(k_, (int)(X_.rows() - 1));
    int kp1 = std::min(k_ + OFFSET, (int)(X_.rows() - 1));  // k plus offset
                                                            // std::cout << "esto es X" << std::endl;
                                                            // std::cout << X_ << std::endl;
                                                            // printf("masabajo2\n");
    // printf("K-->%f, %f, %f, %f, %f, %f\n", X_(k, 0), X_(k, 1), X_(k, 2), X_(k, 3), X_(k, 4), X_(k, 5));
    // printf("KP1-->%f, %f, %f, %f, %f, %f\n", X_(kp1, 0), X_(kp1, 1), X_(kp1, 2), X_(kp1, 3), X_(kp1, 4), X_(kp1, 5));
    quadGoal_.pos = getPos(k_);
    quadGoal_.vel = getVel(k_);
    //// printf("masabajo3\n");
    quadGoal_.accel = (use_ff_) ? getAccel(k_) : vectorNull();
    quadGoal_.jerk = (use_ff_) ? getJerk(k_) : vectorNull();

    quadGoal_.dyaw = 0;

    if (status_ == TRAVELING || status_ == GOAL_SEEN)
    {
      double desired_yaw = atan2(quadGoal_.vel.y, quadGoal_.vel.x);
      double diff = desired_yaw - quadGoal_.yaw;
      angle_wrap(diff);
      yaw(diff, quadGoal_);
    }
    /*  double heading_ = atan2(term_goal_.pos.y - quadGoal_.pos.y, term_goal_.pos.x - quadGoal_.pos.x);
      double diff = heading_ - quadGoal_.yaw;
      angle_wrap(diff);
      yaw(diff, quadGoal_);

      sleep(3);

      quadGoal_.dyaw = 0;*/

    // quadGoal_.yaw = atan2(quadGoal_.vel.y / quadGoal_.vel.x);

    k_++;
  }
  else
  {
    quadGoal_.cut_power = true;
  }

  // printf("In pubCB5\n");
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
  // printf("Y mas masabajo\n");
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
  printf("********In trajIsFree\n");
  // std::cout << X << std::endl;

  mtx.lock();
  int n = 1;  // Find nearest element

  Eigen::Vector3d eig_search_point(X(0, 0), X(0, 1), X(0, 2));
  pcl::PointXYZ pcl_search_point = eigenPoint2pclPoint(eig_search_point);
  double r = 100000;
  int last_i = 0;

  while (last_i < X.rows() - 1)
  {
    printf("Inside the loop, last_i=%d\n", last_i);
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