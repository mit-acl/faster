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

using namespace JPS;

CVX::CVX(ros::NodeHandle nh, ros::NodeHandle nh_replan_CB, ros::NodeHandle nh_pub_CB)
  : nh_(nh), nh_replan_CB_(nh_replan_CB), nh_pub_CB_(nh_pub_CB)
{
  replan_ = false;
  optimized_ = false;
  flight_mode_.mode = flight_mode_.NOT_FLYING;

  pub_goal_ = nh_.advertise<acl_msgs::QuadGoal>("goal", 1);
  pub_traj_ = nh_.advertise<nav_msgs::Path>("traj", 1);
  pub_setpoint_ = nh_.advertise<visualization_msgs::Marker>("setpoint", 1);
  pub_trajs_sphere_ = nh_.advertise<visualization_msgs::MarkerArray>("trajs_sphere", 1);
  pub_forces_ = nh_.advertise<visualization_msgs::MarkerArray>("forces", 1);
  pub_actual_traj_ = nh_.advertise<visualization_msgs::Marker>("actual_traj", 1);
  pub_path_jps_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps", 1);

  sub_goal_ = nh_.subscribe("term_goal", 1, &CVX::goalCB, this);
  sub_mode_ = nh_.subscribe("flightmode", 1, &CVX::modeCB, this);
  sub_state_ = nh_.subscribe("state", 1, &CVX::stateCB, this);
  sub_map_ = nh_.subscribe("occup_grid", 1, &CVX::mapCB, this);
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
  /*  setpoint_.pose.orientation.x = 0.0;
    setpoint_.pose.orientation.y = 0.0;
    setpoint_.pose.orientation.z = 0.0;
    setpoint_.pose.orientation.w = 1.0;*/
  setpoint_.scale.x = 0.35;
  setpoint_.scale.y = 0.35;
  setpoint_.scale.z = 0.35;
  setpoint_.color = color(ORANGE_TRANS);

  Accel::initialize_optimizer();

  term_goal_.pos = vectorNull();

  quadGoal_.pos = vectorNull();
  quadGoal_.vel = vectorNull();
  quadGoal_.accel = vectorNull();
  quadGoal_.jerk = vectorNull();

  nextQuadGoal_.pos = vectorNull();
  nextQuadGoal_.vel = vectorNull();
  nextQuadGoal_.accel = vectorNull();
  nextQuadGoal_.jerk = vectorNull();

  markerID_ = 0;

  cells_x_ = (int)WDX / RES;
  cells_y_ = (int)WDY / RES;
  cells_z_ = (int)WDZ / RES;

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

void CVX::solveJPS3D(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr)
{
  // printf("In solveJPSD\n");
  // Create a map
  const Vec3f start(state_.pos.x, state_.pos.y, state_.pos.z);
  const Vec3f goal(term_goal_.pos.x, term_goal_.pos.y, term_goal_.pos.z);
  const Vec3i dim(cells_x_, cells_y_, cells_z_);  //  number of cells in each dimension
  // printf("In solveJPSD2\n");
  const Vec3f center_map(state_.pos.x, state_.pos.y, state_.pos.z);  // position of the drone
  // Read the pointcloud
  // printf("In solveJPSD2.5\n");
  MapReader<Vec3i, Vec3f> reader(pclptr, dim, RES, center_map);  // Map read

  // printf("In solveJPSD3\n");

  std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();
  map_util->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());
  // printf("In solveJPSD3.5\n");

  std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(false));  // Declare a planner
  // printf("In solveJPSD5\n");
  planner_ptr->setMapUtil(map_util);  // Set collision checking function
  planner_ptr->updateMap();

  // printf("In solveJPSD6\n");
  Timer time_jps(true);
  bool valid_jps = planner_ptr->plan(
      start, goal, 1, true);  // Plan from start to goal with heuristic weight=1, and using JPS (if false --> use A*)
  // printf("In solveJPSD6.5\n");
  if (valid_jps == true)  // There is a solution
  {
    double dt_jps = time_jps.Elapsed().count();
    // printf("JPS Planner takes: %f ms\n", dt_jps);
    // printf("JPS Path Distance: %f\n", total_distance3f(planner_ptr->getPath()));  // getRawPath() if you want the
    // path
    // with more corners (not "cleaned")
    // printf("JPS Path: \n");
    vec_Vecf<3> path_jps_vector =
        planner_ptr->getPath();  // getRawPath() if you want the path with more corners (not "cleaned")

    directionJPS_ = path_jps_vector[1] - path_jps_vector[0];
    // printf("Estoy aqui: \n");
    /*    for (const auto& it : path_jps_vector)
        {
          std::cout << it.transpose() << std::endl;
        }*/
    // printf("Estoy aqui2: \n");
    path_jps_ = clearArrows();
    // printf("Estoy aqui3: \n");
    vectorOfVectors2MarkerArray(path_jps_vector, &path_jps_);
    // printf("Estoy aqui4: \n");
    pub_path_jps_.publish(path_jps_);
    // printf("Estoy aqui5: \n");
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
  // printf("In goalCB\n");
  term_goal_ = msg;
  replanning_needed_ = true;
  goal_click_initialized_ = true;
  clearMarkerActualTraj();
}

void CVX::replanCB(const ros::TimerEvent& e)
{
  // printf("In replanCB\n");
  // double t0replanCB = ros::Time::now().toSec();
  clearMarkerSetOfArrows();
  if (!kdtree_map_initialized_)
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

  // TODO: r_sphere_max should be a parameter
  double r_sphere_max = 4.0;
  double r = std::min(dist_to_goal, r_sphere_max);  // radius of the sphere
  Eigen::Vector3d curr_pos(state_.pos.x, state_.pos.y, state_.pos.z);
  Eigen::Vector3d term_goal(term_goal_.pos.x, term_goal_.pos.y, term_goal_.pos.z);

  // TODO: I'm using only the direction of the force, but not the magnitude. Could I use the magnitude?
  // TODO: If I'm only using the direction of the force, not sure if quadratic+linear separation is needed in the
  // attractive force

  if (replanning_needed_ == false || dist_to_goal < GOAL_RADIUS)
  {
    // printf("No replanning needed\n");
    return;
  }

  // If you want the force to be the direction selector
  // Eigen::Vector3d force = computeForce(curr_pos, term_goal);
  // double x = force[0], y = force[1], z = force[2];

  // If you want the JPS3D solution to be the direction selector
  double x = directionJPS_[0], y = directionJPS_[1], z = directionJPS_[2];

  double theta0 = acos(z / (sqrt(x * x + y * y + z * z)));
  double phi0 = atan2(y, x);

  bool found_it = 0;

  Eigen::AngleAxis<float> rot_z(phi0, Eigen::Vector3f::UnitZ());
  Eigen::AngleAxis<float> rot_y(theta0, Eigen::Vector3f::UnitY());

  for (double theta = 0; theta <= 3.14 / 2 && !found_it; theta = theta + 3.14 / 10)
  {
    for (double phi = 0; phi <= 2 * 3.14 && !found_it; phi = phi + 3.14 / 10)
    {
      Eigen::Vector3f p1, p2;
      p1[0] = r * sin(theta) * cos(phi);
      p1[1] = r * sin(theta) * sin(phi);
      p1[2] = r * cos(theta);
      Eigen::Vector3f trans(state_.pos.x, state_.pos.y, state_.pos.z);
      p2 = rot_z * rot_y * p1 + trans;

      if (p2[2] < 0)  // If below the ground, discard
      {
        continue;
      }

      double xf_sphere[6] = { 0, 0, 0, 0, 0, 0 };
      xf_sphere[0] = p2[0];
      xf_sphere[1] = p2[1];
      xf_sphere[2] = p2[2];

      mtx_goals.lock();
      /*      ROS_INFO("Antes de asignar x0, nQG: %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", nextQuadGoal_.pos.x,
                     nextQuadGoal_.pos.y, nextQuadGoal_.pos.z, nextQuadGoal_.vel.x, nextQuadGoal_.vel.y,
         nextQuadGoal_.vel.z);*/

      double x0[6] = { nextQuadGoal_.pos.x, nextQuadGoal_.pos.y, nextQuadGoal_.pos.z,
                       nextQuadGoal_.vel.x, nextQuadGoal_.vel.y, nextQuadGoal_.vel.z };
      double u0[3] = { nextQuadGoal_.accel.x, nextQuadGoal_.accel.y, nextQuadGoal_.accel.z };
      mtx_goals.unlock();
      printf("x0   xf\n");
      for (int i = 0; i < 6; i++)
      {
        printf("%0.2f  %0.2f\n", x0[i], xf_sphere[i]);
      }

      solver_accel_.set_xf(xf_sphere);
      solver_accel_.set_x0(x0);
      double max_values[2] = { V_MAX, A_MAX };
      solver_accel_.set_max(max_values);
      solver_accel_.set_u0(u0);
      solver_accel_.genNewTraj();
      U_temp_ = solver_accel_.getU();
      X_temp_ = solver_accel_.getX();
      // solver_accel_.genNewTraj(u_max_, xf_sphere);  // Now X_temp_ has the stuff
      bool isFree = trajIsFree(X_temp_);
      // printf("Its free\n");
      createMarkerSetOfArrows(X_temp_, isFree);
      // printf("Markers Created\n");
      if (isFree)
      {
        // printf("******Replanned!\n");
        X_ = X_temp_;
        U_ = U_temp_;
        replan_ = true;
        optimized_ = true;
        pubTraj(X_);
        found_it = 1;
        double dist_end_traj_to_goal =
            sqrt(pow(term_goal_.pos.x - xf_sphere[0], 2) + pow(term_goal_.pos.y - xf_sphere[1], 2) +
                 pow(term_goal_.pos.z - xf_sphere[2], 2));
        if (dist_end_traj_to_goal < GOAL_RADIUS)
        {  // I've found a free path that ends in the goal --> no more replanning (to avoid oscillations when reaching
          // the goal)
          replanning_needed_ = false;
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

void CVX::modeCB(const acl_msgs::QuadFlightMode& msg)
{
  // printf("In modeCB\n");
  if (msg.mode == msg.LAND && flight_mode_.mode != flight_mode_.LAND)
  {
    double xf[6] = { quadGoal_.pos.x, quadGoal_.pos.y, z_land_, 0, 0, 0 };
    double max_values[2] = { V_MAX, A_MAX };
    solver_accel_.set_max(max_values);  // TODO: To land, I use u_min_
    solver_accel_.set_xf(xf);
    solver_accel_.genNewTraj();
  }
  flight_mode_.mode = msg.mode;
}

void CVX::stateCB(const acl_msgs::State& msg)
{
  // printf("In stateCB\n");
  state_ = msg;
  // Stop updating when we get GO
  if (flight_mode_.mode == flight_mode_.NOT_FLYING || flight_mode_.mode == flight_mode_.KILL)
  {
    quadGoal_.pos = msg.pos;
    quadGoal_.vel = msg.vel;
    z_start_ = msg.pos.z;
    z_start_ = std::max(0.0, z_start_);
  }

  pubActualTraj();
}

void CVX::pubCB(const ros::TimerEvent& e)
{
  mtx_goals.lock();
  // printf("In pubCB\n");

  // double t0pubCB = ros::Time::now().toSec();

  if (flight_mode_.mode == flight_mode_.LAND)
  {
    double d = sqrt(pow(quadGoal_.pos.z - z_land_, 2));
    if (d < 0.1)
    {
      ros::Duration(1.0).sleep();
      flight_mode_.mode = flight_mode_.NOT_FLYING;
    }
  }

  // printf("In pubCB2\n");

  quadGoal_.header.stamp = ros::Time::now();
  quadGoal_.header.frame_id = "world";

  quadGoal_.vel = vectorNull();
  quadGoal_.accel = vectorNull();
  quadGoal_.jerk = vectorNull();

  nextQuadGoal_.vel = vectorNull();
  nextQuadGoal_.accel = vectorNull();
  nextQuadGoal_.jerk = vectorNull();

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
  static int k = 0;
  if (optimized_ && flight_mode_.mode != flight_mode_.NOT_FLYING && flight_mode_.mode != flight_mode_.KILL)
  {
    quadGoal_.cut_power = false;
    if (replan_)
    {
      printf("***Setting k=0!\n");
      k = 0;  // Start again publishing the waypoints in X_ from the first row
      replan_ = false;
    }

    k = std::min(k, (int)(X_.rows() - 1));
    int kp1 = std::min(k + 1, (int)(X_.rows() - 1));  // k plus one

    quadGoal_.pos = getPos(k);
    quadGoal_.vel = getVel(k);
    quadGoal_.accel = (use_ff_) ? getAccel(k) : vectorNull();
    quadGoal_.jerk = (use_ff_) ? getJerk(k) : vectorNull();
    nextQuadGoal_.pos = getPos(kp1);
    nextQuadGoal_.vel = getVel(kp1);
    nextQuadGoal_.accel = (use_ff_) ? getAccel(kp1) : vectorNull();
    nextQuadGoal_.jerk = (use_ff_) ? getJerk(kp1) : vectorNull();
    k++;
  }
  else
  {
    quadGoal_.cut_power = true;
  }

  // printf("In pubCB5\n");
  ROS_INFO("publishing quadGoal: %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", quadGoal_.pos.x, quadGoal_.pos.y,
           quadGoal_.pos.z, quadGoal_.vel.x, quadGoal_.vel.y, quadGoal_.vel.z);
  ROS_INFO("(y nextQuadGoal_): %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", nextQuadGoal_.pos.x, nextQuadGoal_.pos.y,
           nextQuadGoal_.pos.z, nextQuadGoal_.vel.x, nextQuadGoal_.vel.y, nextQuadGoal_.vel.z);

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
  geometry_msgs::Vector3 tmp;
  tmp.x = X_(i, 0);
  tmp.y = X_(i, 1);
  tmp.z = X_(i, 2);
  return tmp;
}

geometry_msgs::Vector3 CVX::getVel(int i)
{
  geometry_msgs::Vector3 tmp;
  tmp.x = X_(i, 3);
  tmp.y = X_(i, 4);
  tmp.z = X_(i, 5);
  return tmp;
}

geometry_msgs::Vector3 CVX::getAccel(int i)
{
  geometry_msgs::Vector3 tmp;
  tmp.x = U_(i, 0);
  tmp.y = U_(i, 1);
  tmp.z = U_(i, 2);
  return tmp;
}

geometry_msgs::Vector3 CVX::getJerk(int i)
{
  geometry_msgs::Vector3 tmp;
  tmp.x = U_(i, 3);
  tmp.y = U_(i, 4);
  tmp.z = U_(i, 5);
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

  for (int i = 1; i < N_; i++)
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
  // printf("In createMarkerSetOfArrows\n");
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_msg, *pclptr);
  std::vector<int> index;
  // TODO: there must be a better way to check this. It's here because (in the simulation) sometimes all the points
  // are NaN (when the drone is on the ground and stuck moving randomly). If this is not done, the program breaks. I
  // think it won't be needed in the real drone
  // printf("In mapCB3\n");
  pcl::removeNaNFromPointCloud(*pclptr, *pclptr, index);
  if (pclptr->size() == 0)
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
  solveJPS3D(pclptr);

  kdtree_map_.setInputCloud(pclptr);
  kdtree_map_initialized_ = 1;
  // printf("pasado2\n");
  mtx.unlock();
  // printf("pasado esto\n");
}

// TODO: check also against unkown space? Be careful because with the new points cloud I may have information of
// previously-unknown voxels
bool CVX::trajIsFree(Eigen::MatrixXd X)
{
  // printf("In trajIsFree\n");

  mtx.lock();
  // TODO: this cloud should be a subset of the entire cloud, not all the cloud (faster?)
  bool isFree = true;
  for (int i = 0; i < X.rows(); i = i + 1)  // Sample (a subset of) the points in the trajectory
  {
    pcl::PointXYZ searchPoint(X(i, 0), X(i, 1), X(i, 2));
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // TODO: implement smart check
    // TODO: maybe nearestKSearch is faster

    // Check collision in all the new point clouds
    // TODO: maybe I could check collision only against some of the new point clouds, not all of them.
    // TODO: change the -1 in the line below

    // TODO: check that it is inside the voxel, (not inside a sphere) would be more accurate
    unsigned novale = v_kdtree_new_pcls_.size() - 1;
    for (unsigned i = v_kdtree_new_pcls_.size() - 1; i < v_kdtree_new_pcls_.size() && i >= 0; ++i)
    {
      if (v_kdtree_new_pcls_[i].kdTree.radiusSearch(searchPoint, DRONE_RADIUS, pointIdxRadiusSearch,
                                                    pointRadiusSquaredDistance) > 0)
      {
        isFree = false;  // if we model the drone as an sphere, I'm done (there is a collision)
      }
    }

    // Check collision in the map

    if (kdtree_map_.radiusSearch(searchPoint, DRONE_RADIUS, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      isFree = false;  // if we model the drone as an sphere, I'm done (there is a collision)
    }
  }
  mtx.unlock();
  return isFree;  // this traj is free
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

std_msgs::ColorRGBA CVX::color(int id)
{
  std_msgs::ColorRGBA red;
  red.r = 1;
  red.g = 0;
  red.b = 0;
  red.a = 1;
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1;
  blue.a = 1;
  std_msgs::ColorRGBA blue_light;
  blue_light.r = 0.5;
  blue_light.g = 0.7;
  blue_light.b = 1;
  blue_light.a = 1;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1;
  green.b = 0;
  green.a = 1;
  std_msgs::ColorRGBA yellow;
  yellow.r = 1;
  yellow.g = 1;
  yellow.b = 0;
  yellow.a = 1;
  std_msgs::ColorRGBA orange_trans;  // orange transparent
  orange_trans.r = 1;
  orange_trans.g = 0.5;
  orange_trans.b = 0;
  orange_trans.a = 0.7;
  switch (id)
  {
    case RED:
      return red;
      break;
    case BLUE:
      return blue;
      break;
    case BLUE_LIGHT:
      return blue_light;
      break;
    case GREEN:
      return green;
      break;
    case YELLOW:
      return yellow;
      break;
    case ORANGE_TRANS:
      return orange_trans;
      break;
    default:
      ROS_ERROR("COLOR NOT DEFINED");
  }
}

// coeff is from highest degree to lowest degree. Returns the smallest positive real solution. Returns -1 if a
// root is imaginary or if it's negative

geometry_msgs::Point CVX::pointOrigin()
{
  geometry_msgs::Point tmp;
  tmp.x = 0;
  tmp.y = 0;
  tmp.z = 0;
  return tmp;
}

geometry_msgs::Point CVX::eigen2point(Eigen::Vector3d vector)
{
  geometry_msgs::Point tmp;
  tmp.x = vector[0];
  tmp.y = vector[1];
  tmp.z = vector[2];
  return tmp;
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

geometry_msgs::Vector3 CVX::vectorNull()
{
  geometry_msgs::Vector3 tmp;
  tmp.x = 0;
  tmp.y = 0;
  tmp.z = 0;
}
