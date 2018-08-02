#include "cvx.hpp"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/MarkerArray.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>

#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>

CVX::CVX(ros::NodeHandle nh) : nh_(nh)
{
  replan_ = false;
  optimized_ = false;
  flight_mode_.mode = flight_mode_.NOT_FLYING;

  pub_goal_ = nh_.advertise<acl_msgs::QuadGoal>("goal", 1);
  pub_traj_ = nh_.advertise<nav_msgs::Path>("traj", 1);
  pub_setpoint_ = nh_.advertise<visualization_msgs::Marker>("setpoint", 1);
  pub_trajs_sphere_ = nh_.advertise<visualization_msgs::MarkerArray>("trajs_sphere", 1);

  sub_goal_ = nh_.subscribe("term_goal", 1, &CVX::goalCB, this);
  sub_mode_ = nh_.subscribe("flightmode", 1, &CVX::modeCB, this);
  sub_state_ = nh_.subscribe("state", 1, &CVX::stateCB, this);
  sub_map_ = nh_.subscribe("occup_grid", 1, &CVX::mapCB, this);

  pubGoalTimer_ = nh_.createTimer(ros::Duration(0.01), &CVX::pubCB, this);

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
  setpoint_.pose.orientation.x = 0.0;
  setpoint_.pose.orientation.y = 0.0;
  setpoint_.pose.orientation.z = 0.0;
  setpoint_.pose.orientation.w = 1.0;
  setpoint_.scale.x = 0.35;
  setpoint_.scale.y = 0.35;
  setpoint_.scale.z = 0.35;
  setpoint_.color.a = 0.7;  // Don't forget to set the alpha!
  setpoint_.color.r = 1.0;
  setpoint_.color.g = 0.5;
  setpoint_.color.b = 0.0;

  initialize_optimizer();

  markerID_ = 0;
  markerID_last_ = 0;

  ROS_INFO("Initialized");
}

void CVX::goalCB(const acl_msgs::TermGoal& msg)
{
  // Prevents race condition with lower accel landing
  if (msg.pos.x != 0.0 || msg.pos.y != 0)
  {
    double xf[6] = { msg.pos.x, msg.pos.y, msg.pos.z, msg.vel.x, msg.vel.y, msg.vel.z };

    // Sample positions in an sphere using spherical coordinates
    double r = 2;     // radius of the sphere
    int n_phi = 6;    // Number of samples in phi.
    int n_theta = 6;  // Number of samples in theta.
    double d_theta = 3.14 / 2;
    double d_phi = 3.14 / 2;

    double x = xf[0] - state_.pos.x, y = xf[1] - state_.pos.y,
           z = xf[2] - state_.pos.z;  // x,y,z of the goal with respecto to the current pos
    double theta0 = acos(z / (sqrt(x * x + y * y + z * z)));
    double phi0 = atan2(y, x);

    visualization_msgs::MarkerArray trajs_sphere;  // all the trajectories generated in the sphere
    markerID_ = 0;
    clearMarkerSetOfArrows();

    int i_phi = 0;
    int i_theta = 0;

    double ms_to_solve_trjs = 0;
    for (double phi = phi0 - d_phi / 2; i_phi <= n_phi; phi = phi + d_phi / n_phi)
    {
      i_phi++;
      i_theta = 0;
      for (double theta = theta0 - d_theta / 2; i_theta <= n_theta; theta = theta + d_theta / n_theta)
      {
        i_theta++;
        double xf_sphere[6] = { 0, 0, 0, 0, 0, 0 };
        xf_sphere[0] = r * sin(theta) * cos(phi) + state_.pos.x;
        xf_sphere[1] = r * sin(theta) * sin(phi) + state_.pos.y;
        xf_sphere[2] = r * cos(theta) + state_.pos.z;

        double then = ros::Time::now().toSec();
        genNewTraj(u_max_, xf_sphere);  // Now X_ has the stuff
        ms_to_solve_trjs += 1000 * (ros::Time::now().toSec() - then);

        // trajs_sphere.markers.push_back(createMarkerLineStrip(X_));  // add marker to array
        createMarkerSetOfArrows(X_, &trajs_sphere);
      }
    }

    ROS_WARN("TOTAL SOLVE TIME trajs in sphere: %0.2f ms", ms_to_solve_trjs);

    pub_trajs_sphere_.publish(trajs_sphere);

    genNewTraj(u_max_, xf);  // Now X_ has the stuff
  }
}

void CVX::genNewTraj(double u_max, double xf[])
{
  double x0[6] = {
    quadGoal_.pos.x, quadGoal_.pos.y, quadGoal_.pos.z, quadGoal_.vel.x, quadGoal_.vel.y, quadGoal_.vel.z
  };
  double u0[3] = { quadGoal_.accel.x, quadGoal_.accel.y, quadGoal_.accel.z };

  double then = ros::Time::now().toSec();
  // Call optimizer
  double dt = callOptimizer(u_max, x0, xf);
  ROS_WARN("solve time: %0.2f ms", 1000 * (ros::Time::now().toSec() - then));

  double** x = get_state();
  double** u = get_control();

  then = ros::Time::now().toSec();
  interpInput(dt, xf, u0, x0, u, x, U_, X_);
  ROS_WARN("interp time: %0.2f ms", 1000 * (ros::Time::now().toSec() - then));

  replan_ = true;
  optimized_ = true;
  // ROS_INFO("%0.2f %0.2f %0.2f", x[N_-1][0], x[N_-1][1], x[N_-1][2]);
  // ROS_INFO("%0.2f %0.2f %0.2f", X(X.rows()-1,0), X(X.rows()-1,1), X(X.rows()-1,2));

  pubTraj(X_);
  // pubTraj(x);
}

void CVX::interpInput(double dt, double xf[], double u0[], double x0[], double** u, double** x, Eigen::MatrixXd& U,
                      Eigen::MatrixXd& X)
{
  // linearly interpolate between control input from optimizer
  double dc = 0.01;
  int size = (int)(N_)*dt / dc;
  U = Eigen::MatrixXd::Zero(size, 6);
  X = Eigen::MatrixXd::Zero(size, 6);

  int j = 1;
  double s[3] = { (u[1][0] - u0[0]) / dt, (u[1][1] - u0[1]) / dt, (u[1][2] - u0[2]) / dt };

  for (int i = 0; i < size; i++)
  {
    if (i > 0 && dc * (i - 1) >= dt * (j))
    {
      j++;
      s[0] = (u[j][0] - u[j - 1][0]) / dt;
      s[1] = (u[j][1] - u[j - 1][1]) / dt;
      s[2] = (u[j][2] - u[j - 1][2]) / dt;
    }

    if (j == 1)
    {
      U(i, 0) = s[0] * (dc * i) + u0[0];
      U(i, 1) = s[1] * (dc * i) + u0[1];
      U(i, 2) = s[2] * (dc * i) + u0[2];
    }
    else
    {
      U(i, 0) = s[0] * (dc * i - dt * (j - 1)) + u[j - 1][0];
      U(i, 1) = s[1] * (dc * i - dt * (j - 1)) + u[j - 1][1];
      U(i, 2) = s[2] * (dc * i - dt * (j - 1)) + u[j - 1][2];
    }
  }

  for (int i = 0; i < size - 1; i++)
  {
    U(i, 3) = (U(i + 1, 0) - U(i, 0)) / dc;
    U(i, 4) = (U(i + 1, 1) - U(i, 1)) / dc;
    U(i, 5) = (U(i + 1, 2) - U(i, 2)) / dc;
  }

  U.row(size - 1) << 0, 0, 0, 0, 0, 0;

  int k = 1;
  double p[3] = { (x[1][0] - x0[0]) / dt, (x[1][1] - x0[1]) / dt, (x[1][2] - x0[2]) / dt };
  double v[3] = { (x[1][3] - x0[3]) / dt, (x[1][4] - x0[4]) / dt, (x[1][5] - x0[5]) / dt };

  for (int i = 0; i < size; i++)
  {
    if (i > 0 && dc * (i - 1) >= dt * k)
    {
      k++;
      p[0] = (x[k][0] - x[k - 1][0]) / dt;
      p[1] = (x[k][1] - x[k - 1][1]) / dt;
      p[2] = (x[k][2] - x[k - 1][2]) / dt;
      v[0] = (x[k][3] - x[k - 1][3]) / dt;
      v[1] = (x[k][4] - x[k - 1][4]) / dt;
      v[2] = (x[k][5] - x[k - 1][5]) / dt;
    }

    if (k == 1)
    {
      X(i, 0) = p[0] * (dc * i) + x0[0];
      X(i, 1) = p[1] * (dc * i) + x0[1];
      X(i, 2) = p[2] * (dc * i) + x0[2];
      X(i, 3) = v[0] * (dc * i) + x0[3];
      X(i, 4) = v[1] * (dc * i) + x0[4];
      X(i, 5) = v[2] * (dc * i) + x0[5];
    }
    else
    {
      X(i, 0) = p[0] * (dc * (i)-dt * (k - 1)) + x[k - 1][0];
      X(i, 1) = p[1] * (dc * (i)-dt * (k - 1)) + x[k - 1][1];
      X(i, 2) = p[2] * (dc * (i)-dt * (k - 1)) + x[k - 1][2];
      X(i, 3) = v[0] * (dc * (i)-dt * (k - 1)) + x[k - 1][3];
      X(i, 4) = v[1] * (dc * (i)-dt * (k - 1)) + x[k - 1][4];
      X(i, 5) = v[2] * (dc * (i)-dt * (k - 1)) + x[k - 1][5];
    }
  }

  for (int i = 0; i < 6; i++)
  {
    X(size - 1, i) = xf[i];
  }

  // for (int i=1;i<21;i++){
  //   ROS_INFO("%0.3f %0.3f %0.3f",x[i][0],x[i][3],u[i][0]);
  // }

  // std::cout << std::endl;

  // for (int i=0;i<size;i++){
  //   ROS_INFO("%0.3f %0.3f",X(i,0),X(i,3));
  // }
}

double CVX::callOptimizer(double u_max, double x0[], double xf[])
{
  bool converged = false;
  // TODO: Set initial dt as a function of xf, x0 and u_max. Be careful because u_max can be accel, jerk,...
  double dt = 0.08;
  double** x;

  while (!converged)
  {
    load_default_data(dt, u_max, x0, xf);
    int r = optimize();
    if (r == 1)
    {
      x = get_state();
      int s = checkConvergence(xf, x[N_]);
      if (s == 1)
        converged = true;
      else
        dt += 0.025;
    }
    else
      dt += 0.025;
  }

  ROS_INFO("converged, dt = %0.2f", dt);

  return dt;
}

int CVX::checkConvergence(double xf[], double xf_opt[])
{
  int r = 0;
  float d = 0;
  float dv = 0;

  for (int i = 0; i < 3; i++)
  {
    d += pow(xf[i] - xf_opt[i], 2);
    dv += pow(xf[i + 3] - xf_opt[i + 3], 2);
  }

  d = sqrt(d);
  dv = sqrt(dv);

  if (d < 0.2 && dv < 0.2)
    r = 1;
  else
    r = 0;

  return r;
}

void CVX::modeCB(const acl_msgs::QuadFlightMode& msg)
{
  if (msg.mode == msg.LAND && flight_mode_.mode != flight_mode_.LAND)
  {
    double xf[6] = { quadGoal_.pos.x, quadGoal_.pos.y, z_land_, 0, 0, 0 };
    genNewTraj(u_min_, xf);
  }
  flight_mode_.mode = msg.mode;
}

void CVX::stateCB(const acl_msgs::State& msg)
{
  state_ = msg;
  // Stop updating when we get GO
  if (flight_mode_.mode == flight_mode_.NOT_FLYING || flight_mode_.mode == flight_mode_.KILL)
  {
    quadGoal_.pos = msg.pos;
    quadGoal_.vel = msg.vel;
    z_start_ = msg.pos.z;
    z_start_ = std::max(0.0, z_start_);
  }
}

void CVX::pubCB(const ros::TimerEvent& e)
{
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

  quadGoal_.vel.x = 0;
  quadGoal_.vel.y = 0;
  quadGoal_.vel.z = 0;
  quadGoal_.accel.x = 0;
  quadGoal_.accel.y = 0;
  quadGoal_.accel.z = 0;
  quadGoal_.jerk.x = 0;
  quadGoal_.jerk.y = 0;
  quadGoal_.jerk.z = 0;

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

  static int k = 0;
  if (optimized_ && flight_mode_.mode != flight_mode_.NOT_FLYING && flight_mode_.mode != flight_mode_.KILL)
  {
    quadGoal_.cut_power = false;
    if (replan_)
    {
      k = 0;
      replan_ = false;
    }

    if (k < X_.rows())
    {
      quadGoal_.pos.x = X_(k, 0);
      quadGoal_.pos.y = X_(k, 1);
      quadGoal_.pos.z = X_(k, 2);
      quadGoal_.vel.x = X_(k, 3);
      quadGoal_.vel.y = X_(k, 4);
      quadGoal_.vel.z = X_(k, 5);
      quadGoal_.accel.x = use_ff_ * U_(k, 0);
      quadGoal_.accel.y = use_ff_ * U_(k, 1);
      quadGoal_.accel.z = use_ff_ * U_(k, 2);
      quadGoal_.jerk.x = use_ff_ * U_(k, 3);
      quadGoal_.jerk.y = use_ff_ * U_(k, 4);
      quadGoal_.jerk.z = use_ff_ * U_(k, 5);
      k++;
    }
    else
    {
      quadGoal_.pos.x = X_(X_.rows() - 1, 0);
      quadGoal_.pos.y = X_(X_.rows() - 1, 1);
      quadGoal_.pos.z = X_(X_.rows() - 1, 2);
      quadGoal_.vel.x = X_(X_.rows() - 1, 3);
      quadGoal_.vel.y = X_(X_.rows() - 1, 4);
      quadGoal_.vel.z = X_(X_.rows() - 1, 5);
    }
  }
  else
  {
    quadGoal_.cut_power = true;
  }

  pub_goal_.publish(quadGoal_);

  // Pub setpoint maker
  setpoint_.header.stamp = ros::Time::now();
  setpoint_.pose.position.x = quadGoal_.pos.x;
  setpoint_.pose.position.y = quadGoal_.pos.y;
  setpoint_.pose.position.z = quadGoal_.pos.z;
  pub_setpoint_.publish(setpoint_);
}

void CVX::pubTraj(double** x)
{
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

void CVX::createMarkerSetOfArrows(Eigen::MatrixXd X, visualization_msgs::MarkerArray* trajs_sphere)
{
  bool isFree = trajIsFree(X);

  geometry_msgs::Point p_last;
  p_last.x = state_.pos.x;
  p_last.y = state_.pos.y;
  p_last.z = state_.pos.z;
  for (int i = 0; i < X.rows(); i = i + 10)  // Push (a subset of) the points in the trajectory
  {
    markerID_++;
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.id = markerID_;
    if (isFree)
    {
      m.color.r = 0.5;
      m.color.g = 0.7;
      m.color.b = 1;
      m.color.a = 1;
    }
    else
    {
      m.color.r = 1;
      m.color.g = 0;
      m.color.b = 0;
      m.color.a = 1;
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
    (*trajs_sphere).markers.push_back(m);
    p_last = p;
  }

  markerID_last_ = markerID_;

  // m.ns = "RAVEN_wall";
  // m.lifetime = ros::Duration(5);  // 3 second duration
}

void CVX::clearMarkerSetOfArrows()
{
  visualization_msgs::MarkerArray tmp;
  for (int i = markerID_; i <= markerID_last_; i++)
  {
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::DELETE;
    m.id = i;
    tmp.markers.push_back(m);
  }

  pub_trajs_sphere_.publish(tmp);
}

std::vector<int> v_kdtree_new_pcls_;

/*void CVX::pclCB()
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_tmp;
  kdtree_tmp.setInputCloud(msg);
  v_kdtree_new_pcls_.push_back(kdtree_tmp); // vector that has all the kdtrees of the new point clouds
}*/

void CVX::mapCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg)
{
  if (pcl2ptr_msg->width == 0 || pcl2ptr_msg->height == 0)  // Point Cloud is empty (this happens at the beginning)
  {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*pcl2ptr_msg, *pclptr);

  // reset v_kdtree_new_pcls_;
  kdtree_map_.setInputCloud(pclptr);
  kdtree_map_initialized_ = 1;
}

bool CVX::trajIsFree(Eigen::MatrixXd X)
{
  if (!kdtree_map_initialized_)
  {
    return true;
  }
  float radius_drone = 0.2;  // TODO: this should be a parameter

  // TODO: this cloud should be a subset of the entire cloud, not all the cloud (faster?)

  for (int i = 0; i < X.rows(); i = i + 1)  // Sample (a subset of) the points in the trajectory
  {
    pcl::PointXYZ searchPoint(X(i, 0), X(i, 1), X(i, 2));
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // TODO: maybe nearestKSearch is faster

    /*    // Search for collision in all the new point clouds
        for (std::vector<pcl::KdTreeFLANN<pcl::PointXYZ>>::iterator it = v_kdtree_new_pcls_.begin();
             it != v_kdtree_new_pcls_.end(); ++it)
        {
          // If there is at least one neigbour inside the radius=radius_drone
          if ((*it).radiusSearch(searchPoint, radius_drone, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
          {
            return false;  // if we model the drone as an sphere, I'm done (there is a collision)
          }
        }*/

    // Search for collision in the map
    if (kdtree_map_.radiusSearch(searchPoint, radius_drone, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      return false;  // if we model the drone as an sphere, I'm done (there is a collision)
    }
  }
  return true;
}

/*
visualization_msgs::Marker CVX::createMarkerLineStrip(Eigen::MatrixXd X)
{
  static int markerID = 0;
  markerID++;
  visualization_msgs::Marker m;
  m.id = markerID;
  m.color.r = 0.5;
  m.color.g = 0.7;
  m.color.b = 1;
  m.color.a = 1;
  m.scale.x = 0.01;
  m.pose.position.x = 1;
  m.pose.position.y = 2;
  m.pose.position.z = 5;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;

  m.header.frame_id = "world";
  m.header.stamp = ros::Time::now();

  geometry_msgs::Point p;
  p.x = 5;
  p.y = 9;
  p.z = 7;

  for (int i = 0; i < X.rows(); i++)  // Push all the points that will be in the linestrip
  {
    geometry_msgs::Point p;
    p.x = X(i, 0);
    p.y = X(i, 1);
    p.z = X(i, 2);
    m.points.push_back(p);
  }

  // m.ns = "RAVEN_wall";
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = visualization_msgs::Marker::ADD;
  // m.lifetime = ros::Duration(5);  // 3 second duration
  return m;
}

*/