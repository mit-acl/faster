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

// TODO: This values should be the same as the global_mapper.yaml
#define WDX 60    //[m] world dimension in x
#define WDY 60    //[m] world dimension in y
#define WDZ 6     //[m] world dimension in z
#define RES 0.35  //[m]

#define RED 1
#define GREEN 2
#define BLUE 3
#define BLUE_LIGHT 4
#define YELLOW 5
#define ORANGE_TRANS 6

#define DC 0.01           //(seconds) Duration for the interpolation=Value of the timer pubGoal
#define GOAL_RADIUS 0.2   //(m) Drone has arrived to the goal when distance_to_goal<GOAL_RADIUS
#define DRONE_RADIUS 0.3  //(m) Used for collision checking

CVX::CVX(ros::NodeHandle nh, ros::NodeHandle nh_replan_CB) : nh_(nh), nh_replan_CB_(nh_replan_CB)
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

  pubGoalTimer_ = nh_.createTimer(ros::Duration(DC), &CVX::pubCB, this);

  pubGoalReplan_ = nh_replan_CB_.createTimer(ros::Duration(3 * DC), &CVX::replanCB, this);

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
  setpoint_.color = color(ORANGE_TRANS);

  initialize_optimizer();

  term_goal_.pos.x = 0;
  term_goal_.pos.y = 0;
  term_goal_.pos.z = 0;

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
  printf("In solveJPSD\n");
  // Create a map
  const Vec3f start(state_.pos.x, state_.pos.y, state_.pos.z);
  const Vec3f goal(term_goal_.pos.x, term_goal_.pos.y, term_goal_.pos.z);
  const Vec3i dim(cells_x_, cells_y_, cells_z_);  //  number of cells in each dimension
  printf("In solveJPSD2\n");
  const Vec3f center_map(state_.pos.x, state_.pos.y, state_.pos.z);  // position of the drone
  // Read the pointcloud
  printf("In solveJPSD2.5\n");
  MapReader<Vec3i, Vec3f> reader(pclptr, dim, RES, center_map);  // Map read

  printf("In solveJPSD3\n");

  std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();
  map_util->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());
  printf("In solveJPSD3.5\n");

  std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(true));  // Declare a planner
  printf("In solveJPSD5\n");
  planner_ptr->setMapUtil(map_util);  // Set collision checking function
  planner_ptr->updateMap();

  printf("In solveJPSD6\n");
  Timer time_jps(true);
  bool valid_jps = planner_ptr->plan(
      start, goal, 1, true);  // Plan from start to goal with heuristic weight=1, and using JPS (if false --> use A*)
  printf("In solveJPSD6.5\n");
  if (valid_jps == true)  // There is a solution
  {
    double dt_jps = time_jps.Elapsed().count();
    printf("JPS Planner takes: %f ms\n", dt_jps);
    printf("JPS Path Distance: %f\n", total_distance3f(planner_ptr->getRawPath()));
    printf("JPS Path: \n");
    vec_Vecf<3> path_jps_vector = planner_ptr->getPath();

    for (const auto& it : path_jps_vector)
    {
      std::cout << it.transpose() << std::endl;
    }
    path_jps_ = clearArrows();
    vectorOfVectors2MarkerArray(path_jps_vector, &path_jps_);
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

  Eigen::Vector3d force = computeForce(curr_pos, term_goal);
  if (replanning_needed_ == false || dist_to_goal < GOAL_RADIUS)
  {
    // printf("No replanning needed\n");
    return;
  }

  double x = force[0], y = force[1], z = force[2];  // x,y,z of the goal with respecto to the current pos
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
      genNewTraj(u_max_, xf_sphere);  // Now X_temp_ has the stuff
      bool isFree = trajIsFree(X_temp_);
      createMarkerSetOfArrows(X_temp_, isFree);
      if (isFree)
      {
        X_ = X_temp_;
        U_ = U_temp_;
        replan_ = true;
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

void CVX::genNewTraj(double u_max, double xf[])
{
  // printf("In genNewTraj\n");
  double x0[6] = {
    quadGoal_.pos.x, quadGoal_.pos.y, quadGoal_.pos.z, quadGoal_.vel.x, quadGoal_.vel.y, quadGoal_.vel.z
  };
  double u0[3] = { quadGoal_.accel.x, quadGoal_.accel.y, quadGoal_.accel.z };

  double then = ros::Time::now().toSec();
  // Call optimizer
  double dt = callOptimizer(u_max, x0, xf);
  // ROS_WARN("solve time: %0.2f ms", 1000 * (ros::Time::now().toSec() - then));
  double** x = get_state();
  double** u = get_control();

  then = ros::Time::now().toSec();
  interpInput(dt, xf, u0, x0, u, x, U_temp_, X_temp_);
  // ROS_WARN("interp time: %0.2f ms", 1000 * (ros::Time::now().toSec() - then));

  optimized_ = true;
  // ROS_INFO("%0.2f %0.2f %0.2f", x[N_-1][0], x[N_-1][1], x[N_-1][2]);
  // ROS_INFO("%0.2f %0.2f %0.2f", X(X.rows()-1,0), X(X.rows()-1,1), X(X.rows()-1,2));
  // pubTraj(x);
}

void CVX::interpInput(double dt, double xf[], double u0[], double x0[], double** u, double** x, Eigen::MatrixXd& U,
                      Eigen::MatrixXd& X)
{
  // printf("In interpInput\n");
  // linearly interpolate between control input from optimizer
  double dc = DC;
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
  // printf("In callOptimizer\n");
  bool converged = false;
  // TODO: Be careful because u_max can be accel, jerk,...Also, there may be constraints in vel_max if input is accel
  float accelx = copysign(1, xf[0] - x0[0]) * u_max;
  float accely = copysign(1, xf[1] - x0[1]) * u_max;
  float accelz = copysign(1, xf[2] - x0[2]) * u_max;
  float v0x = x0[3];
  float v0y = x0[4];
  float v0z = x0[5];
  float tx = solvePolyOrder2(Eigen::Vector3f(0.5 * accelx, v0x, x0[0] - xf[0]));  // Solve equation xf=x0+v0t+0.5*a*t^2
  float ty = solvePolyOrder2(Eigen::Vector3f(0.5 * accely, v0y, x0[1] - xf[1]));
  float tz = solvePolyOrder2(Eigen::Vector3f(0.5 * accelz, v0z, x0[2] - xf[2]));
  float dt_initial = std::max({ tx, ty, tz }) / N_;
  // ROS_INFO("dt I found= %0.2f", dt_found);
  double dt = dt_initial;  // 0.025
  double** x;
  int i = 0;

  while (!converged)
  {
    load_default_data(dt, u_max, x0, xf);
    int r = optimize();
    i = i + 1;
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

  // ROS_INFO("Iterations = %d\n", i);
  // ROS_INFO("converged, dt = %0.2f", dt);
  // printf("difference= %0.2f\n", dt - dt_initial);

  return dt;
}

int CVX::checkConvergence(double xf[], double xf_opt[])
{
  // printf("In checkConvergence\n");
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
  // printf("In modeCB\n");
  if (msg.mode == msg.LAND && flight_mode_.mode != flight_mode_.LAND)
  {
    double xf[6] = { quadGoal_.pos.x, quadGoal_.pos.y, z_land_, 0, 0, 0 };
    genNewTraj(u_min_, xf);
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
      k = 0;  // Start again publishing the waypoints in X_ from the first row
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

  // printf("publishing quadGoal_ %0.2f\n", quadGoal_.pos.x);

  pub_goal_.publish(quadGoal_);

  // Pub setpoint maker.  setpoint_ is the last quadGoal sent to the drone
  setpoint_.header.stamp = ros::Time::now();
  setpoint_.pose.position.x = quadGoal_.pos.x;
  setpoint_.pose.position.y = quadGoal_.pos.y;
  setpoint_.pose.position.z = quadGoal_.pos.z;
  pub_setpoint_.publish(setpoint_);
  // printf("#########Time in pubCB %0.2f ms\n", 1000 * (ros::Time::now().toSec() - t0pubCB));
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
  //  printf("In pubTraj\n");

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

void CVX::mapCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg)
{
  // printf("In mapCB\n");

  if (pcl2ptr_msg->width == 0 || pcl2ptr_msg->height == 0)  // Point Cloud is empty (this happens at the beginning)
  {
    return;
  }
  mtx.lock();
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_msg, *pclptr);

  for (unsigned i = 0; i < v_kdtree_new_pcls_.size(); ++i)
  {
    if (v_kdtree_new_pcls_[i].time <= pcl2ptr_msg->header.stamp)  // if the map already contains that point cloud...
    {
      v_kdtree_new_pcls_.erase(v_kdtree_new_pcls_.begin() + i);  // ...erase that point cloud from the vector
      i = i - 1;  // Needed because I'm changing the vector inside the loop
    }
  }

  solveJPS3D(pclptr);
  kdtree_map_.setInputCloud(pclptr);
  kdtree_map_initialized_ = 1;
  mtx.unlock();
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
  //  printf("In computeForce\n");

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

// coeff is from highest degree to lowest degree. Returns the smallest positive real solution. Returns a BIG number if a
// root is imaginary or if it's negative
float CVX::solvePolyOrder2(Eigen::Vector3f coeff)
{
  // printf("In solvePolyOrder2\n");

  // std::cout << "solving\n" << coeff << std::endl;
  float a = coeff[0];
  float b = coeff[1];
  float c = coeff[2];
  float dis = b * b - 4 * a * c;
  if (dis >= 0)
  {
    float x1 = (-b - sqrt(dis)) / (2 * a);
    float x2 = (-b + sqrt(dis)) / (2 * a);

    if (x1 > 0)
    {
      return x1;
    }
    if (x2 > 0)
    {
      return x2;
    }
  }
  ROS_ERROR("No solution found to the equation");
  return std::numeric_limits<float>::max();
}

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
/*
void CVX::goalCB(const acl_msgs::TermGoal& msg)
{
  // Prevents race condition with lower accel landing
  if (msg.pos.x != 0.0 || msg.pos.y != 0)
  {
    double xf[6] = { msg.pos.x, msg.pos.y, msg.pos.z, msg.vel.x, msg.vel.y, msg.vel.z };

    // TODO: these should be parameters
    // Sample positions in an sphere using spherical coordinates

    double r = 8;     // radius of the sphere
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
    double then_cchecking = ros::Time::now().toSec();
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
        // TODO: ignore the trajectory if xf_sphere[2]<0 (below the ground)??
        double then = ros::Time::now().toSec();
        genNewTraj(u_max_, xf_sphere);  // Now X_ has the stuff
        ms_to_solve_trjs += 1000 * (ros::Time::now().toSec() - then);

        // trajs_sphere.markers.push_back(createMarkerLineStrip(X_));  // add marker to array
        createMarkerSetOfArrows(X_, &trajs_sphere);
      }
    }

    double ms_cchecking = 1000 * (ros::Time::now().toSec() - then_cchecking) - ms_to_solve_trjs;
    ROS_WARN("TOTAL SOLVE TIME trajs in sphere: %0.2f ms", ms_to_solve_trjs);
    ROS_WARN("TOTAL SOLVE TIME Col.Checking+Create Markers: %0.2f ms", ms_cchecking);

    pub_trajs_sphere_.publish(trajs_sphere);


    // genNewTraj(u_max_, xf);  // Now X_ has the stuff
  }
}*/

/*
  //////////////////////////////////////////////////////
  if (replanning_needed_ == false)
  {
    printf("Not replanning needed\n");
    return;
  }
  if (!kdtree_map_initialized_)
  {
    ROS_WARN("Run the mapper");
    return;
  }
  // printf("In replan CB\n");

  double dist_to_goal = sqrt(pow(term_goal_.pos.x - state_.pos.x, 2) + pow(term_goal_.pos.y - state_.pos.y, 2) +
                             pow(term_goal_.pos.z - state_.pos.z, 2));

  if (dist_to_goal < GOAL_RADIUS)
  {
    return;  // no replan if I'm in the goal
  }

  // TODO: r_sphere_max should be a parameter
  double r_sphere_max = 4.0;
  double r = std::min(dist_to_goal, r_sphere_max);  // radius of the sphere
  int n_phi = 6;                                    // Number of samples in phi.
  int n_theta = 6;                                  // Number of samples in theta.
  double d_theta = 3.14 / 2;
  double d_phi = 3.14 / 2;
  int i_phi = 0;
  int i_theta = 0;
  Eigen::Vector3d curr_pos(state_.pos.x, state_.pos.y, state_.pos.z);
  Eigen::Vector3d term_goal(term_goal_.pos.x, term_goal_.pos.y, term_goal_.pos.z);

  // TODO: I'm using only the direction of the force, but not the magnitude. Could I use the magnitude?
  // TODO: If I'm only using the direction of the force, not sure if quadratic+linear separation is needed in the
  // attractive force
  Eigen::Vector3d force = computeForce(curr_pos, term_goal);

  double x = force[0], y = force[1], z = force[2];  // x,y,z of the goal with respecto to the current pos
  double theta0 = acos(z / (sqrt(x * x + y * y + z * z)));
  double phi0 = atan2(y, x);

  visualization_msgs::MarkerArray trajs_sphere;  // all the trajectories generated in the sphere
  markerID_ = 0;
  clearMarkerSetOfArrows();

  bool found_it = 0;

  double xx = term_goal[0] - state_.pos.x, yy = term_goal[1] - state_.pos.y,
         zz = term_goal[2] - state_.pos.z;  // x,y,z of the goal with respecto to the current pos
  theta0 = acos(zz / (sqrt(xx * xx + yy * yy + zz * zz)));
  phi0 = atan2(yy, xx);

  Eigen::AngleAxis<float> rot_z(phi0, Eigen::Vector3f::UnitZ());
  Eigen::AngleAxis<float> rot_y(theta0, Eigen::Vector3f::UnitY());

  // std::vector<Eigen::Vector3d> s_points;  // sampled points

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
      // s_points.push_back(p2);

      double xf_sphere[6] = { 0, 0, 0, 0, 0, 0 };
      xf_sphere[0] = p2[0];
      xf_sphere[1] = p2[1];
      xf_sphere[2] = p2[2];
      genNewTraj(u_max_, xf_sphere);  // Now X_ has the stuff
      createMarkerSetOfArrows(X_, &trajs_sphere);
    }
  }

  pub_trajs_sphere_.publish(trajs_sphere);

  double xf[6] = { term_goal_.pos.x, term_goal_.pos.y, term_goal_.pos.z,
                   term_goal_.vel.x, term_goal_.vel.y, term_goal_.vel.z };
  genNewTraj(u_max_, xf);  // Now X_ has the stuff*/

/*  double xf[6] = { term_goal_.pos.x, term_goal_.pos.y, term_goal_.pos.z,
                     term_goal_.vel.x, term_goal_.vel.y, term_goal_.vel.z };
    genNewTraj(u_max_, xf);  // Now X_ has the stuff*/

/*  for (double phi = phi0; i_phi <= n_phi && !found_it; phi = phi + d_phi / n_phi)
  {
    i_phi++;
    i_theta = 0;
    for (double theta = theta0; i_theta <= n_theta && !found_it; theta = theta + d_theta / n_theta)
    {
      i_theta++;
      double xf_sphere[6] = { 0, 0, 0, 0, 0, 0 };
      xf_sphere[0] = r * sin(theta) * cos(phi) + state_.pos.x;
      xf_sphere[1] = r * sin(theta) * sin(phi) + state_.pos.y;
      xf_sphere[2] = r * cos(theta) + state_.pos.z;
      if (xf_sphere[2] < 0)  // If below the ground, discard
      {
        break;
      }

      // TODO: also "negative" angles
      genNewTraj(u_max_, xf_sphere);  // Now X_ has the stuff
      if (trajIsFree(X_))
      {
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
  }*/

/*  double xx = term_goal[0] - state_.pos.x, yy = term_goal[1] - state_.pos.y,
         zz = term_goal[2] - state_.pos.z;  // x,y,z of the goal with respecto to the current pos
  theta0 = acos(zz / (sqrt(xx * xx + yy * yy + zz * zz)));
  phi0 = atan2(yy, xx);*/