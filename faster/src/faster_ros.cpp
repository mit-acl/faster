#include "faster_ros.hpp"

// this object is created in the faster_ros_node
FasterRos::FasterRos(ros::NodeHandle nh, ros::NodeHandle nh_replan_CB, ros::NodeHandle nh_pub_CB)
  : nh_(nh), nh_replan_CB_(nh_replan_CB), nh_pub_CB_(nh_pub_CB)
{
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

  ros::param::param<double>("cntrl/spinup_time", par_.spinup_time, 0.5);

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

  markerID_ = 0;

  // Publishers
  pub_goal_jackal_ = nh_.advertise<geometry_msgs::Twist>("goal_jackal", 1);
  pub_goal_ = nh_.advertise<acl_msgs::QuadGoal>("goal", 1);
  pub_traj_whole_ = nh_.advertise<nav_msgs::Path>("traj_whole", 1);
  pub_traj_safe_ = nh_.advertise<nav_msgs::Path>("traj_safe", 1);
  pub_setpoint_ = nh_.advertise<visualization_msgs::Marker>("setpoint", 1);
  pub_intersectionI_ = nh_.advertise<visualization_msgs::Marker>("intersection_I", 1);
  pub_point_G_ = nh_.advertise<geometry_msgs::PointStamped>("point_G", 1);
  pub_point_G_term_ = nh_.advertise<geometry_msgs::PointStamped>("point_G_term", 1);
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
  poly_whole_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("poly_whole", 1, true);
  poly_safe_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("poly_safe", 1, true);
  pub_jps_inters_ = nh_.advertise<geometry_msgs::PointStamped>("jps_intersection", 1);
  pub_log_ = nh_.advertise<acl_msgs::Cvx>("log_topic", 1);
  pub_traj_committed_colored_ = nh_.advertise<visualization_msgs::MarkerArray>("traj_committed_colored", 1);
  pub_traj_whole_colored_ = nh_.advertise<visualization_msgs::MarkerArray>("traj_whole_colored", 1);
  pub_traj_safe_colored_ = nh_.advertise<visualization_msgs::MarkerArray>("traj_safe_colored", 1);

  // Subscribers
  occup_grid_sub_.subscribe(nh_, "occup_grid", 1);
  unknown_grid_sub_.subscribe(nh_, "unknown_grid", 1);
  sync_.reset(new Sync(MySyncPolicy(1), occup_grid_sub_, unknown_grid_sub_));
  sync_->registerCallback(boost::bind(&FasterRos::mapCB, this, _1, _2));
  sub_goal_ = nh_.subscribe("/move_base_simple/goal", 1, &FasterRos::terminalGoalCB, this);
  sub_mode_ = nh_.subscribe("fastermode", 1, &FasterRos::modeCB, this);
  sub_state_ = nh_.subscribe("state", 1, &FasterRos::stateCB, this);
  // sub_odom_ = nh_.subscribe("odom", 1, &FasterRos::odomCB, this);

  // Timers
  pubCBTimer_ = nh_pub_CB_.createTimer(ros::Duration(par_.dc), &FasterRos::pubCB, this);
  replanCBTimer_ = nh_.createTimer(ros::Duration(par_.dc), &FasterRos::replanCB, this);

  // Markers
  setpoint_ = getMarkerSphere(0.35, ORANGE_TRANS);
  R_ = getMarkerSphere(0.35, ORANGE_TRANS);
  I_ = getMarkerSphere(0.35, YELLOW);
  E_ = getMarkerSphere(0.35, RED);
  M_ = getMarkerSphere(0.35, BLUE);
  H_ = getMarkerSphere(0.35, GREEN);
  A_ = getMarkerSphere(0.35, RED);

  // If you want another thread for the replanCB: replanCBTimer_ = nh_.createTimer(ros::Duration(par_.dc),
  // &FasterRos::replanCB, this);

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

  faster_ptr_ = std::unique_ptr<Faster>(new Faster(par_));

  ROS_INFO("Planner initialized");
}

void FasterRos::replanCB(const ros::TimerEvent& e)
{
  if (ros::ok())
  {
    vec_Vecf<3> JPS_safe;
    vec_Vecf<3> JPS_whole;
    vec_E<Polyhedron<3>> poly_safe;
    vec_E<Polyhedron<3>> poly_whole;
    std::vector<state> X_safe;
    std::vector<state> X_whole;

    faster_ptr_->replan(JPS_safe, JPS_whole, poly_safe, poly_whole, X_safe, X_whole);
    clearJPSPathVisualization(2);
    publishJPSPath(JPS_safe, JPS_SAFE);
    publishJPSPath(JPS_whole, JPS_WHOLE);

    publishPoly(poly_safe, SAFE);
    publishPoly(poly_whole, WHOLE);
    pubTraj(X_safe, SAFE_COLORED);
    pubTraj(X_whole, WHOLE_COLORED);
  }
}

void FasterRos::publishPoly(const vec_E<Polyhedron<3>>& poly, int type)
{
  // std::cout << "Going to publish= " << (poly[0].hyperplanes())[0].n_ << std::endl;
  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly);
  poly_msg.header.frame_id = "world";

  switch (type)
  {
    case SAFE:
      poly_safe_pub_.publish(poly_msg);
      break;
    case WHOLE:
      poly_whole_pub_.publish(poly_msg);
      break;
  }
}

void FasterRos::stateCB(const acl_msgs::State& msg)
{
  state state_tmp;
  state_tmp.setPos(msg.pos.x, msg.pos.y, msg.pos.z);
  state_tmp.setVel(msg.vel.x, msg.vel.y, msg.vel.z);
  state_tmp.setAccel(0.0, 0.0, 0.0);
  faster_ptr_->updateState(state_tmp);

  /*  double roll, pitch, yaw;
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
    mtx_state.unlock();*/

  // Stop updating when we get GO
  /*  if (flight_mode_.mode == flight_mode_.NOT_FLYING || flight_mode_.mode == flight_mode_.KILL)
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
    }*/

  /*  static int i = 0;
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
    log_.veloc_norm = vel.norm();*/
}

void FasterRos::modeCB(const faster_msgs::Mode& msg)
{
  faster_ptr_->changeMode(msg.mode);
}

void FasterRos::pubCB(const ros::TimerEvent& e)
{
  state next_goal;
  faster_ptr_->getNextGoal(next_goal);

  acl_msgs::QuadGoal quadGoal;
  // visualization_msgs::Marker setpoint;
  // Pub setpoint maker.  setpoint_ is the last quadGoal sent to the drone

  // printf("Publicando Goal=%f, %f, %f\n", quadGoal_.pos.x, quadGoal_.pos.y, quadGoal_.pos.z);

  quadGoal.pos = eigen2rosvector(next_goal.pos);
  quadGoal.vel = eigen2rosvector(next_goal.vel);
  quadGoal.accel = eigen2rosvector(next_goal.accel);
  quadGoal.jerk = eigen2rosvector(next_goal.jerk);
  quadGoal.dyaw = next_goal.dyaw;
  quadGoal.yaw = next_goal.yaw;
  quadGoal.header.stamp = ros::Time::now();
  quadGoal.header.frame_id = "world";

  pub_goal_.publish(quadGoal);

  setpoint_.header.stamp = ros::Time::now();
  setpoint_.pose.position.x = quadGoal.pos.x;
  setpoint_.pose.position.y = quadGoal.pos.y;
  setpoint_.pose.position.z = quadGoal.pos.z;

  pub_setpoint_.publish(setpoint_);
}

void FasterRos::clearJPSPathVisualization(int i)
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

void FasterRos::clearMarkerArray(visualization_msgs::MarkerArray* tmp, ros::Publisher* publisher)
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

void FasterRos::publishJPSPath(vec_Vecf<3>& path, int i)
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

void FasterRos::pubTraj(const std::vector<state>& data, int type)
{
  // Trajectory
  nav_msgs::Path traj;
  traj.poses.clear();
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = "world";

  geometry_msgs::PoseStamped temp_path;

  for (int i = 0; i < data.size(); i = i + 8)
  {
    temp_path.pose.position.x = data[i].pos(0);
    temp_path.pose.position.y = data[i].pos(0);
    temp_path.pose.position.z = data[i].pos(0);
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
    traj_committed_colored_ = stateVector2ColoredMarkerArray(data, type, par_.v_max);
    pub_traj_committed_colored_.publish(traj_committed_colored_);
  }

  if (type == WHOLE_COLORED)
  {
    traj_whole_colored_ = stateVector2ColoredMarkerArray(data, type, par_.v_max);
    pub_traj_whole_colored_.publish(traj_whole_colored_);
  }

  if (type == SAFE_COLORED)
  {
    traj_safe_colored_ = stateVector2ColoredMarkerArray(data, type, par_.v_max);
    pub_traj_safe_colored_.publish(traj_safe_colored_);
  }
}

void FasterRos::pubJPSIntersection(Eigen::Vector3d& inters)
{
  geometry_msgs::PointStamped p;
  p.header.frame_id = "world";
  p.point = eigen2point(inters);
  pub_jps_inters_.publish(p);
}

void FasterRos::pubActualTraj()
{
  static geometry_msgs::Point p_last = pointOrigin();

  state current_state;
  faster_ptr_->getState(current_state);
  Eigen::Vector3d act_pos = current_state.pos;

  /*  // mtx_G.lock();
    Eigen::Vector3d t_goal = G_;
    // mtx_G.unlock();
    float dist_to_goal = (t_goal - act_pos).norm();

    if (dist_to_goal < 2 * par_.goal_radius)
    {
      return;
    }
  */

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

/*void FasterRos::pubG(state G)
{
  geometry_msgs::PointStamped p;
  p.header.frame_id = "world";
  // mtx_G.lock();
  p.point = eigen2point(G.pos);
  // mtx_G.unlock();
  pub_point_G_.publish(p);
}*/

void FasterRos::clearMarkerActualTraj()
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

void FasterRos::clearMarkerColoredTraj()
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
void FasterRos::mapCB(const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_map_ros,
                      const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_unk_ros)
{
  std::cout << "In mapCB" << std::endl;
  // Occupied Space Point Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_map_ros, *pclptr_map);
  // Unknown Space Point Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_unk(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_unk_ros, *pclptr_unk);

  std::cout << "Going to update Map" << std::endl;

  faster_ptr_->updateMap(pclptr_map, pclptr_unk);
}

void FasterRos::pubState(const state& data, const ros::Publisher pub)
{
  geometry_msgs::PointStamped p;
  p.header.frame_id = "world";
  p.point = eigen2point(data.pos);
  pub.publish(p);
}

void FasterRos::terminalGoalCB(const geometry_msgs::PoseStamped& msg)
{
  state G_term;
  G_term.setPos(msg.pose.position.x, msg.pose.position.y, 1.0);  // TODO
  faster_ptr_->setTerminalGoal(G_term);

  state G;  // projected goal
  faster_ptr_->getG(G);

  pubState(G_term, pub_point_G_term_);
  pubState(G, pub_point_G_);

  clearMarkerActualTraj();
  // std::cout << "Exiting from goalCB\n";
}

// Odometry Callback (for the Jackal)
/*void FasterRos::odomCB(const nav_msgs::Odometry& msg)
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
  // printf("(State): %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", msg.pos.x, msg.pos.y, msg.pos.z, msg.vel.x,
  //     msg.vel.y, msg.vel.z);

  std::cout << bold << red << "IN ODOM CB:" << msg.pose.pose.orientation << reset << std::endl;
  std::cout << bold << red << "Yaw=" << yaw * 180 / 3.14 << reset << std::endl;
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
}*/