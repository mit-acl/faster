// Authors: Jesus Tordesillas
// Date: August 2018, December 2018

// TODO (from December 2018)
// Put several iterations to find dt (increasing dt in each iteration)
// right now the unkown space that cvx receives is only a sphere around the drone, not the whole real unknown
// space
// Set dt as a constant in the optimization problem (so that many constraints no se tengan que poner de nuevo) (what in
// Cvxgen is called parameter)

// When taking off, something weird happen (started to happen when I added the penalization to JPS in the obj function)
// Ahroa mismo las pointclouds no las estoy teniendo en cuenta
// Las point clouds y los maps se actualizan MUY lentamente!!
// Ver si lo de los splines est'a bien, los de los control points (no deber'ian cambiar al cambiar dt??)
// Por qu'e a veces hay obst'aculos dentro de los polytopes
// Ver si los values del archivo de parametros estan bien
// Quitar la pcl CB si las point clouds no las estoy usando --> Quitada
// Modificar el timer del maper? Ahora esta a 20 Hz
// Modificar lo que hice de ir comprobando celda a celda en el ray del mapper
// A veces (cuando hay muchos polytopes) gurobi no encuentra solucion. Creo que es por el dt, que es muy
// pequeno-->Implementar Loop
// Mirar a ver si el punto inicial de la whole trajectory se est'a cogiendo en la solucion conjunta de la iteracion
// previa
// La point cloud ya no la estoy usando --> quitarla del launch de la camara
// hay algo raro en el flight_status_, lo del takeoff est'a hand-coded ahora mismo

// COSAS QUE PONER EN EL PAPER
// Notese que la WHOLE trajectory NO empieza en la posicion actual, sino un poco más adelante!!
// El yaw lo voy a poner apuntando hacia la interseccion JPS-unkown space. NOOOO, quadGoal_ es mejor creo (como esta
// ahora)
// Hablar de los tipos de filtros que estoy usando en la depth image
// JPSk lo estoy corriendo desde la initCond_, NO desde la posicion actual del dron

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
  ros::param::param<bool>("~use_vel", par_.use_vel, true);

  ros::param::param<double>("~dc", par_.dc, 0.01);
  ros::param::param<double>("~goal_radius", par_.goal_radius, 0.2);
  ros::param::param<double>("~drone_radius", par_.drone_radius, 0.15);

  ros::param::param<int>("~N_safe", par_.N_safe, 10);
  ros::param::param<int>("~N_whole", par_.N_whole, 10);

  ros::param::param<double>("~factor_deltaTp_", par_.factor_deltaTp, 3);
  ros::param::param<double>("~factor_deltaT", par_.factor_deltaT, 3);
  ros::param::param<int>("~min_states_deltaTp", par_.min_states_deltaTp, 50);
  ros::param::param<int>("~min_states_deltaT", par_.min_states_deltaT, 50);

  ros::param::param<double>("~Ra", par_.Ra, 2.0);
  ros::param::param<double>("~Ra_max", par_.Ra_max, 2.5);
  ros::param::param<double>("~Rb", par_.Rb, 6.0);
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

  /*  ros::param::param<double>("~factor_initial_whole", par_.factor_initial_whole, 2.0);
    ros::param::param<double>("~factor_final_whole", par_.factor_final_whole, 2.0);
    ros::param::param<double>("~factor_increment_whole", par_.factor_increment_whole, 1.0);
    ros::param::param<double>("~factor_initial_safe", par_.factor_initial_safe, 2.0);
    ros::param::param<double>("~factor_final_safe", par_.factor_final_safe, 2.0);
    ros::param::param<double>("~factor_increment_safe", par_.factor_increment_safe, 1.0);*/

  ros::param::param<int>("~max_poly", par_.max_poly, 4);
  ros::param::param<double>("~dist_max_vertexes", par_.dist_max_vertexes, 1.5);

  ros::param::param<int>("~gurobi_threads", par_.gurobi_threads, 1);
  ros::param::param<int>("~gurobi_verbose", par_.gurobi_verbose, 0);

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

  if (par_.N_safe <= par_.max_poly + 2)
  {
    std::cout << bold << red << "Needed: N_safe>=max_poly+ 2 at least" << reset
              << std::endl;  // To decrease the probability of not finding a solution
    abort();
  }
  if (par_.N_whole <= par_.max_poly + 2)
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

  if (par_.Ra_max > (par_.wdx / 2.0) || (par_.Ra_max > par_.wdy / 2.0))
  {
    std::cout << bold << red << "Needed: par_.Ra_max > par_.wdx/2.0|| par_.Ra_max > par_.wdy/2.0" << reset
              << std::endl;  // To decrease the probability of not finding a solution
    abort();
  }

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
  flight_mode_.mode = flight_mode_.NOT_FLYING;

  pub_goal_ = nh_.advertise<acl_msgs::QuadGoal>("goal", 1);
  pub_point_G_ = nh_.advertise<geometry_msgs::PointStamped>("point_G", 1);
  pub_traj_whole_ = nh_.advertise<nav_msgs::Path>("traj_whole", 1);
  pub_traj_safe_ = nh_.advertise<nav_msgs::Path>("traj_safe", 1);

  pub_setpoint_ = nh_.advertise<visualization_msgs::Marker>("setpoint", 1);
  pub_intersectionI_ = nh_.advertise<visualization_msgs::Marker>("intersection_I", 1);
  pub_point_E_ = nh_.advertise<visualization_msgs::Marker>("point_E", 1);
  pub_point_R_ = nh_.advertise<visualization_msgs::Marker>("point_R", 1);
  pub_point_M_ = nh_.advertise<visualization_msgs::Marker>("point_M", 1);

  pub_actual_traj_ = nh_.advertise<visualization_msgs::Marker>("actual_traj", 1);
  pub_path_jps1_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps1", 1);
  pub_path_jps2_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps2", 1);
  pub_path_jps_whole_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps_whole", 1);
  pub_path_jps_safe_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps_safe", 1);

  cvx_whole_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("cvx_whole", 1, true);
  cvx_safe_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("cvx_safe", 1, true);

  pub_jps_inters_ = nh_.advertise<geometry_msgs::PointStamped>("jps_intersection", 1);

  pub_intersec_points_ = nh_.advertise<visualization_msgs::MarkerArray>("intersection_points", 1);

  pub_log_ = nh_.advertise<acl_msgs::Cvx>("log_topic", 1);

  // pub_planning_vis_ = nh_.advertise<visualization_msgs::MarkerArray>("planning_vis", 1);

  // pub_samples_safe_path_ = nh_.advertise<visualization_msgs::MarkerArray>("samples_safe_path", 1);

  // pub_trajs_sphere_ = nh_.advertise<visualization_msgs::MarkerArray>("trajs_sphere", 1);
  // pub_forces_ = nh_.advertise<visualization_msgs::MarkerArray>("forces", 1);

  // pub_point_I_ = nh_.advertise<visualization_msgs::Marker>("IntersectionJPS_", 1);

  // cvx_decomp_el_o_pub__ = nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array_occupied", 1, true);

  // cvx_decomp_poly_uo_pub_ =
  //    nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array_unk_and_occupied", 1, true);
  // cvx_decomp_el_uo2_pub__ = nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array_occupied2", 1, true);

  occup_grid_sub_.subscribe(nh_, "occup_grid", 1);
  unknown_grid_sub_.subscribe(nh_, "unknown_grid", 1);
  sync_.reset(new Sync(MySyncPolicy(1), occup_grid_sub_, unknown_grid_sub_));
  sync_->registerCallback(boost::bind(&CVX::mapCB, this, _1, _2));

  sub_goal_ = nh_.subscribe("term_goal", 1, &CVX::goalCB, this);
  sub_mode_ = nh_.subscribe("flightmode", 1, &CVX::modeCB, this);
  sub_state_ = nh_.subscribe("state", 1, &CVX::stateCB, this);

  // sub_vicon_ = nh_.subscribe("vicon", 1, &CVX::viconCB, this);

  // sub_map_ = nh_.subscribe("occup_grid", 1, &CVX::mapCB, this);
  // sub_unk_ = nh_.subscribe("unknown_grid", 1, &CVX::unkCB, this);
  // sub_frontier_ = nh_.subscribe("frontier_grid", 1, &CVX::frontierCB, this);
  // sub_pcl_ = nh_.subscribe("pcloud", 1, &CVX::pclCB, this);

  pubCBTimer_ = nh_pub_CB_.createTimer(ros::Duration(par_.dc), &CVX::pubCB, this);

  replanCBTimer_ = nh_.createTimer(ros::Duration(par_.dc), &CVX::replanCB, this);

  // If you want another thread for the replanCB: replanCBTimer_ = nh_.createTimer(ros::Duration(par_.dc),
  // &CVX::replanCB, this);

  // Initialize setpoint marker
  setpoint_.header.frame_id = "world";
  setpoint_.id = 0;
  setpoint_.type = visualization_msgs::Marker::SPHERE;
  setpoint_.scale.x = 0.35;
  setpoint_.scale.y = 0.35;
  setpoint_.scale.z = 0.35;
  setpoint_.color = color(ORANGE_TRANS);

  // Initialize R marker
  R_.header.frame_id = "world";
  R_.id = 0;
  R_.type = visualization_msgs::Marker::SPHERE;
  R_.scale.x = 0.10;
  R_.scale.y = 0.10;
  R_.scale.z = 0.10;
  R_.color = color(ORANGE_TRANS);

  // Initialize I marker
  I_.header.frame_id = "world";
  I_.id = 0;
  I_.type = visualization_msgs::Marker::SPHERE;
  I_.scale.x = 0.10;
  I_.scale.y = 0.10;
  I_.scale.z = 0.10;
  I_.color = color(GREEN);

  // Initialize T marker
  E_.header.frame_id = "world";
  E_.id = 0;
  E_.type = visualization_msgs::Marker::SPHERE;
  E_.scale.x = 0.10;
  E_.scale.y = 0.10;
  E_.scale.z = 0.10;
  E_.color = color(RED);

  // Initialize M marker
  M_.header.frame_id = "world";
  M_.id = 0;
  M_.type = visualization_msgs::Marker::SPHERE;
  M_.scale.x = 0.10;
  M_.scale.y = 0.10;
  M_.scale.z = 0.10;
  M_.color = color(BLUE);

  // mtx_G.lock();
  G_ << 0, 0, 0;
  // mtx_G.unlock();
  G_term_ << 0, 0, 0;

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

  /*  solver_vel_.setDC(par_.dc);
    solver_accel_.setDC(par_.dc);
    solver_jerk_.setDC(par_.dc);

    solver_vel_.setq(par_.q);
    solver_accel_.setq(par_.q);
    solver_jerk_.setq(par_.q);*/

  double max_values[3] = { par_.v_max, par_.a_max, par_.j_max };
  /*  solver_jerk_.set_max(max_values);*/

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

  std::cout << "Done Setups\n";

  /*  double max_values_vel[1] = { par_.v_max };
    solver_vel_.set_max(max_values_vel);*/

  pclptr_unk_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  pclptr_map_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  name_drone_ = ros::this_node::getNamespace();
  name_drone_.erase(0, 2);  // Erase slashes

  map_util_ = std::make_shared<VoxelMapUtil>();
  planner_ptr_ = std::unique_ptr<JPSPlanner3D>(new JPSPlanner3D(false));

  JPS_old_.clear();

  // My stuff goes here

  /*  Eigen::Vector3d E;
    vec_Vecf<3> JPSk;
    JPSk.push_back(Eigen::Vector3d(34.21, 35.74, 1.02));
    JPSk.push_back(Eigen::Vector3d(34.16, 36.30, 0.95));
    JPSk.push_back(Eigen::Vector3d(34.16, 38.10, 0.95));
    JPSk.push_back(Eigen::Vector3d(34.76, 38.70, 0.95));
    JPSk.push_back(Eigen::Vector3d(34.91, 38.70, 0.95));
    JPSk.push_back(Eigen::Vector3d(35.36, 39.00, 0.95));
    JPSk.push_back(Eigen::Vector3d(36.00, 40.50, 1.00));

    int li1;
    double ra = 5.56;
    bool noPointsOutsideSphere1;
    E = getFirstIntersectionWithSphere(JPSk, ra, JPSk[0], &li1, &noPointsOutsideSphere1);

    std::cout << "li=" << li1 << std::endl;
    std::cout << "noPointsOutsideSphere1=" << noPointsOutsideSphere1 << std::endl;
    vec_Vecf<3> JPSk_inside_sphere(JPSk.begin(), JPSk.begin() + li1 + 1);  // Elements of JPS that are inside the sphere

    if (noPointsOutsideSphere1 == false)
    {
      JPSk_inside_sphere.push_back(E);
    }

    std::cout << green << bold << "before creating more vertexes" << reset << std::endl;
    printElementsOfJPS(JPSk_inside_sphere);

    std::cout << green << bold << "E is " << reset << E.transpose() << std::endl;

    createMoreVertexes(JPSk_inside_sphere, par_.dist_max_vertexes);

    std::cout << "After creating more vertexes=" << std::endl;
    printElementsOfJPS(JPSk_inside_sphere);

    if (JPSk_inside_sphere.size() > par_.max_poly + 1)  // If I have more than (par_.max_poly + 1) vertexes
    {
      JPSk_inside_sphere.erase(JPSk_inside_sphere.begin() + par_.max_poly + 1,
                               JPSk_inside_sphere.end());  // Force JPS to have less than par_.max_poly elements
      E = JPSk_inside_sphere[JPSk_inside_sphere.size() - 1];
    }

    std::cout << "JPS used for whole is" << std::endl;
    printElementsOfJPS(JPSk_inside_sphere);*/

  // End of my stuff

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

/*void CVX::novale(const sensor_msgs::PointCloud2::ConstPtr& data1, const sensor_msgs::PointCloud2::ConstPtr& data2)
{
  // Solve all of perception here...
}
*/
/*void CVX::viconCB( acl_msgs::ViconState msg)
{
  accel_vicon_<<msg

}*/
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

void CVX::publishJPS2handIntersection(vec_Vecf<3> JPS2_fix, Eigen::Vector3d& inter1, Eigen::Vector3d& inter2,
                                      bool solvedFix)
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

  // std::cout << dim.transpose() << std::endl;
  /*  MapReader<Vec3i, Vec3f> reader(pclptr, cells_x_, cells_y_, cells_z_, par_.factor_jps * par_.res, center_map,
                                   par_.z_ground, par_.z_max,
                                   par_.inflation_jps);  // Map read*/

  mtx_jps_map_util.lock();

  map_util_->readMap(pclptr, cells_x_, cells_y_, cells_z_, par_.factor_jps * par_.res, center_map, par_.z_ground,
                     par_.z_max,
                     par_.inflation_jps);  // Map read

  // map_util_->info();

  // map_util_->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());

  mtx_jps_map_util.unlock();
}

vec_Vecf<3> CVX::solveJPS3D(Vec3f& start_sent, Vec3f& goal_sent, bool* solved, int i)
{
  Eigen::Vector3d start(start_sent(0), start_sent(1), std::max(start_sent(2), 0.0));
  Eigen::Vector3d goal(goal_sent(0), goal_sent(1), std::max(goal_sent(2), 0.0));

  Vec3f originalStart = start;

  if (flight_mode_.mode == flight_mode_.TAKEOFF)
  {
    std::cout << "flight_mode_.TAKEOFF=" << std::endl;
  }

  if (flight_mode_.mode != flight_mode_.GO || takeoff_done_ == false)
  {
    vec_Vecf<3> solution;
    solution.push_back(start);
    solution.push_back(goal);
    *solved = true;
    return solution;
  }

  pcl::PointXYZ pcl_start = eigenPoint2pclPoint(start);
  pcl::PointXYZ pcl_goal = eigenPoint2pclPoint(goal);

  ///////////////////////////////////////////////////////////////
  /////////////////////////// RUN JPS ///////////////////////////
  ///////////////////////////////////////////////////////////////

  mtx_jps_map_util.lock();

  // Set start and goal free
  const Veci<3> start_int = map_util_->floatToInt(start);
  const Veci<3> goal_int = map_util_->floatToInt(goal);
  map_util_->setFreeVoxelAndSurroundings(start_int, par_.inflation_jps);
  map_util_->setFreeVoxelAndSurroundings(goal_int, par_.inflation_jps);
  // map_util_->setFree(goal_int);

  planner_ptr_->setMapUtil(map_util_);  // Set collision checking function
  // planner_ptr_->updateMap();
  // printf("Out4\n");

  // MyTimer time_jps(true);
  // std::cout << "Planning from start=" << start << std::endl;

  double t0JPSk = ros::Time::now().toSec();
  bool valid_jps = planner_ptr_->plan(
      start, goal, 1, true);  // Plan from start to goal with heuristic weight=1, and using JPS (if false --> use A*)
                              // printf("       JPS solver takes: %f ms\n", (double)time_jps.ElapsedF().count());
                              /*  if (i == 1)
                                {
                                  log_.JPSk_ms = 1000 * (ros::Time::now().toSec() - t0JPSk);
                                }*/

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
    if (path.size() > 1)
    {
      path[0] = start;
      path[path.size() - 1] = goal;  // force to start and end in the start and goal (and not somewhere in the voxel)
    }
    else
    {  // happens when start and goal are very near (--> same cell)
      vec_Vecf<3> tmp;
      tmp.push_back(start);
      tmp.push_back(goal);
      path = tmp;
    }
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
    ROS_ERROR("JPS didn't find a solution from\n");
    std::cout << "start=" << start.transpose() << std::endl;
    std::cout << "goal=" << goal.transpose() << std::endl;
  }
  mtx_jps_map_util.unlock();

  *solved = valid_jps;
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
  Eigen::Vector3d temp(state_.pos.x, state_.pos.y, state_.pos.z);
  G_ = projectClickedGoal(temp);
  pubTerminalGoal();
  mtx_state.unlock();
  mtx_G.unlock();
  // std::cout << "G_=\n" << G_ << std::endl;

  status_ = (status_ == GOAL_REACHED) ? YAWING : TRAVELING;
  /*  if (status_ == YAWING)
    {
      // printf("GCB: status_ = YAWING\n");
    }
    if (status_ == TRAVELING)
    {
      // printf("GCB: status_ = TRAVELING\n");
    }*/
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
  double dyaw_not_filtered;

  /*  if (diff > 0)
      dyaw_not_filtered = par_.w_max;
    else
      dyaw_not_filtered = -par_.w_max;*/

  dyaw_not_filtered = copysign(1, diff) * par_.w_max;

  dyaw_filtered_ = (1 - par_.alpha_filter_dyaw) * dyaw_not_filtered + par_.alpha_filter_dyaw * dyaw_filtered_;
  quad_goal.dyaw = dyaw_filtered_;

  quad_goal.yaw += dyaw_filtered_ * par_.dc;

  /*  if(diff<0.1){//don't yaw if difference (in) is very small (Hysteresis)
      quad_goal.yaw=quad_goal.yaw;
      quad_goal.dyaw=0;
      return;
    }*/

  /*  std::cout<<bold<<green<<std::setprecision(10)<<"diff antes="<<diff<<reset<<std::endl;
      std::cout<<bold<<green<<std::setprecision(10)<<"par_.dc  "<<par_.dc<<reset<<std::endl;
    std::cout<<bold<<green<<std::setprecision(10)<<"par_.w_max= "<<par_.w_max<<reset<<std::endl;
    std::cout<<bold<<green<<std::setprecision(10)<<"-par_.dc * par_.w_max= "<<-par_.dc * par_.w_max<<reset<<std::endl;
    std::cout<<bold<<green<<std::setprecision(10)<<"par_.dc * par_.w_max= "<<par_.dc * par_.w_max<<reset<<std::endl;
    saturate(diff, -par_.dc * par_.w_max, par_.dc * par_.w_max);
    std::cout<<bold<<green<<std::setprecision(10)<<"diff despues="<<diff<<reset<<std::endl;

    double dyaw_not_filtered=copysign(1,diff)*par_.w_max;
    std::cout<<bold<<green<<"dyaw_not_filtered="<<dyaw_not_filtered<<reset<<std::endl;

    //Low pass filter for dyaw
    quad_goal.dyaw=(1-par_.alpha_filter_dyaw)*dyaw_not_filtered  +par_.alpha_filter_dyaw*quad_goal.dyaw;
    std::cout<<"quad_goal.dyaw="<<quad_goal.dyaw<<std::endl;

    quad_goal.yaw += quad_goal.dyaw*par_.dc;*/
}

vec_Vecf<3> CVX::fix(vec_Vecf<3>& JPS_old, Eigen::Vector3d& start, Eigen::Vector3d& goal, bool* solved)
{
  vec_Vecf<3> fix;
  vec_Vecf<3> JPS_old_original = JPS_old;
  bool thereIsIntersection = false;
  // std::cout << "*********In fix0.5" << std::endl;
  /*  vec_Vecf<3> null1(1, Eigen::Vector3d::Zero());
    vec_Vecf<3> null2(1, Eigen::Vector3d::Zero());
    vec_Vecf<3>& path_start2fix(null1);  //
    vec_Vecf<3>& path_fix2goal(null2);*/
  vec_Vecf<3> path_start2fix;  // referenceFs has to be initialized
  vec_Vecf<3> path_fix2goal;
  // std::cout << "*********In fix0.6" << std::endl;
  path_start2fix.clear();
  path_fix2goal.clear();
  vec_Vecf<3> path_fixed;

  // std::cout << "*********In fix0.7" << std::endl;
  Eigen::Vector3d inters1 = getFirstCollisionJPS(JPS_old, &thereIsIntersection, MAP,
                                                 RETURN_INTERSECTION);  // intersection starting from start
  // std::cout << "Here thereIsIntersection=" << thereIsIntersection << std::endl;
  // std::cout << "*********In fix2" << std::endl;

  if (thereIsIntersection)
  {
    // std::cout << "*********In fix2.5" << std::endl;
    clearJPSPathVisualization(2);
    vec_Vecf<3> tmp = JPS_old;
    // std::cout << "*****tmp is:" << std::endl;
    // printElementsOfJPS(tmp);
    std::reverse(tmp.begin(), tmp.end());  // flip all the vector
    Eigen::Vector3d inters2 = getFirstCollisionJPS(tmp, &thereIsIntersection, MAP,
                                                   RETURN_INTERSECTION);  // intersection starting from the goal

    // std::reverse(path_fix2goal.begin(), path_fix2goal.end());
    bool solvedFix, solvedStart2Fix, solvedFix2Goal;
    // std::cout << "*********In fix3" << std::endl;

    // printf("Calling to fix from\n");
    // std::cout << inters1.transpose() << std::endl << "to" << inters2.transpose() << std::endl;

    if ((inters1 - inters2).lpNorm<1>() > 0.01)  // Hack to delete corner cases TODO
    // if (inters1.isApprox(inters2, 0.01))
    {
      fix = solveJPS3D(inters1, inters2, &solvedFix, 2);
    }
    else
    {
      fix.push_back(inters1);
    }
    printf("AQUI2\n");

    if ((start - inters1).lpNorm<1>() > 0.01)  // Hack to delete corner cases TODO
    // if (start.isApprox(inters1, 0.01))
    {
      path_start2fix = solveJPS3D(start, inters1, &solvedStart2Fix, 2);
    }
    else
    {
      path_start2fix.push_back(start);
    }

    printf("AQUI3\n");

    if ((inters2 - goal).lpNorm<1>() > 0.01)  // Hack to delete corner cases TODO
    // if (inters2.isApprox(goal, 0.01))
    {
      path_fix2goal = solveJPS3D(inters2, goal, &solvedFix2Goal, 2);
    }
    else
    {
      path_fix2goal.push_back(goal);
    }

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
      path_fixed.insert(path_fixed.end(), fix.begin() + 1, fix.end());
      path_fixed.insert(path_fixed.end(), path_fix2goal.begin() + 1, path_fix2goal.end());
      /*      printf("***************Start to fix******************\n");
            printElementsOfJPS(path_start2fix);
            printf("***************Fix***************************\n");
            printElementsOfJPS(fix);
            printf("***************Fix to Goal***************************\n");
            printElementsOfJPS(path_fix2goal);
            printf("***************Everything***************************\n");
            printElementsOfJPS(path_fixed);*/
      if (par_.visual == true)
      {
        publishJPS2handIntersection(path_fixed, inters1, inters2, solved_complete_fix);
      }
    }

    // printf("Before publishing\n");

    // printf("published\n");

    // printf("Solved, fix=:\n");
    // printElementsOfJPS(fix);
  }

  else
  {
    printf("there is no intersection\n");
    /*   std::cout << "the start is " << start.transpose() << std::endl;
      std::cout << "the goal is " << goal.transpose() << std::endl;*/

    *solved = true;
    // JPS_old[0] = start;
    // JPS_old[JPS_old.size() - 1] = goal;

    /*    std::cout << "voy a copiar" << std::endl;
        printElementsOfJPS(JPS_old_original);*/

    std::copy(JPS_old_original.begin(), JPS_old_original.end(), back_inserter(path_fixed));  // Copy JPS_old into fix
    // fix = JPS_old;
    path_fixed[0] = start;
    path_fixed[path_fixed.size() - 1] = goal;

    /*    std::cout << "Lo copiado" << std::endl;
        printElementsOfJPS(path_fixed);*/

    if (par_.visual == true)
    {
      publishJPS2handIntersection(path_fixed, path_fixed[0], path_fixed[path_fixed.size() - 1], 1);
    }
  }
  // printf("finisshing fix\n");

  return path_fixed;
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

  double t0replanCB = ros::Time::now().toSec();
  // printf("replanCB: Before mtx_state!!!\n");
  mtx_state.lock();
  Eigen::Vector3d state_pos(state_.pos.x, state_.pos.y, state_.pos.z);  // Local copy of state
  Eigen::Vector3d state_vel(state_.vel.x, state_.vel.y, state_.vel.z);  // Local copy of state

  // std::cout << "state_pos=" << state_pos.transpose() << std::endl;
  mtx_state.unlock();

  // printf("replanCB: Before mtx_G!!!\n");
  mtx_G.lock();
  G_ = projectClickedGoal(state_pos);
  pubTerminalGoal();
  // std::cout << "Projected Goal" << G_.transpose() << std::endl;
  mtx_G.unlock();
  // printf("In replanCB0.1\n");
  if (par_.visual == true)
  {
    clearMarkerSetOfArrows();
    pubintersecPoint(Eigen::Vector3d::Zero(), false);  // Clear the intersection points markers
  }

  /*  mtx_G_term.lock();
    Eigen::Vector3d P2 = G_term_;
    mtx_G_term.unlock();*/

  // printf("In replanCB0.3\n");
  mtx_G.lock();
  Eigen::Vector3d G = G_;  // Local copy of the terminal goal
  mtx_G.unlock();

  mtx_G_term.lock();
  Eigen::Vector3d G_term = G_term_;  // Local copy of the terminal terminal goal
  mtx_G_term.unlock();

  double dist_to_goal = (G_term - state_pos).norm();

  double dist_to_goal_commanded = (G_term - state_pos).norm();

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
  // printf("Entering in replanCB, planner_status_=%d\n", planner_status_);
  // printf("In replanCB0.4s\n");
  if (status_ == GOAL_SEEN || status_ == GOAL_REACHED || planner_status_ == REPLANNED || status_ == YAWING)
  {
    // printf("No replanning needed because planner_status_=%d and/or status_=%d \n", planner_status_, status_);
    // printf("or because status_=%d\n", status_);
    return;
  }

  std::cout << bold << on_red << "************IN REPLAN CB*********" << reset << std::endl;
  print_status();

  // std::cout << "replanCB3\n" << std::endl;

  // printf("ReplanCB: Init takes %f ms\n", (double)time_init.Elapsed().count());

  // MyTimer time_med(true);
  ///////////////////////////////////////////////////////////////////////////////

  int li1;    // last index inside the sphere of JPSk
  int li2;    // last index inside the sphere of JPS2
  int liold;  // last index inside the sphere of JPS2

  int lia;      // last index inside the sphere of JPSa
  int lik_m_1;  // last index inside the sphere of JPS_k_m_1
  int lib;      // last index inside the sphere of JPSb

  double dist1 = 0, dist2 = 0, J1 = 0, J2 = 0, JPrimj1 = 0, JPrimj2 = 0, JPrimv1 = 0, JPrimv2 = 0, JDist1 = 0,
         JDist2 = 0;
  J1 = std::numeric_limits<double>::max();
  J2 = std::numeric_limits<double>::max();
  Eigen::MatrixXd U_temp1, U_temp2, X_temp1, X_temp2;

  // std::cout << "replanCB4\n" << std::endl;
  // printf("init2\n");
  // std::cout << "Running JPS3d from=" << state_pos.transpose() << std::endl;
  // std::cout << "Running JPS3d to terminal goal=" << G.transpose() << std::endl;

  double x0[9];
  if (X_initialized_)  // Needed to skip the first time (X_ still not initialized)
  {
    mtx_k.lock();

    log_.entered_safe_path = 0;
    if (k_ > deltaTp_ && status_ == TRAVELING)
    {
      ROS_WARN("Switched to the SAFE PATH!!");
      log_.entered_safe_path = 1;
    }

    mtx_offsets.lock();
    k_initial_cond_1_ = std::min(k_ + deltaT_, (int)(X_.rows() - 1));
    mtx_offsets.unlock();
    mtx_k.unlock();
    // printf("Ahora mismo, k_initial_cond_1=%d\n", k_initial_cond_1_);

    updateInitialCond(k_initial_cond_1_);

    mtx_initial_cond.lock();
    x0[0] = initialCond_.pos.x;
    x0[1] = initialCond_.pos.y;
    x0[2] = initialCond_.pos.z;
    x0[3] = initialCond_.vel.x;
    x0[4] = initialCond_.vel.y;
    x0[5] = initialCond_.vel.z;
    x0[6] = initialCond_.accel.x;
    x0[7] = initialCond_.accel.y;
    x0[8] = initialCond_.accel.z;
    mtx_initial_cond.unlock();
  }
  else
  {
    mtx_initial_cond.lock();

    x0[0] = state_pos(0);
    x0[1] = state_pos(1);
    x0[2] = state_pos(2);
    x0[3] = state_vel(0);
    x0[4] = state_vel(1);
    x0[5] = state_vel(2);
    x0[6] = 0;
    x0[7] = 0;
    x0[8] = 0;

    mtx_initial_cond.unlock();
  }

  Eigen::Vector3d InitPos;
  InitPos << x0[0], x0[1], x0[2];

  // std::cout << "InitPos" << InitPos.transpose() << std::endl;

  static bool first_time = true;  // how many times I've solved JPSk

  /*  std::cout << "Term Goal: *******************************" << std::endl;
    std::cout << G.transpose() << std::endl;*/

  double ra = std::min((dist_to_goal - 0.001), par_.Ra);

  /// MY STUFF ///////////////

  vec_Vecf<3> JPSa;
  vec_Vecf<3> JPSb;
  vec_Vecf<3> JPSk;
  double dist_a, dist_b, Ja, Jb;
  bool solvedjpsa = false, solvedjpsb = false;

  std::cout << bold << blue << "OtherStuff:  " << std::fixed << otherStuff_t << "ms\n" << reset;

  MyTimer timer_jps(true);
  if (first_time == true)
  {
    first_time = false;
    // std::cout << "Running JPS first time" << std::endl;
    JPSa = solveJPS3D(InitPos, G, &solvedjpsa, 1);
    log_.computed_both = 0;
    JPSk = JPSa;
  }
  else
  {
    // std::cout << "Running JPS other times" << std::endl;
    JPSa = solveJPS3D(InitPos, G, &solvedjpsa, 1);

    log_.JPSa_solved = solvedjpsa;
    log_.JPSa_ms = timer_jps.ElapsedMs();

    if (solvedjpsa == false)
    {
      std::cout << bold << red << "JPSa didn't find a solution" << reset << std::endl;
      return;
    }
    /*    std::cout << "This is JPSa" << std::endl;
        printElementsOfJPS(JPSa);*/

    Eigen::Vector3d C = getFirstIntersectionWithSphere(JPSa, ra, JPSa[0], &lia);
    Eigen::Vector3d C_old = getFirstIntersectionWithSphere(JPS_k_m_1_, ra, JPS_k_m_1_[0], &lik_m_1);

    Eigen::Vector3d v1 = C - JPSa[0];            // point i expressed with origin=origin sphere
    Eigen::Vector3d v2 = C_old - JPS_k_m_1_[0];  // point i minus 1
    double alpha = angleBetVectors(v1, v2);
    log_.angle_deg = alpha * 180 / 3.14;

    if (alpha <= par_.alpha_0_deg * 3.14 / 180)
    {  // No need to decide
      clearJPSPathVisualization(2);
      JPSk = JPSa;
      log_.decision = 1;
      log_.computed_both = 0;
    }
    else
    {
      // Going to decide
      std::cout << "Esto es lo que voy a arreglar:" << std::endl;
      printElementsOfJPS(JPS_k_m_1_);
      log_.computed_both = 1;

      bool solvedjpsb;

      MyTimer timer_jps_fix(true);
      JPSb = fix(JPS_k_m_1_, InitPos, G, &solvedjpsb);
      log_.JPS_fix_ms = timer_jps_fix.ElapsedMs();
      log_.JPS_fix_solved = solvedjpsb;
      if (solvedjpsb == true)
      {
        std::cout << "Esto es JPSb (lo arreglado):" << std::endl;
        printElementsOfJPS(JPSb);
        Eigen::Vector3d D = getFirstIntersectionWithSphere(JPSb, ra, JPSb[0], &lib);
        // std::cout << "Going to compute norm, la=" << lia << ", lib=" << lib << std::endl;
        dist_a = normJPS(JPSa, lia + 1);
        dist_b = normJPS(JPSb, lib + 1);

        double C_vector[9] = { C(0), C(1), C(2), 0, 0, 0, 0, 0, 0 };
        std::cout << "C=" << C.transpose() << " D=" << D.transpose() << std::endl;
        sg_whole_.setXf(C_vector);
        sg_whole_.findDT(1);

        double dta = sg_whole_.dt_;
        Ja = sg_whole_.N_ * dta + dist_a / par_.v_max;
        log_.Ja_inside = sg_whole_.N_ * dta;
        log_.Ja_outside = dist_a / par_.v_max;
        log_.Ja = Ja;
        // std::cout << "Norm computed3" << std::endl;

        double D_vector[9] = { D(0), D(1), D(2), 0, 0, 0, 0, 0, 0 };
        sg_whole_.setXf(D_vector);
        sg_whole_.findDT(1);
        // std::cout << "Norm computed4" << std::endl;
        double dtb = sg_whole_.dt_;
        Jb = sg_whole_.N_ * dtb + dist_b / par_.v_max;

        log_.Jb_inside = sg_whole_.N_ * dtb;
        log_.Jb_outside = dist_b / par_.v_max;
        log_.Jb = Jb;

        log_.decision = (Ja < Jb) ? 1 : 2;
        // Decision:
        JPSk = (Ja < Jb) ? JPSa : JPSb;
        std::cout << green << "Ja=  " << std::fixed << Ja << ", Jb=  " << std::fixed << Jb << reset << std::endl;
      }
      else
      {
        log_.decision = 1;
        std::cout << bold << red << "JPSb didn't find a solution" << reset << std::endl;
        JPSk = JPSa;
      }
    }
  }

  log_.JPStotal_ms = timer_jps.ElapsedMs();
  std::cout << bold << blue << "JPS (all):  " << std::fixed << timer_jps << "ms\n" << reset;

  // JPSk = JPSa;  // Quitar esto despues
  /*  std::cout << "Esto es JPS_k_m_1_:" << std::endl;
    printElementsOfJPS(JPS_k_m_1_);*/

  /*  std::cout << "Esto es JPSa:" << std::endl;
    printElementsOfJPS(JPSa);

    std::cout << "Esto es JPSk:" << std::endl;
    printElementsOfJPS(JPSk);*/
  JPS_k_m_1_ = JPSk;  // saved for the next iteration
  // solvedjps1 = true;

  ///////////////////////////

  ra = std::min((dist_to_goal - 0.001), par_.Ra_max);  // radius of the sphere Sa

  if (1)
  {
    JPSk_solved_ = true;
    if (par_.visual == true)
    {
      clearJPSPathVisualization(JPSk_NORMAL);
      publishJPSPath(JPSk, JPSk_NORMAL);
    }
  }
  else
  {
    printf("JPSk didn't find a solution\n");
    JPSk_solved_ = false;
    return;
  }

  bool noPointsOutsideSphere1;
  std::cout << "here, ra=" << ra << std::endl;
  // std::cout << "here, state_pos=" << state_pos.transpose() << std::endl;

  std::cout << "JPSk is" << std::endl;
  printElementsOfJPS(JPSk);

  Eigen::Vector3d E;
  E = getFirstIntersectionWithSphere(JPSk, ra, JPSk[0], &li1, &noPointsOutsideSphere1);

  B_ = E;

  vec_Vecf<3> JPSk_inside_sphere(JPSk.begin(), JPSk.begin() + li1 + 1);  // Elements of JPS that are inside the sphere

  if (noPointsOutsideSphere1 == false)
  {
    JPSk_inside_sphere.push_back(E);
  }
  std::cout << "JPS used for whole before creating more vertexes=" << std::endl;
  printElementsOfJPS(JPSk_inside_sphere);

  createMoreVertexes(JPSk_inside_sphere, par_.dist_max_vertexes);

  std::cout << "JPS used for whole after creating extra vertexes=" << std::endl;
  printElementsOfJPS(JPSk_inside_sphere);

  //////////////////////////////////////////////////////////////////////////
  //////////// Solve with GUROBI Whole trajectory //////////////////////////
  /////////////////////////////////////////////////////////////////////////
  std::cout << bold << green << "***********WHOLE TRAJ*********************" << reset << std::endl;

  double before = ros::Time::now().toSec();

  if (JPSk_inside_sphere.size() > par_.max_poly + 1)  // If I have more than (par_.max_poly + 1) vertexes
  {
    JPSk_inside_sphere.erase(JPSk_inside_sphere.begin() + par_.max_poly + 1,
                             JPSk_inside_sphere.end());  // Force JPS to have less than par_.max_poly elements
    E = JPSk_inside_sphere[JPSk_inside_sphere.size() - 1];
  }

  std::cout << "JPS used for whole is" << std::endl;
  printElementsOfJPS(JPSk_inside_sphere);

  if (par_.visual == true)
  {
    clearJPSPathVisualization(JPS_WHOLE);
    publishJPSPath(JPSk_inside_sphere, JPS_WHOLE);
  }

  MyTimer cvx_ellip_decomp_t(true);
  cvxEllipsoidDecompOcc(JPSk_inside_sphere);  // result saved in l_constraints_
  log_.cvx_decomp_whole_ms = cvx_ellip_decomp_t.ElapsedMs();
  std::cout << bold << blue << "CVXDecompWhole:  " << std::fixed << cvx_ellip_decomp_t << "ms" << reset << std::endl;

  if (par_.visual)
  {
    E_.header.stamp = ros::Time::now();
    E_.pose.position.x = E(0);
    E_.pose.position.y = E(1);
    E_.pose.position.z = E(2);
    pub_point_E_.publish(E_);
  }

  double xf[9] = { E(0), E(1), E(2), 0, 0, 0, 0, 0, 0 };
  sg_whole_.setXf(xf);
  sg_whole_.setX0(x0);
  sg_whole_.setPolytopes(l_constraints_whole_);
  bool solved_whole = false;

  if (l_constraints_whole_[0].inside(InitPos) == false)
  {
    std::cout << red << "First point of whole traj is outside" << reset << std::endl;
  }

  MyTimer whole_gurobi_t(true);
  solved_whole = sg_whole_.genNewTraj();
  log_.gurobi_whole_ms = sg_whole_.runtime_ms_;
  log_.gurobi_whole_ms_mine = whole_gurobi_t.ElapsedMs();
  log_.gurobi_whole_trials = sg_whole_.trials_;
  log_.gurobi_whole_dt = sg_whole_.dt_;
  log_.gurobi_whole_factor = sg_whole_.factor_that_worked_;

  std::cout << bold << blue << "WholeGurobi:  " << std::fixed << whole_gurobi_t << "ms, (" << std::fixed
            << sg_whole_.runtime_ms_ << " ms), " << reset << sg_whole_.trials_ << " trials (dt=" << sg_whole_.dt_
            << "), f_worked=" << std::setprecision(2) << sg_whole_.factor_that_worked_ << std::endl;

  if (solved_whole == false)
  {
    std::cout << red << "No solution found for the whole trajectory" << reset << std::endl;
    return;
  }

  MyTimer fill_whole_t(true);
  sg_whole_.fillXandU();
  std::cout << bold << blue << "Fill Whole:  " << std::fixed << fill_whole_t << "ms" << reset << std::endl;
  mtx_X_U_temp.lock();

  ///////////////////////////////////////////////////////////
  ///////////////       SAFE PATH   //////////////////////
  //////////////////////////////////////////////////////////
  std::cout << bold << green << "***********SAFE PATH*********************" << reset << std::endl;

  MyTimer otherStuff3_t(true);
  mtx_offsets.lock();
  deltaTp_ = std::min((int)deltaTp_,
                      (int)(sg_whole_.X_temp_.rows() - 1));  // R is the point of the trajectory offset_rp ms after the
                                                             // start of the  whole trajectory
  int index = deltaTp_;
  mtx_offsets.unlock();

  std::cout << "index=" << index << std::endl;

  std::cout << "Rows of X_temp_" << sg_whole_.X_temp_.rows() << std::endl;

  /*  std::cout << "******************Actual state:" << std::endl;
    std::cout << "Pos=" << state_pos.transpose() << std::endl;
    std::cout << "Vel=" << state_vel.transpose() << std::endl;*/

  Eigen::Vector3d posR(sg_whole_.X_temp_(index, 0), sg_whole_.X_temp_(index, 1), sg_whole_.X_temp_(index, 2));
  Eigen::Vector3d velR(sg_whole_.X_temp_(index, 3), sg_whole_.X_temp_(index, 4), sg_whole_.X_temp_(index, 5));
  Eigen::Vector3d accelR(sg_whole_.X_temp_(index, 6), sg_whole_.X_temp_(index, 7), sg_whole_.X_temp_(index, 8));

  std::cout << "posR=" << posR.transpose() << std::endl;
  mtx_X_U_temp.unlock();

  MyTimer check_collision_AR_t(true);
  if (ARisInFreeSpace(index) == false and takeoff_done_ == true)
  {
    std::cout << red << bold << "The piece A-->R is not in Free Space" << std::endl;
    return;
  }

  std::cout << bold << blue << "Check collision with AR:  " << std::fixed << check_collision_AR_t << "ms" << reset
            << std::endl;

  vec_Vecf<3> JPSk_inside_sphere_tmp = JPSk_inside_sphere;
  bool thereIsIntersection2;
  std::cout << "JPSk_inside_sphere_tmp ANTES:" << std::endl;
  printElementsOfJPS(JPSk_inside_sphere_tmp);
  Eigen::Vector3d M = getFirstCollisionJPS(JPSk_inside_sphere_tmp, &thereIsIntersection2, UNKNOWN_MAP,
                                           RETURN_INTERSECTION);  // results saved in JPSk_inside_sphere_tmp

  std::cout << "Point M is:" << M.transpose() << std::endl;
  if (par_.visual)
  {
    M_.header.stamp = ros::Time::now();
    M_.pose.position.x = M(0);
    M_.pose.position.y = M(1);
    M_.pose.position.z = M(2);
    pub_point_M_.publish(M_);
  }

  std::cout << "JPSk_inside_sphere_tmp DESPUES:" << std::endl;

  printElementsOfJPS(JPSk_inside_sphere_tmp);

  JPSk_inside_sphere_tmp[0] = posR;

  std::cout << bold << blue << "OtherStuff 3:  " << std::fixed << otherStuff3_t << "ms" << reset << std::endl;

  MyTimer cvx_ellip_decomp2_t(true);
  cvxEllipsoidDecompUnkOcc2(JPSk_inside_sphere_tmp);  // result saved in l_constraints_
  publishJPSPath(JPSk_inside_sphere_tmp, JPS_SAFE);

  log_.cvx_decomp_safe_ms = cvx_ellip_decomp2_t.ElapsedMs();
  std::cout << bold << blue << "CVXDecompSafe:  " << std::fixed << cvx_ellip_decomp2_t << "ms" << reset << std::endl;

  mtx_X_U_temp.unlock();

  if (par_.visual)
  {
    R_.header.stamp = ros::Time::now();
    R_.pose.position.x = posR(0);
    R_.pose.position.y = posR(1);
    R_.pose.position.z = posR(2);
    pub_point_R_.publish(R_);
  }

  bool isGinside = l_constraints_safe_[l_constraints_safe_.size() - 1].inside(G);
  if (isGinside)
  {
    std::cout << red << bold << "G is inside" << reset << std::endl;
  }
  M = (isGinside == true) ? G : M;

  double x0_safe[9] = { posR[0], posR[1], posR[2], velR[0], velR[1], velR[2], accelR[0], accelR[1], accelR[2] };
  double xf_safe[9] = { M[0], M[1], M[2], 0, 0, 0, 0, 0, 0 };  // Note that the final position of xf_safe is only
                                                               // used to find dt, not as a final condition

  /*  std::cout << "Punto inicial: " << x0_safe[0] << ", " << x0_safe[1] << ", " << x0_safe[2] << std::endl;
    std::cout << "Punto final: " << xf_safe[0] << ", " << xf_safe[1] << ", " << xf_safe[2] << std::endl;*/

  sg_safe_.setXf(xf_safe);
  sg_safe_.setX0(x0_safe);
  std::cout << "Setting polytopes for safe" << std::endl;
  sg_safe_.setPolytopes(l_constraints_safe_);
  std::cout << "Polytopes set=" << l_constraints_safe_.size() << std::endl;

  bool isMinside = l_constraints_safe_[l_constraints_safe_.size() - 1].inside(M);

  if (isMinside && takeoff_done_ == true)
  {
    // std::cout << bold << "Forcing final constraint M=" << M.transpose() << reset << std::endl;
    sg_safe_.setForceFinalConstraint(1);  // !thereIsIntersection2 If no intersection --> goal is inside
                                          // polytope --> force final constraint
  }
  else
  {
    sg_safe_.setForceFinalConstraint(0);
  }

  if (l_constraints_safe_[0].inside(posR) == false)
  {
    std::cout << red << "First point of safe traj is outside" << reset << std::endl;
  }

  // std::cout << "l_constraints_uo_.size()=" << l_constraints_uo_.size() << std::endl;
  MyTimer safe_gurobi_t(true);
  std::cout << "Generating new trajectory" << std::endl;
  bool solved_safe = sg_safe_.genNewTraj();
  std::cout << "Generated" << std::endl;

  if (solved_safe == false)
  {
    std::cout << red << "No solution found for the safe path" << reset << std::endl;
    return;
  }
  log_.gurobi_safe_ms = sg_safe_.runtime_ms_;
  log_.gurobi_safe_ms_mine = safe_gurobi_t.ElapsedMs();
  log_.gurobi_safe_trials = sg_safe_.trials_;
  log_.gurobi_safe_dt = sg_safe_.dt_;
  log_.gurobi_safe_factor = sg_safe_.factor_that_worked_;

  std::cout << bold << blue << "SafeGurobi:  " << std::fixed << safe_gurobi_t << "ms, (" << std::fixed
            << sg_safe_.runtime_ms_ << " ms), " << reset << sg_safe_.trials_ << " trials (dt=" << sg_safe_.dt_
            << "), f_worked=" << std::setprecision(2) << sg_safe_.factor_that_worked_ << std::endl;
  ///////////////////////////////////////////////////////////
  ///////////////       MERGE RESULTS    ////////////////////
  ///////////////////////////////////////////////////////////

  MyTimer fill_safe_t(true);
  // Both have solution
  std::cout << "Going to fill" << std::endl;
  sg_safe_.fillXandU();
  std::cout << bold << blue << "Fill Safe:  " << std::fixed << fill_safe_t << "ms" << reset << std::endl;

  MyTimer otherStuff2_t(true);

  // X_temp_ will be 1st part of whole + safe path

  mtx_X_U_temp.lock();

  //# of cols are the same for all the trajectories
  int colsU = sg_whole_.U_temp_.cols();
  int colsX = sg_whole_.X_temp_.cols();

  int rows_X_safe = sg_safe_.X_temp_.rows();
  int rows_U_safe = sg_safe_.U_temp_.rows();
  int rows_X_whole = sg_whole_.X_temp_.rows();
  int rows_U_whole = sg_whole_.U_temp_.rows();
  // printf("Going to resize\n");
  U_temp_.conservativeResize(index + rows_X_safe + 1, colsU);  // Set the number of rows of U_temp_
  X_temp_.conservativeResize(index + rows_U_safe + 1, colsX);  // Set the number of rows of X_temp_

  /*  std::cout << "U_temp_ has (rows,cols)=" << U_temp_.rows() << ", " << U_temp_.cols() << std::endl;
    std::cout << "X_temp_ has (rows,cols)=" << X_temp_.rows() << ", " << X_temp_.cols() << std::endl;

    std::cout << "index is" << index << std::endl;
    std::cout << "rows_U_safe is" << rows_U_safe << std::endl;*/

  // copy the 1st part of the whole traj and the part of the safe path
  // std::cout << "Last row of X_temp_copied is" << X_temp_.row(index + 1).transpose() << std::endl;
  U_temp_.block(0, 0, index + 1, colsU) = sg_whole_.U_temp_.block(0, 0, index + 1, colsU);
  X_temp_.block(0, 0, index + 1, colsX) = sg_whole_.X_temp_.block(0, 0, index + 1, colsX);

  U_temp_.block(index + 1, 0, rows_U_safe, colsU) = sg_safe_.U_temp_;
  X_temp_.block(index + 1, 0, rows_X_safe, colsX) = sg_safe_.X_temp_;

  // std::cout << "******************SOL CONJUNTA*********************\n";
  // std::cout << X_temp_ << std::endl;
  // std::cout << U_temp_ << std::endl;

  // std::cout << "******************Last position of SOL CONJUNTA*********************\n";

  // printf("Copied block\n");
  mtx_X_U_temp.unlock();

  if (par_.visual == true)
  {
    // clearJPSPathVisualization(JPS_WHOLE);
    // publishJPSPath(JPSk_inside_sphere, JPS_WHOLE);
    pubTraj(sg_safe_.X_temp_, SAFE);
    pubTraj(sg_whole_.X_temp_, WHOLE);
  }

  ///////////////////////////////////////////////////////////
  ///////////////       OTHER STUFF    //////////////////////
  ///////////////////////////////////////////////////////////
  JPS_old_ = JPSk;
  mtx_k.lock();
  k_initial_cond_ = k_initial_cond_1_;
  // printf("Ahora mismo, k_initial_cond_=%d and k_=%d\n", k_initial_cond_, k_);
  mtx_k.unlock();

  optimized_ = true;
  mtx_planner_status_.lock();
  planner_status_ = REPLANNED;
  mtx_planner_status_.unlock();
  // printf("ReplanCB: planner_status_ = REPLANNED\n");
  // std::cout << "Rescue Path:\n" << sg_safe_.X_temp_ << std::endl;
  Eigen::Vector3d F;
  F << sg_safe_.X_temp_(rows_X_safe - 1, 0), sg_safe_.X_temp_(rows_X_safe - 1, 1),
      sg_safe_.X_temp_(rows_X_safe - 1, 2);  // Final point of the safe path
  //  = sg_safe_.X_temp_.block(sg_safe_.X_temp_.rows() - 1, 0, 1, 3);  // Final point of the safe path
  // std::cout << "*Final Point Safe Path=" << F.transpose() << std::endl;
  double dist = (G_term_ - F).norm();
  // std::cout << "******Distance=" << dist << std::endl;
  if (dist < par_.goal_radius)
  {
    std::cout << "Changed to GoalSeen" << std::endl;
    status_ = GOAL_SEEN;
  }

  std::cout << bold << blue << "OtherStuff 2:  " << std::fixed << otherStuff2_t << "ms" << reset << std::endl;

  log_.total_replanning_ms = replanCB_t.ElapsedMs();
  std::cout << bold << blue << "TOTAL REPLANNING CB:  " << std::fixed << replanCB_t << "ms" << reset << std::endl;

  mtx_offsets.lock();
  int states_last_replan = ceil(replanCB_t.ElapsedMs() / (par_.dc * 1000));  // Number of states that
                                                                             // would have been needed for
                                                                             // the last replan
  deltaTp_ = std::max(par_.factor_deltaTp * states_last_replan,
                      (double)par_.min_states_deltaTp);  // deltaTp
  std::cout << "Next deltaTp_:  " << std::fixed << deltaTp_ << " states" << std::endl;
  deltaT_ = std::max(par_.factor_deltaT * states_last_replan,
                     (double)par_.min_states_deltaT);  // Delta_t

  mtx_offsets.unlock();

  // Time allocation
  double new_init_whole =
      std::max(sg_whole_.factor_that_worked_ - par_.gamma_whole, 1.0);         // sg_whole_.factor_that_worked_ - 0.25;
  double new_final_whole = sg_whole_.factor_that_worked_ + par_.gammap_whole;  // high end factor is not a problem
  sg_whole_.setFactorInitialAndFinalAndIncrement(new_init_whole, new_final_whole, par_.increment_whole);

  double new_init_safe = std::max(sg_safe_.factor_that_worked_ - par_.gamma_safe, 1.0);
  double new_final_safe = sg_safe_.factor_that_worked_ + par_.gammap_safe;  // high end factor is not a problem
  sg_safe_.setFactorInitialAndFinalAndIncrement(new_init_safe, new_final_safe, par_.increment_safe);

  std::cout << "Next factors: W: " << std::fixed << std::setprecision(2) << new_init_whole << "-->" << new_final_whole
            << " R: " << new_init_safe << "-->" << new_final_safe << reset << std::endl;

  log_.header.stamp = ros::Time::now();

  if (status_ != GOAL_REACHED)
  {
    pub_log_.publish(log_);
  }
  return;
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

std::vector<Eigen::Vector3d> CVX::simulateForward(Eigen::Vector3d& pos_init, Eigen::Vector3d& vel_init,
                                                  Eigen::Vector3d& accel_init, Eigen::MatrixXd& jerk_sent)
{
  double t = 0.01;

  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d accel;
  Eigen::Vector3d jerk;

  Eigen::Vector3d pos0 = pos_init;
  Eigen::Vector3d vel0 = vel_init;
  Eigen::Vector3d accel0 = accel_init;

  std::cout << "SIMULATED ONLINE!:" << std::endl;
  std::cout << pos0.transpose() << "  " << vel0.transpose() << "  " << accel0.transpose() << std::endl;
  std::cout << "going to simulate forward,jerk_sent.rows()=" << jerk_sent.rows() << std::endl;

  for (int j = 0; j < jerk_sent.rows(); j++)
  {
    // std::cout << "Simulating forward" << std::endl;
    jerk = jerk_sent.row(j).transpose();
    pos = (1 / 6.0) * jerk * t * t * t + accel0 * t * t / 2.0 + vel0 * t + pos0;
    /*    std::cout << "accel0 * t" << (accel0 * t).transpose() << std::endl;
        std::cout << "vel0" << (vel0).transpose() << std::endl;

        std::cout << "accel0 * t" << (accel0 * t).transpose() << std::endl;*/

    vel = (1 / 2.0) * jerk * t * t + accel0 * t + vel0;
    accel = jerk * t + accel0;

    pos0 = pos;
    accel0 = accel;
    vel0 = vel;

    std::cout << pos.transpose() << "  " << vel.transpose() << "  " << accel.transpose() << std::endl;
  }

  std::vector<Eigen::Vector3d> result;
  result.push_back(pos);
  result.push_back(vel);
  result.push_back(accel);
  return result;
}

// Compute the intersection of JPS with the first polytope of the vector "constraints" (each element of "constraints"
// represents a polytope)
Eigen::Vector3d CVX::getIntersectionJPSwithPolytope(vec_Vecf<3>& path, std::vector<LinearConstraint3D>& constraints,
                                                    bool& thereIsIntersection)
{
  // std::cout << "**IntersectionF:Path given=" << std::endl;
  // printElementsOfJPS(path);
  LinearConstraint3D constraint = constraints[0];
  // Each element of cs_vector is a pair (A,b) representing a polytope
  bool there_is_intersection = false;
  int last_id_inside = 0;
  // std::cout << "Inside Finding intersection" << std::endl;
  for (size_t i = 0; i < path.size(); i++)
  {
    if (constraint.inside(path[i]) == false)  // If a vertex of the path is not in JPS
    {
      there_is_intersection = true;
      break;
    }
    else
    {
      last_id_inside = i;
    }
  }

  // std::cout << "**IntersectionF: there_is_intersection= " << there_is_intersection << std::endl;

  thereIsIntersection = there_is_intersection;
  if (there_is_intersection == false)
  {  // If no intersection, return last point in the path
    return path[path.size() - 1];
  }

  // std::cout << "Out Looop 2" << std::endl;

  int n_of_faces = constraint.b().rows();
  MatDNf<3> A = constraint.A();
  VecDf b = constraint.b();

  /*  for (int m = 0; m < b.size(); m++)
    {
      std::cout << "b[m] is" << b[m] << std::endl;
    }

    for (int m = 0; m < A.rows(); m++)
    {
      std::cout << "A[m] is " << A.row(m) << std::endl;
    }*/
  // std::cout << "A directamente es\n" << A << std::endl;

  Eigen::Vector3d inters;
  /*  std::cout << "last_id_inside=" << last_id_inside << std::endl;
    std::cout << "Num of el in path " << path.size() << std::endl;
    std::cout << "Number of faces " << n_of_faces << std::endl;*/

  int j = 0;
  vec_Vecf<3> intersections;
  for (size_t j = 0; j < n_of_faces; j++)
  {
    bool intersection_with_this_face = false;
    Eigen::Vector4d coeff;
    Eigen::Vector3d normal = A.row(j);  // normal vector
    coeff << normal(0), normal(1), normal(2), -b(j);
    // std::cout << "j=" << j << std::endl;

    intersection_with_this_face =
        getIntersectionWithPlane(path[last_id_inside], path[last_id_inside + 1], coeff, inters);
    // std::cout << "j despues=" << j << std::endl;
    if (intersection_with_this_face == true)
    {
      intersections.push_back(inters);

      // break;
    }
  }

  if (intersections.size() == 0)
  {  // There is no intersection
    ROS_ERROR("THIS IS IMPOSSIBLE, THERE SHOULD BE AN INTERSECTION");
  }

  std::vector<double> distances;

  // And now take the nearest intersection
  for (size_t i = 0; i < intersections.size(); i++)
  {
    double distance = (intersections[i] - path[0]).norm();
    distances.push_back(distance);
  }
  int minElementIndex = std::min_element(distances.begin(), distances.end()) - distances.begin();
  inters = intersections[minElementIndex];
  // std::cout << "Plane coeff" << coeff.transpose() << std::endl;
  // std::cout << "Intersection=" << inters.transpose() << " is the correct one!" << std::endl;
  // std::cout << "Reached this point" << std::endl;

  if (par_.visual == true)
  {
    I_.header.stamp = ros::Time::now();
    I_.pose.position.x = inters(0);
    I_.pose.position.y = inters(1);
    I_.pose.position.z = inters(2);
    pub_intersectionI_.publish(I_);
  }

  // std::cout << "Going to return" << inters.transpose() << std::endl;
  return inters;
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
}

void CVX::pubCB(const ros::TimerEvent& e)
{
  // print_status();
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

  // Save previous dyaw:
  dyaw_filtered_ = quadGoal_.dyaw;

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
       // ROS_WARN("Optimization took too long. Increase deltaT");
    }

    if ((planner_status_ == REPLANNED && (k_ == k_initial_cond_ || to_land_ == true)) ||  // Should be k_==
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
      std::cout << bold << magenta << "Rejecting current plan, planning again. Suggestion: Increase delta_t" << reset
                << std::endl;
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

    quadGoal_.pos = getPos(k_);
    quadGoal_.vel = getVel(k_);
    quadGoal_.accel = (par_.use_ff) ? getAccel(k_) : vectorNull();
    quadGoal_.jerk = (par_.use_ff) ? getJerk(k_) : vectorNull();
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

      //      std::cout << red << bold << "diff after wrappping=" << diff << reset << std::endl;

      yaw(diff, quadGoal_);
      /*      printf("Inside, desired_yaw=%0.2f,quadGoal_.yaw=%0.2f, diff=%f , abs(diff)=%f\n", desired_yaw,
         quadGoal_.yaw, diff, fabs(diff));*/
      if (fabs(diff) < 0.2)
      {
        // printf("It's less than 0.2!!\n");
        status_ = TRAVELING;
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

      // std::cout << red << bold << std::setprecision(6) << "desired_yaw=" << desired_yaw << reset << std::endl;
      // std::cout << red << bold << std::setprecision(6) << "quadGoal_.yaw=" << quadGoal_.yaw << reset << std::endl;

      double diff = desired_yaw - quadGoal_.yaw;
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

    mtx_offsets.lock();
    if (k_ > deltaTp_ && status_ == TRAVELING)
    {
      // ROS_WARN("Switched to the SAFE PATH!!");
    }
    mtx_offsets.unlock();

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

  // std::cout << green << bold << std::setprecision(6) << "quadGoal_.yaw sent=" << quadGoal_.yaw << reset << std::endl;
  // std::cout << green << bold << std::setprecision(6) << "quadGoal_.dyaw sent=" << quadGoal_.dyaw << reset <<
  // std::endl;

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

void CVX::modeCB(const acl_msgs::QuadFlightMode& msg)
{
  printf("*****In modeCB\n");
  if (msg.mode == msg.LAND)  //&& flight_mode_.mode != flight_mode_.LAND
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
    mtx_goals.lock();
    mtx_state.lock();
    double x0[9] = { state_.pos.x, state_.pos.y, state_.pos.z, state_.vel.x, state_.vel.y, state_.vel.z, 0, 0, 0 };

    double xf[9] = { state_.pos.x, state_.pos.y, par_.z_land, 0, 0, 0, 0, 0, 0 };
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

    // printf("Hola6.7\n");
    /*        solver_jerk_.set_xf(xf);
            solver_jerk_.set_x0(x0);
            k_initial_cond_ = std::min(k_ + deltaT, (int)(X_.rows() - 1));
            solver_jerk_.set_x0(x0);
            solver_jerk_.set_xf(xf);
            solver_jerk_.genNewTraj();*/
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
    initialCond_.pos = msg.pos;
    mtx_initial_cond.unlock();
  }

  static int i = 0;
  i++;

  if (status_ != GOAL_REACHED && par_.visual == true)
  {
    pubActualTraj();
  }

  if (i % 6 == 0 && status_ != GOAL_REACHED && i != 0)
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
    // std::cout << "heere before the lock" << std::endl;
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

geometry_msgs::Vector3 CVX::getPos(int i)
{
  // int input_order = solver_jerk_.getOrder();
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
  // int input_order = solver_jerk_.getOrder();
  geometry_msgs::Vector3 tmp;
  mtx_X_U.lock();
  /*  switch (input_order)
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
    }*/
  tmp.x = X_(i, 3);
  tmp.y = X_(i, 4);
  tmp.z = X_(i, 5);
  mtx_X_U.unlock();
  return tmp;
}

geometry_msgs::Vector3 CVX::getAccel(int i)
{
  // int input_order = solver_jerk_.getOrder();
  geometry_msgs::Vector3 tmp;
  mtx_X_U.lock();
  /*  switch (input_order)
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
    }*/
  tmp.x = X_(i, 6);
  tmp.y = X_(i, 7);
  tmp.z = X_(i, 8);
  mtx_X_U.unlock();

  return tmp;
}

geometry_msgs::Vector3 CVX::getJerk(int i)
{
  // int input_order = solver_jerk_.getOrder();
  geometry_msgs::Vector3 tmp;

  mtx_X_U.lock();
  /*  switch (input_order)
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
    }*/
  tmp.x = U_(i, 0);
  tmp.y = U_(i, 1);
  tmp.z = U_(i, 2);
  mtx_X_U.unlock();
  return tmp;
}

void CVX::cvxEllipsoidDecompUnkOcc2(vec_Vecf<3>& path)
{
  // std::cout << "In cvxDecomp 0!" << std::endl;
  if (kdtree_map_initialized_ == false)
  {
    return;
  }

  if (takeoff_done_ == false)
  {
    std::cout << bold << green << "No takeoff_done_ done yet" << std::endl;
    vec_Vec3f empty_obs;
    ellip_decomp_util_uo2_.set_obs(empty_obs);  // No unkown space when taking off
  }
  else
  {
    ellip_decomp_util_uo2_.set_obs(vec_uo_);
  }
  ellip_decomp_util_uo2_.set_local_bbox(
      Vec3f(2, 2, 1));  // Only try to find cvx decomp in the Mikowsski sum of JPS and this box (I think)
                        // par_.drone_radius
  ellip_decomp_util_uo2_.set_inflate_distance(par_.drone_radius);  // The obstacles are inflated by this distance
  ellip_decomp_util_uo2_.dilate(path);                             // Find convex polyhedra
  // decomp_util.shrink_polyhedrons(par_.drone_radius);  // Shrink polyhedra by the drone radius. NOT RECOMMENDED (leads
  // to lack of continuity in path sometimes)

  if (par_.visual == true)
  {
    decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(ellip_decomp_util_uo2_.get_ellipsoids());
    es_msg.header.frame_id = "world";

    decomp_ros_msgs::PolyhedronArray poly_msg =
        DecompROS::polyhedron_array_to_ros(ellip_decomp_util_uo2_.get_polyhedrons());
    poly_msg.header.frame_id = "world";

    // Publish visualization msgs
    // cvx_decomp_el_uo2_pub__.publish(es_msg);
    cvx_safe_pub_.publish(poly_msg);
  }
  // Convert to inequality constraints Ax < b
  // std::vector<polytope> polytopes;
  auto polys = ellip_decomp_util_uo2_.get_polyhedrons();

  l_constraints_safe_.clear();

  for (size_t i = 0; i < path.size() - 1; i++)
  {
    // std::cout << "Inserting constraint" << std::endl;
    const auto pt_inside = (path[i] + path[i + 1]) / 2;
    LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());
    l_constraints_safe_.push_back(cs);
  }

  if (l_constraints_safe_[0].inside(path[0]) == false)
  {
    std::cout << red << "In cvxEllip: First point of safe traj is outside" << reset << std::endl;
  }
}

void CVX::cvxEllipsoidDecompOcc(vec_Vecf<3>& path)
{
  // std::cout << "In cvxDecomp 0!" << std::endl;
  if (kdtree_map_initialized_ == false)
  {
    return;
  }

  // vec_Vec3f obs;
  // std::cout << "Type Obstacles==OCCUPIED_SPACE**************" << std::endl;
  // pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr_cloud_map = kdtree_map_.getInputCloud();
  // obs = pclptr_to_vec(ptr_cloud_map);

  // Using ellipsoid decomposition
  ellip_decomp_util_.set_obs(vec_o_);
  ellip_decomp_util_.set_local_bbox(
      Vec3f(2, 2, 1));  // Only try to find cvx decomp in the Mikowsski sum of JPS and this box (I think)
                        // par_.drone_radius
  ellip_decomp_util_.set_inflate_distance(par_.drone_radius);  // The obstacles are inflated by this distance
  ellip_decomp_util_.dilate(path);                             // Find convex polyhedra
  // decomp_util.shrink_polyhedrons(par_.drone_radius);  // Shrink polyhedra by the drone radius. NOT RECOMMENDED (leads
  // to lack of continuity in path sometimes)

  if (par_.visual == true)
  {
    decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(ellip_decomp_util_.get_ellipsoids());
    es_msg.header.frame_id = "world";

    decomp_ros_msgs::PolyhedronArray poly_msg =
        DecompROS::polyhedron_array_to_ros(ellip_decomp_util_.get_polyhedrons());
    poly_msg.header.frame_id = "world";

    // Publish visualization msgs
    // cvx_decomp_el_o_pub__.publish(es_msg);
    cvx_whole_pub_.publish(poly_msg);
  }
  // Convert to inequality constraints Ax < b
  // std::vector<polytope> polytopes;
  auto polys = ellip_decomp_util_.get_polyhedrons();

  l_constraints_whole_.clear();

  for (size_t i = 0; i < path.size() - 1; i++)
  {
    // std::cout << "Inserting constraint" << std::endl;
    const auto pt_inside = (path[i] + path[i + 1]) / 2;
    LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());
    l_constraints_whole_.push_back(cs);
  }
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

// Occupied CB
void CVX::mapCB(const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_map_ros,
                const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_unk_ros)
{
  // std::cout<<"In mapCB"<<std::endl;

  MyTimer mapCB_t(true);
  // std::cout << "Before mtx_map.lock()" << std::endl;
  mtx_map.lock();
  // std::cout << "Before mtx_unk.loc()" << std::endl;
  mtx_unk.lock();

  // double before = ros::Time::now().toSec();

  pcl::fromROSMsg(*pcl2ptr_map_ros, *pclptr_map_);

  // ROS_WARN("ToPcl takes: %0.2f ms", 1000 * (ros::Time::now().toSec() - before_copy_to_pcl));

  updateJPSMap(pclptr_map_);  // UPDATE EVEN WHEN THERE ARE NO POINTS

  if (pcl2ptr_map_ros->width != 0 && pcl2ptr_map_ros->height != 0)  // Point Cloud is empty
  {
    // std::cout << "Updating kdtree_map_, size=" << pclptr_map_->size() << std::endl;
    kdtree_map_.setInputCloud(pclptr_map_);
    kdtree_map_initialized_ = 1;

    // Option 1 to convert pcl to a vector

    /*  MyTimer timer1(true);
      Eigen::MatrixXd mat = (pclptr_map_->getMatrixXfMap()).cast<double>();
      int cols = pclptr_map_->points.size();
      std::vector<Eigen::Vector3d> v(pclptr_map_->points.size());
      Eigen::Matrix<double, 3, Eigen::Dynamic>::Map(v.data()->data(), 3, cols) = mat;
      std::cout << bold << blue << "With map:  " << timer1 << "ms" << reset << std::endl;*/

    // Option 2 (faster):

    // MyTimer timer2(true);
    vec_o_ = pclptr_to_vec(pclptr_map_);
    // std::cout << bold << blue << "With pclptr_to_vec:  " << timer2 << "ms" << reset << std::endl;
  }
  else
  {
    std::cout << "Occupancy Grid received is empty, maybe map is too small?\n";
  }

  ///////////Unkown CB/////////////////
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

    kdtree_unk_.setInputCloud(pclptr_unk_);  // Commented this to improve speed
    kdtree_unk_initialized_ = 1;
    vec_uo_ = pclptr_to_vec(pclptr_unk_);                         // insert unknown space
    vec_uo_.insert(vec_uo_.end(), vec_o_.begin(), vec_o_.end());  // append known space
  }
  mtx_map.unlock();
  mtx_unk.unlock();
  std::cout << bold << blue << "MapCB:  " << mapCB_t << "ms" << reset << std::endl;

  // ROS_WARN("MapCB takes: %0.2f ms", 1000 * (ros::Time::now().toSec() - before));
}

// Returns the first collision of JPS with the map (i.e. with the known obstacles). Note that JPS will collide with a
// map B if JPS was computed using an older map A
// If type_return==Intersection, it returns the last point in the JPS path that is at least par_.inflation_jps from map
Eigen::Vector3d CVX::getFirstCollisionJPS(vec_Vecf<3>& path, bool* thereIsIntersection, int map, int type_return)
{
  vec_Vecf<3> original = path;

  // vec_Vecf<3> path_behind;
  /*    std::cout << "In getFirstCollisionJPS\n" << std::endl;
      std::cout << "*****ORIGINAL******" << std::endl;
      printElementsOfJPS(original);*/
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
              std::cout << "In Return Intersection, vertexes_eliminated_tmp=" << vertexes_eliminated_tmp << std::endl;
              original.erase(original.begin() + vertexes_eliminated_tmp,
                             original.end());  // Now original contains all the elements eliminated
              original.push_back(path[0]);

              std::cout << "Result before reduceJPSbyDistance" << original[original.size() - 1].transpose()
                        << std::endl;

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
      printf("JPS provided doesn't intersect any obstacles, returning the first element of the path you gave me\n");
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

void CVX::pubActualTraj()
{
  // ROS_ERROR("In pubActualTraj\n");

  static geometry_msgs::Point p_last = pointOrigin();
  mtx_state.lock();
  Eigen::Vector3d act_pos(state_.pos.x, state_.pos.y, state_.pos.z);
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

// P1-P2 is the direction used for projection. P2 is the gal clicked
Eigen::Vector3d CVX::projectClickedGoal(Eigen::Vector3d& P1)
{
  //[px1, py1, pz1] is inside the map (it's the center of the map, where the drone is)
  //[px2, py2, pz2] is outside the map
  mtx_G_term.lock();
  Eigen::Vector3d P2 = G_term_;
  mtx_G_term.unlock();
  // return P2;  // TODO: Comment this line after the HW experiments!
  double x_max = P1(0) + par_.wdx / 2;
  double x_min = P1(0) - par_.wdx / 2;
  double y_max = P1(1) + par_.wdy / 2;
  double y_min = P1(1) - par_.wdy / 2;
  double z_max = P1(2) + par_.wdz / 2;
  double z_min = P1(2) - par_.wdz / 2;

  if ((P2(0) < x_max && P2(0) > x_min) && (P2(1) < y_max && P2(1) > y_min) && (P2(2) < z_max && P2(2) > z_min))
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

  vec_Vecf<3> intersections;
  /*  std::cout << "The planes" << std::endl;
    for (int i = 0; i < 6; i++)
    {
      std::cout << all_planes[i].transpose() << std::endl;
    }*/
  // std::cout << "Projecting: intersectionPoint" << inters.transpose() << std::endl;
  int axis;  // 1 is x, 2 is y, 3 is z
  for (int i = 0; i < 6; i++)
  {
    if (getIntersectionWithPlane(P1, P2, all_planes[i], inters) == true)
    {
      intersections.push_back(inters);
    }
  }

  if (intersections.size() == 0)
  {  // There is no intersection
    ROS_ERROR("THIS IS IMPOSSIBLE, THERE SHOULD BE AN INTERSECTION");
  }
  std::vector<double> distances;
  // And now take the nearest intersection
  for (size_t i = 0; i < intersections.size(); i++)
  {
    double distance = (intersections[i] - P1).norm();
    distances.push_back(distance);
  }
  int minElementIndex = std::min_element(distances.begin(), distances.end()) - distances.begin();
  inters = intersections[minElementIndex];
  // axis = (int)(i / 2);
  // int N = 1;
  // pcl::PointXYZ searchPoint(inters[0], inters[1], inters[2]);
  // std::vector<int> id(N);
  // std::vector<float> pointNKNSquaredDistance(N);

  // Let's find now the nearest free or unkown point to the intersection
  /*      if (kdtree_frontier_initialized_)
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
        }*/

  // Now let's put the intersection 1.8 * par_.inflation_jps meters away from the end of the map.
  Eigen::Vector3d sign(inters[0] / fabs(inters[0]), inters[1] / fabs(inters[1]), inters[2] / fabs(inters[2]));

  // std::cout << "sign=" << sign.transpose() << std::endl;

  /*      inters(0) = (axis == 0) ? inters(0) + sign(0) * 1.5 * par_.inflation_jps : inters(0);
        inters(1) = (axis == 1) ? inters(1) + sign(1) * 1.5 * par_.inflation_jps : inters(1);
        inters(2) = (axis == 2) ? inters(2) + sign(2) * 1.5 * par_.inflation_jps : inters(2);*/

  // inters = inters + sign * 1.5 * par_.inflation_jps;
  // std::cout << "*****Projecting: returned" << inters.transpose() << std::endl;
  return inters;
}

/*    mtx_X_U.lock();
    int my_length = std::min(deltaT, (int)(U_.rows() - (k_ + 1)));
    my_length = (my_length < 1) ? 1 : my_length;
    Eigen::MatrixXd jerk_simulate = U_.block(k_ + 1, 0, my_length, U_.cols());
    Eigen::Vector3d vicon_accel = X_.block(k_, 6, 1, 3).transpose();  // TODO: Take this from the IMU in the real HW

    std::vector<Eigen::Vector3d> simulated;

    mtx_X_U.unlock();

    std::cout << "****STARTING FROM:" << std::endl;
    std::cout << "pos:" << state_pos.transpose() << std::endl;
    std::cout << "vel:" << state_vel.transpose() << std::endl;
    std::cout << "accel:" << vicon_accel.transpose() << std::endl;

    std::cout << "Y aplicando el jerk:" << std::endl;
    std::cout << "jerk:" << jerk_simulate << std::endl;

    simulated = simulateForward(state_pos, state_vel, vicon_accel,
                                jerk_simulate);  // produces the simulated state deltaT + 1*/

/*
mtx_X_U.lock();

Eigen::Vector3d novale_pos = (X_.block(k_, 0, 1, 3)).transpose();
Eigen::Vector3d novale_vel = (X_.block(k_, 3, 1, 3)).transpose();
Eigen::Vector3d novale_accel = (X_.block(k_, 6, 1, 3)).transpose();
mtx_X_U.unlock();

 std::cout << "STARTING FROM:" << std::endl;
    std::cout << "pos:" << novale_pos.transpose() << std::endl;
    std::cout << "vel:" << novale_vel.transpose() << std::endl;
    std::cout << "accel:" << novale_accel.transpose() << std::endl;

    std::cout << "Y aplicando el jerk:" << std::endl;
    std::cout << "jerk:" << jerk_simulate << std::endl;

    simulated = simulateForward(novale_pos, novale_vel, novale_accel,
                                jerk_simulate);  // produces the simulated state deltaT + 1

    // simulated = simulateForward(state_pos, state_vel, vicon_accel, jerk_simulate);
    std::cout << "SIMULATED:" << std::endl;
    std::cout << simulated[0].transpose() << " " << simulated[1].transpose() << " " << simulated[2].transpose()
              << std::endl;

    mtx_X_U.lock();
    std::cout << "ME DEBERIA DAR:" << std::endl;
    std::cout << X_.block(k_, 0, deltaT + 2, X_.cols()) << std::endl;
    mtx_X_U.unlock();

    mtx_initial_cond.lock();
    std::cout << "IVE TAKEN AS INITIAL CONDITION:" << std::endl;
    std::cout << initialCond_.pos.x << ", " << initialCond_.pos.y << ", " << initialCond_.pos.z << ", "
              << initialCond_.vel.x << ", " << initialCond_.vel.y << ", " << initialCond_.vel.z << ", "
              << initialCond_.accel.x << ", " << initialCond_.accel.y << ", " << initialCond_.accel.z << std::endl;
    mtx_initial_cond.unlock();*/
//     double x0[9] = { (simulated[0])(0), (simulated[0])(1), (simulated[0])(2),  ////////
//                  (simulated[1])(0), (simulated[1])(1), (simulated[1])(2),  ////////
//                  (simulated[2])(0), (simulated[2])(1), (simulated[2])(2) };

// mtx_k.unlock();
/* std::cout << "SIMULATED:" << std::endl;
std::cout << simulated[0].transpose() << " " << simulated[1].transpose() << " " << simulated[2].transpose()
          << std::endl;
x0[0] = (simulated[0])(0);  // Pos
x0[1] = (simulated[0])(1);  // Pos
x0[2] = (simulated[0])(2);  // Pos
x0[3] = (simulated[1])(0);  // Vel
x0[4] = (simulated[1])(1);  // Vel
x0[5] = (simulated[1])(2);  // Vel
x0[6] = (simulated[2])(0);  // Accel
x0[7] = (simulated[2])(1);  // Accel
x0[8] = (simulated[2])(2);  // Accel

std::cout << "here4" << std::endl;*/

/*void CVX::pubPlanningVisual(Eigen::Vector3d center, double ra, double rb, Eigen::Vector3d B1, Eigen::Vector3d C1)
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
}*/

/*void CVX::frontierCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg)
{
  // printf("****In FrontierCB\n");
  if (pcl2ptr_msg->width == 0 || pcl2ptr_msg->height == 0)  // Point Cloud is empty (this happens at the beginning)
  {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_frontier(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_msg, *pclptr_frontier);

  mtx_frontier.lock();
  kdtree_frontier_.setInputCloud(pclptr_frontier);
  mtx_frontier.unlock();
  kdtree_frontier_initialized_ = 1;
}*/

/*void CVX::pclCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg)
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
}*/

/*void CVX::cvxSeedDecompUnkOcc(Vecf<3>& seed)
{
  double before = ros::Time::now().toSec();

  // std::cout << "In cvxDecomp 0!" << std::endl;
  if (kdtree_map_initialized_ == false)
  {
    return;
  }
  // std::cout << "In cvxDecomp 1!" << std::endl;

  // vec_Vec3f obs;

  // std::cout << "Type Obstacles==UNKOWN_AND_OCCUPIED_SPACE**************" << std::endl;

  // pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr_cloud_map = kdtree_map_.getInputCloud();
  // pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr_cloud_unkown = kdtree_unk_.getInputCloud();

  // std::cout << "Number of elements in unkCloud" < < < < std::endl;

 // obs = pclptr_to_vec(pclptr_map_, pclptr_unk_);
 //   std::cout << "Points in mapCloud=" << (*pclptr_map_).points.size() << std::endl;
 //   std::cout << "Points in unkCloud=" << (*pclptr_unk_).points.size() << std::endl;

  // Initialize SeedDecomp3D

  seed_decomp_util_.set_seed(seed);
  seed_decomp_util_.set_obs(vec_uo_);
  seed_decomp_util_.set_local_bbox(Vec3f(4, 4, 1));
  // std::cout << "In cvxDecomp before dilate!" << std::endl;
  seed_decomp_util_.dilate(0.1);
  // std::cout << "In cvxDecomp after dilate!" << std::endl;
  seed_decomp_util_.shrink_polyhedron(par_.drone_radius);
  // std::cout << "In cvxDecomp after shrink!" << std::endl;

  vec_E<Polyhedron<3>> polyhedron_as_array;  // This vector will contain only one element
  polyhedron_as_array.push_back(seed_decomp_util_.get_polyhedron());
  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polyhedron_as_array);
  poly_msg.header.frame_id = "world";

  // Publish visualization msgs
  cvx_decomp_poly_uo_pub_.publish(poly_msg);
  auto poly = seed_decomp_util_.get_polyhedron();
  // std::cout << "Incluso antes A es\n" << poly.hyperplanes()[0].n_.transpose() << std::endl;
  l_constraints_uo_.clear();
  LinearConstraint3D cs(seed, poly.hyperplanes());

  MatDNf<3> A = cs.A();
  // std::cout << "Incluso antes A es\n" << A << std::endl;

  l_constraints_uo_.push_back(cs);
  // ROS_WARN("SeedDecomp takes: %0.2f ms", 1000 * (ros::Time::now().toSec() - before));
}*/