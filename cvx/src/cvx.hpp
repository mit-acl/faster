#include "ros/ros.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <atomic>

#include <Eigen/Dense>

#include <acl_msgs/Cvx.h>
#include <acl_msgs/State.h>
#include <acl_msgs/QuadGoal.h>
#include <acl_msgs/QuadFlightMode.h>
#include <acl_msgs/TermGoal.h>
#include <nav_msgs/Odometry.h>

#include <mutex>

// TimeSynchronizer includes
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

// JPS3D includes
#include "timer.hpp"
#include "read_map.hpp"
#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>

#include "utils.hpp"

// Solvers includes
//#include "solvers/solvers.hpp" CVXGEN solver interface
#include "solvers/solverGurobi.hpp"

// Convex Decomposition includes
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// status_ : YAWING-->TRAVELING-->GOAL_SEEN-->GOAL_REACHED-->YAWING-->TRAVELING-->...
#define YAWING 0
#define TRAVELING 1
#define GOAL_SEEN 2
#define GOAL_REACHED 3

// planner_status_
#define FIRST_PLAN 0
#define START_REPLANNING 1
#define REPLANNED 2

#define MAP 1          // MAP refers to the occupancy grid
#define UNKNOWN_MAP 2  // UNKNOWN_MAP refers to the unkown grid

#define WHOLE 1  // Whole trajectory (part of which is planned on unkonwn space)
#define SAFE 2   // Safe path
#define COMMITTED_COLORED 3
#define WHOLE_COLORED 4
#define SAFE_COLORED 5

#define OCCUPIED_SPACE 1
#define UNKOWN_AND_OCCUPIED_SPACE 2

#define JPSk_NORMAL 1
#define JPS2_NORMAL 2
#define JPS_WHOLE 3
#define JPS_SAFE 4

#define RETURN_LAST_VERTEX 0
#define RETURN_INTERSECTION 1

struct kdTreeStamped
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
  ros::Time time;
};

struct polytope
{
  Eigen::MatrixXd A;
  Eigen::MatrixXd b;
};

struct parameters
{
  bool use_ff;
  bool visual;

  double wdx;
  double wdy;
  double wdz;
  double res;

  double dc;
  double goal_radius;
  double drone_radius;

  int N_whole;
  int N_safe;

  double factor_deltaT;
  double factor_min_deltaT;

  int min_states_deltaT;

  double Ra;
  double Ra_max;
  double w_max;
  double alpha_filter_dyaw;
  double alpha_0_deg;
  double z_ground;
  double z_max;
  double inflation_jps;
  double factor_jps;

  double v_max;
  double a_max;
  double j_max;

  double z_land;

  double gamma_whole;
  double gammap_whole;
  double increment_whole;

  double gamma_safe;
  double gammap_safe;
  double increment_safe;

  int max_poly_whole;
  int max_poly_safe;
  double dist_max_vertexes;

  int gurobi_threads;
  int gurobi_verbose;

  double kw;
  double kyaw;
  double kdalpha;
  double kv;
  double kdist;
  double kalpha;

  double delta_a;
  double delta_H;

  bool use_faster;
  bool keep_optimizing_after_found;
  bool use_smart_deltaT;

  double hack;
};

//####Class CVX
class CVX
{
public:
  CVX(ros::NodeHandle nh, ros::NodeHandle nh_replan_CB, ros::NodeHandle nh_pub_CB);

private:
  // CVXGEN solvers
  // Solver<VEL> solver_vel_;
  // Solver<ACCEL> solver_accel_;
  // Solver<JERK> solver_jerk_;

  SolverGurobi sg_whole_;  // solver gurobi whole trajectory
  SolverGurobi sg_safe_;   // solver gurobi whole trajectory

  // class methods
  // void pubTraj(double** x);
  void pubTraj(Eigen::MatrixXd& X, int type);
  void goalCB(const acl_msgs::TermGoal& msg);
  void stateCB(const acl_msgs::State& msg);
  void odomCB(const nav_msgs::Odometry& odom_ptr);
  void modeCB(const acl_msgs::QuadFlightMode& msg);
  void pubCB(const ros::TimerEvent& e);
  void replanCB(const ros::TimerEvent& e);

  void print_status();

  /*  void interpInput(double dt, double xf[], double u0[], double x0[], double** u, double** x, Eigen::MatrixXd& U,
                     Eigen::MatrixXd& X);*/

  void interpBRETT(double dt, double xf[], double u0[], double x0[], double** u, double** x, Eigen::MatrixXd& U,
                   Eigen::MatrixXd& X);

  visualization_msgs::Marker createMarkerLineStrip(Eigen::MatrixXd X);
  // void clearMarkerSetOfArrows();
  void clearMarkerActualTraj();
  void clearMarkerColoredTraj();
  void mapCB(const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_msg,
             const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_msg2);  // Callback for the occupancy pcloud
  void unkCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg);     // Callback for the unkown pcloud
  void pclCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg);
  void frontierCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg);

  // void novale(const sensor_msgs::PointCloud2::ConstPtr& data1, const sensor_msgs::PointCloud2::ConstPtr& data2);
  // bool trajIsFree(Eigen::MatrixXd X);
  Eigen::Vector3d computeForce(Eigen::Vector3d x, Eigen::Vector3d g);
  // std_msgs::ColorRGBA color(int id);
  Eigen::Vector3d createForceArrow(Eigen::Vector3d x, Eigen::Vector3d f_att, Eigen::Vector3d f_rep,
                                   visualization_msgs::MarkerArray* forces);

  // geometry_msgs::Point pointOrigin();
  // geometry_msgs::Point eigen2point(Eigen::Vector3d vector);
  void pubActualTraj();
  void vectorOfVectors2MarkerArray(vec_Vecf<3> traj, visualization_msgs::MarkerArray* m_array,
                                   std_msgs::ColorRGBA color, int type = visualization_msgs::Marker::ARROW,
                                   std::vector<double> radii = std::vector<double>());
  visualization_msgs::MarkerArray clearArrows();
  // geometry_msgs::Vector3 vectorNull();
  geometry_msgs::Vector3 getPos(int i);
  geometry_msgs::Vector3 getVel(int i);
  geometry_msgs::Vector3 getAccel(int i);
  geometry_msgs::Vector3 getJerk(int i);

  // double solveVelAndGetCost(vec_Vecf<3> path);
  void updateInitialCond(int i);
  // void pubPlanningVisual(Eigen::Vector3d center, double ra, double rb, Eigen::Vector3d B1, Eigen::Vector3d C1);
  // void pubintersecPoint(Eigen::Vector3d p, bool add);
  void yaw(double diff, acl_msgs::QuadGoal& quad_goal);

  void clearMarkerArray(visualization_msgs::MarkerArray* tmp, ros::Publisher* publisher);
  void publishJPSPath(vec_Vecf<3>& path, int i);
  void clearJPSPathVisualization(int i);

  void updateJPSMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr);
  vec_Vecf<3> solveJPS3D(Vec3f& start, Vec3f& goal, bool* solved, int i);

  void pubTerminalGoal();

  void pubJPSIntersection(Eigen::Vector3d& inters);
  Eigen::Vector3d getFirstCollisionJPS(vec_Vecf<3>& path, bool* thereIsIntersection, int map = MAP,
                                       int type_return = RETURN_LAST_VERTEX);
  Eigen::Vector3d projectClickedGoal(Eigen::Vector3d& P1);

  void publishJPS2handIntersection(vec_Vecf<3> JPS2_fix, Eigen::Vector3d& inter1, Eigen::Vector3d& inter2,
                                   bool solvedFix);

  vec_Vecf<3> fix(vec_Vecf<3>& JPS_old, Eigen::Vector3d& start, Eigen::Vector3d& goal, bool* solved);

  // double getDistanceToFirstCollisionJPSwithUnkonwnspace(vec_Vecf<3> path, bool* thereIsIntersection);

  void cvxEllipsoidDecompUnkOcc2(vec_Vecf<3>& path);
  void cvxEllipsoidDecompOcc(vec_Vecf<3>& path);
  void cvxSeedDecompUnkOcc(Vecf<3>& seed);

  // vec_Vecf<3> sampleJPS(vec_Vecf<3>& path, int n);

  // std::vector<double> getDistToNearestObs(vec_Vecf<3>& points);

  Eigen::Vector3d getIntersectionJPSwithPolytope(vec_Vecf<3>& path, std::vector<LinearConstraint3D>& constraints,
                                                 bool& thereIsIntersection);

  std::vector<Eigen::Vector3d> simulateForward(Eigen::Vector3d& pos_init, Eigen::Vector3d& vel_init,
                                               Eigen::Vector3d& accel_init, Eigen::MatrixXd& jerk_sent);

  void createMoreVertexes(vec_Vecf<3>& path, double d);

  bool ARisInFreeSpace(int index);

  int findIndexR(int indexH);
  int findIndexH(bool& needToComputeSafePath);

  // visualization_msgs::MarkerArray Matrix2ColoredMarkerArray(Eigen::MatrixXd& X, int type);

  visualization_msgs::Marker setpoint_;
  visualization_msgs::Marker R_;
  visualization_msgs::Marker I_;
  visualization_msgs::Marker E_;
  visualization_msgs::Marker M_;
  visualization_msgs::Marker H_;
  visualization_msgs::Marker A_;
  acl_msgs::QuadGoal quadGoal_;
  acl_msgs::QuadGoal stateA_;  // It's the initial condition for the solver
  acl_msgs::QuadFlightMode flight_mode_;
  acl_msgs::State state_;
  Eigen::Vector3d G_;       // This goal is always inside of the map
  Eigen::Vector3d G_term_;  // This goal is the clicked goal

  ros::NodeHandle nh_;
  ros::NodeHandle nh_replan_CB_;
  ros::NodeHandle nh_pub_CB_;

  ros::Publisher pub_goal_jackal_;
  ros::Publisher pub_point_G_;
  ros::Publisher pub_goal_;
  ros::Publisher pub_traj_whole_;
  ros::Publisher pub_traj_safe_;
  ros::Publisher pub_setpoint_;
  ros::Publisher pub_trajs_sphere_;
  ros::Publisher pub_forces_;
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

  // ros::Publisher cvx_decomp_el_o_pub__;
  ros::Publisher cvx_whole_pub_;

  // ros::Publisher cvx_decomp_el_uo2_pub__;
  ros::Publisher cvx_safe_pub_;

  // ros::Publisher cvx_decomp_poly_uo_pub_;
  ros::Subscriber sub_goal_;
  ros::Subscriber sub_state_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_mode_;
  ros::Subscriber sub_vicon_;

  Eigen::Vector3d accel_vicon_;

  // ros::Subscriber sub_map_;
  // ros::Subscriber sub_unk_;
  // ros::Subscriber sub_pcl_;
  ros::Subscriber sub_frontier_;
  ros::Timer pubCBTimer_;
  ros::Timer replanCBTimer_;

  parameters par_;     // where all the parameters are
  acl_msgs::Cvx log_;  // to log all the data
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

  vec_E<Polyhedron<3>> polyhedra_;
  std::vector<LinearConstraint3D> l_constraints_whole_;  // Polytope (Linear) constraints
  std::vector<LinearConstraint3D> l_constraints_safe_;   // Polytope (Linear) constraints

  // int deltaTp_old_ = 1000;
  // int deltaTp_ = 10;
  int deltaT_ = 10;
  int deltaT_min_ = 10;
  int indexR_ = 0;

  int markerID_ = 0;
  int markerID_last_ = 0;
  int actual_trajID_ = 0;
  Eigen::MatrixXd U_, X_;  // Contains the intepolated input/states that will be sent to the drone
  Eigen::MatrixXd U_temp_,
      X_temp_;  // Contains the intepolated input/states of a traj. If the traj. is free, it will be copied to U_, X_

  Eigen::MatrixXd U_safe_, X_safe_;
  bool optimized_;
  double spinup_time_;
  double z_start_;
  // double u_min_, u_max_, z_start_, spinup_time_, z_land_;
  // int N_ = 20;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_map_;       // kdtree of the point cloud of the occuppancy grid
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_unk_;       // kdtree of the point cloud of the unknown grid
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_frontier_;  // kdtree of the frontier

  bool kdtree_map_initialized_ = 0;
  bool kdtree_unk_initialized_ = 0;
  bool kdtree_frontier_initialized_ = 0;
  // vector that has all the kdtrees of the pclouds not included in the map:
  std::vector<kdTreeStamped> v_kdtree_new_pcls_;
  bool replanning_needed_ = true;
  bool goal_click_initialized_ = false;

  int cells_x_;  // Number of cells of the map in X
  int cells_y_;  // Number of cells of the map in Y
  int cells_z_;  // Number of cells of the map in Z

  int n_states_publised_ = 0;  // Number of goals=states published

  int status_ = TRAVELING;  // status_ can be TRAVELING, GOAL_SEEN, GOAL_REACHED
  int planner_status_ = FIRST_PLAN;

  bool force_reset_to_0_ = 1;

  int k_ = 0;               // Row of X_ that will be published next;
  int k_initial_cond_ = 0;  // Row of X_ chosen as the initial condition for the planner

  int k_initial_cond_1_ = 0;
  int k_initial_cond_2_ = 0;

  vec_Vecf<3> JPS_old_;

  double dyaw_filtered_ = 0;

  // pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_unk_;

  std::mutex mtx_map;  // mutex of occupied map (kdtree_map_)
  std::mutex mtx_unk;  // mutex of unkonwn map (pclptr_unk_)
  std::mutex mtx_frontier;
  std::mutex mtx_inst;  // mutex of instanteneous data (v_kdtree_new_pcls_)
  std::mutex mtx_goals;
  std::mutex mtx_jps_map_util;  // mutex for map_util_ and planner_ptr_
  std::mutex mtx_k;
  std::mutex mtx_X_U_temp;
  std::mutex mtx_X_U_safe;
  std::mutex mtx_X_U;
  std::mutex mtx_planner_status_;
  std::mutex mtx_initial_cond;
  std::mutex mtx_state;
  std::mutex mtx_offsets;
  // std::mutex mtx_factors;

  std::mutex mtx_G;
  std::mutex mtx_G_term;

  std::shared_ptr<JPS::VoxelMapUtil> map_util_;
  std::unique_ptr<JPSPlanner3D> planner_ptr_;

  bool X_initialized_ = false;

  Eigen::Vector3d pos_old_;
  Eigen::Vector3d B_;

  bool to_land_ = false;
  bool JPSk_solved_ = false;

  message_filters::Subscriber<sensor_msgs::PointCloud2> occup_grid_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> unknown_grid_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
      MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map_;

  vec_Vec3f vec_o_;   // Vector that contains the occupied points
  vec_Vec3f vec_uo_;  // Vector that contains the unkown and occupied points

  SeedDecomp3D seed_decomp_util_;
  EllipsoidDecomp3D ellip_decomp_util_;
  EllipsoidDecomp3D ellip_decomp_util_uo2_;

  vec_Vecf<3> JPS_k_m_1_;
  bool takeoff_done_ = false;

  bool state_initialized_ = false;

  double current_yaw_ = 0;

  double desired_yaw_old_ = 0;

  double alpha_before_ = 0;
  double desired_yaw_B_ = 0;
};
