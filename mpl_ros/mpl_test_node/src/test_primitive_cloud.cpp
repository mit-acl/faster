#include "bag_reader.hpp"
#include <ros/ros.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <motion_primitive_library/planner/mp_cloud_util.h>
#include <decomp_ros_utils/data_ros_utils.h>

#include "sensor_msgs/PointCloud2.h"

using namespace MPL;

std::unique_ptr<MPCloudUtil> planner_;

class sub_pcl_plan_class
{
public:
  sub_pcl_plan_class(ros::NodeHandle& nh)
  {  // Constructor
    es_pub = nh.advertise<decomp_ros_msgs::Ellipsoids>("ellipsoids", 1, true);
    sg_pub = nh.advertise<sensor_msgs::PointCloud>("start_and_goal", 1, true);
    traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("trajectory", 1, true);
    prior_traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("prior_trajectory", 1, true);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
    ps_pub = nh.advertise<sensor_msgs::PointCloud>("ps", 1, true);

    nh.param("file", file_name, std::string("voxel_map"));
    nh.param("topic", topic_name, std::string("voxel_map"));
    nh.param("origin_x", origin(0), 0.0);
    nh.param("origin_y", origin(1), 0.0);
    nh.param("origin_z", origin(2), 0.0);
    nh.param("range_x", dim(0), 0.0);
    nh.param("range_y", dim(1), 0.0);
    nh.param("range_z", dim(2), 0.0);
    nh.param("robot_r", robot_radius, 0.5);
    nh.param("dt", dt, 1.0);
    nh.param("epsilon", epsilon, 1.0);
    nh.param("v_max", v_max, -1.0);
    nh.param("a_max", a_max, -1.0);
    nh.param("u_max", u_max, 1.0);
    nh.param("u_max_z", u_max_z, 1.0);
    nh.param("t_max", t_max, -1.0);
    nh.param("w", w, 10.);
    nh.param("num", num, 1);
    nh.param("max_num", max_num, -1);
    nh.param("use_3d", use_3d, false);
    // Start and goal
    nh.param("start_x", start_x, 12.5);
    nh.param("start_y", start_y, 1.4);
    nh.param("start_z", start_z, 0.0);
    nh.param("start_vx", start_vx, 0.0);
    nh.param("start_vy", start_vy, 0.0);
    nh.param("start_vz", start_vz, 0.0);
    nh.param("goal_x", goal_x, 6.4);
    nh.param("goal_y", goal_y, 16.6);
    nh.param("goal_z", goal_z, 0.0);
    nh.param("use_acc", use_acc, true);
    nh.param("use_jrk", use_jrk, true);

    // Read prior traj
    nh.param("traj_file", traj_file_name, std::string(""));
    nh.param("traj_topic", traj_topic_name, std::string(""));
    nh.param("use_prior", use_prior, false);
  }

  ros::Publisher es_pub;
  ros::Publisher sg_pub;
  ros::Publisher traj_pub;
  ros::Publisher prior_traj_pub;
  ros::Publisher cloud_pub;
  ros::Publisher ps_pub;
  std::string file_name, topic_name;

  // read map
  sensor_msgs::PointCloud map;

  double robot_radius;
  Vec3f origin, dim;
  double dt, v_max, a_max, w, epsilon, t_max;
  double u_max_z, u_max;
  int max_num, num;
  bool use_3d;

  // start and goal variables
  double start_x, start_y, start_z;
  double start_vx, start_vy, start_vz;
  double goal_x, goal_y, goal_z;
  bool use_acc, use_jrk;

  Waypoint3 goal;
  Waypoint3 start;

  // Read prior traj
  std::string traj_file_name, traj_topic_name;
  bool use_prior;

  void callback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void read_map();
  void initialize_planner();
  void execute_planner();
  void publish_stuff();
};

void sub_pcl_plan_class::callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO("Recived\n");
}

void sub_pcl_plan_class::read_map()
{
  ros::Time t0 = ros::Time::now();
  // Read map from bag file
  map = read_bag<sensor_msgs::PointCloud>(file_name, topic_name, 0).back();
  cloud_pub.publish(map);
  ROS_INFO("Takes %f sec to set up map!", (ros::Time::now() - t0).toSec());
}

void sub_pcl_plan_class::publish_stuff()
{
  // Publish location of start and goal
  sensor_msgs::PointCloud sg_cloud;
  sg_cloud.header.frame_id = "map";
  geometry_msgs::Point32 pt1, pt2;
  pt1.x = start_x, pt1.y = start_y, pt1.z = start_z;
  pt2.x = goal_x, pt2.y = goal_y, pt2.z = goal_z;
  sg_cloud.points.push_back(pt1), sg_cloud.points.push_back(pt2);
  sg_pub.publish(sg_cloud);

  // Publish expanded nodes
  sensor_msgs::PointCloud ps = vec_to_cloud(planner_->getCloseSet());
  ps.header.frame_id = "map";
  ps_pub.publish(ps);
}

void sub_pcl_plan_class::execute_planner()
{
  ros::Time t0 = ros::Time::now();
  bool valid = planner_->plan(start, goal);
  if (!valid)
  {
    ROS_WARN("Failed! Takes %f sec for planning, expand [%zu] nodes", (ros::Time::now() - t0).toSec(),
             planner_->getCloseSet().size());
  }
  else
  {
    ROS_INFO("Succeed! Takes %f sec for planning, expand [%zu] nodes", (ros::Time::now() - t0).toSec(),
             planner_->getCloseSet().size());

    // Publish trajectory
    auto traj = planner_->getTraj();
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header.frame_id = "map";
    traj_pub.publish(traj_msg);

    printf("================== Traj -- total J(1): %f, J(2): %F, J(3): %f, total time: %f\n", traj.J(1), traj.J(2),
           traj.J(3), traj.getTotalTime());

    vec_Ellipsoid Es = sample_ellipsoids(traj, Vec3f(robot_radius, robot_radius, 0.1), 50);
    decomp_ros_msgs::Ellipsoids es_msg = DecompROS::ellipsoids_to_ros(Es);
    es_msg.header.frame_id = "map";
    es_pub.publish(es_msg);

    max_attitude(traj, 1000);
  }
}

void sub_pcl_plan_class::initialize_planner()
{
  // Initialize planner

  planner_.reset(new MPCloudUtil(true));
  planner_->setMap(cloud_to_vec(map), robot_radius, origin, dim);  // Set collision checking function
                                                                   // cloud_to_vec transfroms a poincloud to a vector
  planner_->setEpsilon(epsilon);                                   // Set greedy param (default equal to 1)
  planner_->setVmax(v_max);                                        // Set max velocity
  planner_->setAmax(a_max);                                        // Set max acceleration
  planner_->setUmax(u_max);                                        // Set max control
  planner_->setTmax(t_max);                                        // Set max time
  planner_->setDt(dt);                                             // Set dt for each primitive
  planner_->setW(w);                                               // Set w for each primitive
  planner_->setMaxNum(max_num);       // Set maximum allowed expansion, -1 means no limitation
  planner_->setTol(2.0, 2.0, 100.0);  // Tolerance for goal region

  start.pos = Vec3f(start_x, start_y, start_z);
  start.vel = Vec3f(start_vx, start_vy, start_vz);
  start.acc = Vec3f(0, 0, 0);
  start.jrk = Vec3f(0, 0, 0);
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = use_acc;
  start.use_jrk = use_jrk;

  goal.pos = Vec3f(goal_x, goal_y, goal_z);
  goal.vel = Vec3f(0, 0, 0);
  goal.acc = Vec3f(0, 0, 0);
  goal.use_pos = start.use_pos;
  goal.use_vel = start.use_vel;
  goal.use_acc = start.use_acc;
  goal.use_jrk = start.use_jrk;

  // Prior trajectory
  if (!traj_file_name.empty())
  {
    planning_ros_msgs::Trajectory prior_traj =
        read_bag<planning_ros_msgs::Trajectory>(traj_file_name, traj_topic_name, 0).back();
    if (!prior_traj.primitives.empty())
    {
      prior_traj_pub.publish(prior_traj);
      if (use_prior)
      {
        planner_->setPriorTrajectory(toTrajectory3(prior_traj));
        goal.use_acc = false;
        goal.use_jrk = false;
      }
    }
  }

  // Set input control
  vec_Vec3f U;
  const decimal_t du = u_max / num;
  if (use_3d)
  {
    decimal_t du_z = u_max_z / num;
    for (decimal_t dx = -u_max; dx <= u_max; dx += du)
      for (decimal_t dy = -u_max; dy <= u_max; dy += du)
        for (decimal_t dz = -u_max_z; dz <= u_max_z; dz += du_z)  // here we reduce the z control
          U.push_back(Vec3f(dx, dy, dz));  // U is a vector that contains all the possible combinations of (ux,uy,uz)
  }
  else
  {
    for (decimal_t dx = -u_max; dx <= u_max; dx += du)
      for (decimal_t dy = -u_max; dy <= u_max; dy += du)
        U.push_back(Vec3f(dx, dy, 0));
  }
  planner_->setU(U);  // Set discretization with 1 and efforts
  // planner_->setMode(num, use_3d, start); // Set discretization with 1 and efforts
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");
  sub_pcl_plan_class sub_pcl_plan_objeto(nh);

  sub_pcl_plan_objeto.read_map();
  sub_pcl_plan_objeto.initialize_planner();
  sub_pcl_plan_objeto.execute_planner();
  sub_pcl_plan_objeto.publish_stuff();
  ros::spin();

  return 0;
}
