#pragma once

//#include <pcl/kdtree/kdtree_flann.h>

/*struct kdTreeStamped
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
  ros::Time time;
};
*/
struct polytope
{
  Eigen::MatrixXd A;
  Eigen::MatrixXd b;
};

struct parameters
{
  bool use_ff;
  bool visual;
  double spinup_time;

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

struct state
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk = Eigen::Vector3d::Zero();

  double yaw = 0;
  double dyaw = 0;

  void setPos(double x, double y, double z)
  {
    pos << x, y, z;
  }
  void setVel(double x, double y, double z)
  {
    vel << x, y, z;
  }
  void setAccel(double x, double y, double z)
  {
    accel << x, y, z;
  }

  void setJerk(double x, double y, double z)
  {
    jerk << x, y, z;
  }

  void setPos(Eigen::Vector3d& data)
  {
    pos << data.x(), data.y(), data.z();
  }

  void setVel(Eigen::Vector3d& data)
  {
    vel << data.x(), data.y(), data.z();
  }

  void setAccel(Eigen::Vector3d& data)
  {
    accel << data.x(), data.y(), data.z();
  }

  void setJerk(Eigen::Vector3d& data)
  {
    jerk << data.x(), data.y(), data.z();
  }

  void setState(Eigen::Matrix<double, 9, 1>& data)
  {
    pos << data(0, 0), data(1, 0), data(2, 0);
    vel << data(3, 0), data(4, 0), data(5, 0);
    accel << data(6, 0), data(7, 0), data(8, 0);
  }
  void setZero()
  {
    pos = Eigen::Vector3d::Zero();
    vel = Eigen::Vector3d::Zero();
    accel = Eigen::Vector3d::Zero();
    jerk = Eigen::Vector3d::Zero();
    yaw = 0;
    dyaw = 0;
  }

  void printPos()
  {
    std::cout << "Pos= " << pos.transpose() << std::endl;
  }

  void print()
  {
    std::cout << "Pos= " << pos.transpose() << std::endl;
    std::cout << "Vel= " << vel.transpose() << std::endl;
    std::cout << "Accel= " << accel.transpose() << std::endl;
  }

  void printHorizontal()
  {
    std::cout << "Pos, Vel, Accel, Jerk= " << pos.transpose() << " " << vel.transpose() << " " << accel.transpose()
              << " " << jerk.transpose() << std::endl;
  }
};