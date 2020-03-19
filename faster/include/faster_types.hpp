/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

struct polytope
{
  Eigen::MatrixXd A;
  Eigen::MatrixXd b;
};

struct parameters
{
  bool use_ff;
  bool visual;

  double dc;
  double goal_radius;
  double drone_radius;

  int N_whole;
  int N_safe;

  double Ra;
  double w_max;
  double alpha_filter_dyaw;

  double z_ground;
  double z_max;
  double inflation_jps;
  double factor_jps;

  double v_max;
  double a_max;
  double j_max;

  double gamma_whole;
  double gammap_whole;
  double increment_whole;
  double gamma_safe;
  double gammap_safe;
  double increment_safe;

  double delta_a;
  double delta_H;

  int max_poly_whole;
  int max_poly_safe;
  double dist_max_vertexes;

  int gurobi_threads;
  int gurobi_verbose;

  bool use_faster;

  double wdx;
  double wdy;
  double wdz;
  double res;

  bool is_ground_robot;

  /*  double kw;
    double kyaw;
    double kdalpha;
    double kv;
    double kdist;
    double kalpha;*/
};

struct state
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk = Eigen::Vector3d::Zero();

  double yaw = 0;
  double dyaw = 0;

  void setPos(const double x, const double y, const double z)
  {
    pos << x, y, z;
  }
  void setVel(const double x, const double y, const double z)
  {
    vel << x, y, z;
  }
  void setAccel(const double x, const double y, const double z)
  {
    accel << x, y, z;
  }

  void setJerk(const double x, const double y, const double z)
  {
    jerk << x, y, z;
  }

  void setPos(const Eigen::Vector3d& data)
  {
    pos << data.x(), data.y(), data.z();
  }

  void setVel(const Eigen::Vector3d& data)
  {
    vel << data.x(), data.y(), data.z();
  }

  void setAccel(const Eigen::Vector3d& data)
  {
    accel << data.x(), data.y(), data.z();
  }

  void setJerk(const Eigen::Vector3d& data)
  {
    jerk << data.x(), data.y(), data.z();
  }

  void setState(const Eigen::Matrix<double, 9, 1>& data)
  {
    pos << data(0, 0), data(1, 0), data(2, 0);
    vel << data(3, 0), data(4, 0), data(5, 0);
    accel << data(6, 0), data(7, 0), data(8, 0);
  }

  void setYaw(const double& data)
  {
    yaw = data;
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