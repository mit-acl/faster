/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

// Class JPS Manager
#include "ros/ros.h"
// Convex Decomposition includes
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>

#include "read_map.hpp"
#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>

#include <Eigen/Dense>

#include "utils.hpp"

#include <mutex>

class JPS_Manager
{
public:
  JPS_Manager();

  std::mutex mtx_jps_map_util;  // mutex for map_util_ and planner_ptr_

  std::shared_ptr<JPS::VoxelMapUtil> map_util_;
  std::unique_ptr<JPSPlanner3D> planner_ptr_;

  vec_Vec3f vec_o_;   // Vector that contains the occupied points
  vec_Vec3f vec_uo_;  // Vector that contains the unkown and occupied points

  // JPS
  void updateJPSMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr, Eigen::Vector3d& center);
  vec_Vecf<3> solveJPS3D(Vec3f& start, Vec3f& goal, bool* solved, int i);
  void setNumCells(int cells_x, int cells_y, int cells_z);

  // Convex Decomposition
  void cvxEllipsoidDecomp(vec_Vecf<3>& path, int type_space, std::vector<LinearConstraint3D>& l_constraints,
                          vec_E<Polyhedron<3>>& poly_out);

  void setResolution(double res);
  void setFactorJPS(double factor_jps);
  void setInflationJPS(double inflation_jps);

  void setZGroundAndZMax(double z_ground, double z_max);
  void setVisual(bool visual);
  void setDroneRadius(double inflation_jps);

private:
  double factor_jps_, res_, inflation_jps_, z_ground_, z_max_, drone_radius_;
  int cells_x_, cells_y_, cells_z_;
  bool visual_;
  EllipsoidDecomp3D ellip_decomp_util_;
};