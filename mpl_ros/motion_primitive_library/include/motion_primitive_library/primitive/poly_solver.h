/**
 * @file poly_solver.h
 * @brief Trajectory generator back-end
 */
#ifndef POLY_SOLVER_H
#define POLY_SOLVER_H

#include <stdio.h>
#include <iostream>
#include <memory>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/LU>
#include <motion_primitive_library/primitive/poly_traj.h>

/**
 * @brief Trajectory generator back-end class
 *
 * Given intermediate waypoints and associated time allocation, generate the n-th order polynomials
 */
template <int Dim>
class PolySolver {
  public:
    /**
     * @brief Simple constructor
     * @param smooth_derivative_order The max derivative we want continuous
     * @param minimize_derivative The derivative to minimize
     */
    PolySolver(unsigned int smooth_derivative_order, unsigned int minimize_derivative);
    /**
     * @brief Solve the trajector as defined in constructor
     * @param waypoints Intermediate waypoints that the trajectory pass through
     * @param dts Time allocation for each segment
     *
     * Note that the element in dts is the time for that segment
     */
    bool solve(const vec_E<Waypoint<Dim>>& waypoints, const std::vector<decimal_t> &dts);

    ///Get the solved trajectory
    std::shared_ptr<PolyTraj<Dim>> getTrajectory();

  private:
    unsigned int N_;
    unsigned int R_;
    bool debug_;
    std::shared_ptr<PolyTraj<Dim>> ptraj_;
};

///PolySolver for 2D
typedef PolySolver<2> PolySolver2;

///PolySolver for 3D
typedef PolySolver<3> PolySolver3;
#endif
