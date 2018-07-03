/**
 * @file poly_traj.h
 * @brief Trajectory object for PolySolver
 */
#ifndef POLY_TRAJ_H
#define POLY_TRAJ_H

#include <motion_primitive_library/primitive/primitive.h>
#include <deque>

/**
 * @brief Trajectory class for solving n-th polynomial with PolySolver
 */
template <int Dim>
class PolyTraj {
  public:
    ///Simple constructor
    PolyTraj();
    ///Clear
    void clear();
    ///Set coefficients
    void addCoeff(const MatDNf<Dim>& coeff);
    ///Set time allocation
    void addTime(const std::vector<decimal_t> &dts);
    ///Convert to Primitive class
    vec_E<Primitive<Dim>> toPrimitives() const;
    ///Evaluate the waypoint at t
    Waypoint<Dim> evaluate(decimal_t t) const;
    ///Get the total time for the trajectory
    decimal_t getTotalTime() const;
  private:
    std::vector<decimal_t> waypoint_times_;
    std::vector<decimal_t> dts_;
    std::deque<MatDNf<Dim>, Eigen::aligned_allocator<MatDNf<Dim>>> coefficients_;
};

///PolyTraj in 2D
typedef PolyTraj<2> PolyTraj2;

///PolyTraj in 3D
typedef PolyTraj<3> PolyTraj3;
#endif
