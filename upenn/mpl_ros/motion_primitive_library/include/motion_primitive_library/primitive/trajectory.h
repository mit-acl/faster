/**
 * @file trajectory.h
 * @brief Trajectory class
 */


#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <motion_primitive_library/primitive/primitive.h>

/**
 * @brief Used for scaling, ignored for most case
 */
struct VirtualPoint{
  decimal_t p;
  decimal_t v;
  decimal_t t;
};


/**
 * @brief polynomial between two virtual points
 */
class LambdaSeg {
  public:
    LambdaSeg() {}
    LambdaSeg(const VirtualPoint& v1, const VirtualPoint& v2);
    VirtualPoint evaluate(decimal_t t) const;
    decimal_t getT(decimal_t t) const;
    Vec4f a; // a3, a2, a1, a0
    decimal_t ti;
    decimal_t tf;

    decimal_t dT;
};

/**
 * @brief piecewise polynomial for trajectory
 *
 */
class Lambda {
  public:
    Lambda() {}
    Lambda(const std::vector<VirtualPoint>& vs);

    bool exist() const;
    std::vector<VirtualPoint> sample(int N);
    vec_Vec3f sampleT(int N);
    VirtualPoint evaluate(decimal_t t) const;

    decimal_t getT(decimal_t tau) const;
    decimal_t getTau(decimal_t t) const;
    decimal_t getTotalTime() const;
    std::vector<LambdaSeg> segs;
};


/**
 * @brief Trajectory class
 *
 * A trajectory is composed by multiple end-to-end connected primitives, so-called piece-wise polynomials
 */
template <int Dim>
class Trajectory {
  public:
    /**
     * @brief Empty constructor
     */
    Trajectory() {}
    /**
     * @brief Construct from multiple primitives
     */
    Trajectory(const vec_E<Primitive<Dim>>& prs);
    /**
     * @brief Return the total duration of the trajectory
     */
    decimal_t getTotalTime() const;
    /**
     * @brief Retrieve scaling factor
     */
    Lambda lambda() const;
    /**
     * @brief Evaluate state at t, return false if fails to evaluate
     *
     * If t is out of scope, we set t to be the closer bound (0 or total_t_) and return the evaluation;
     * The failure case is when lambda is ill-posed such that \f$t = \lambda(\tau)^{-1}\f$ has no solution
     */ 
    bool evaluate(decimal_t t, Waypoint<Dim>& p) const;
    /**
     * @brief Scale according to ratio at start and end (velocity only)
     */
    bool scale(decimal_t ri, decimal_t rf);
    /**
     * @brief Scale down the whole trajectory according to mv
     */
    bool scale_down(decimal_t mv, decimal_t ri, decimal_t rf);
    /**
     * @brief Sample N states using uniformed time
     */
    vec_E<Waypoint<Dim>> sample(int N) const;
    /**
     * @brief Return total efforts of primitive for the given duration: \f$J(i) = \int_0^t |p^{(i+1)}(t)|^2dt\f$
     *
     * Return J is the summation of efforts in all three dimensions
     * @param i effort is defined as (i+1)-th derivative of polynomial
     */
    decimal_t J(int i) const;
    ///Get time for each segment
    std::vector<decimal_t> getSegsT() const;

    ///Segments of primitives
    vec_E<Primitive<Dim>> segs;
    ///Time in virtual domain
    std::vector<decimal_t> taus;
    ///Time in actual domain
    std::vector<decimal_t> Ts;
    ///Total time of the trajectory
    decimal_t total_t_;
    ///Scaling object
    Lambda lambda_;
};

///Trajectory in 2D
typedef Trajectory<2> Trajectory2;

///Trajectory in 3D
typedef Trajectory<3> Trajectory3;

#endif
