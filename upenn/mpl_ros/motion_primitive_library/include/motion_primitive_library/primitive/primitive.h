/**
 * @file primitive.h
 * @brief Primitive classes
 */

#ifndef PRIMITIVE_H
#define PRIMITIVE_H
#include <memory>
#include <motion_primitive_library/common/data_type.h>
#include "math.h"

/**
 * @brief State lattice
 *
 * State includes position, velocity, acceleration and jerk in \f$R^n\f$, where the dimension \f$n\f$ can be either 2 or 3.
 */
template <int Dim>
struct Waypoint {
  Vecf<Dim> pos; ///<position in \f$R^{Dim}\f$
  Vecf<Dim> vel; ///<velocity in \f$R^{Dim}\f$
  Vecf<Dim> acc; ///<acceleration in \f$R^{Dim}\f$
  Vecf<Dim> jrk; ///<jerk in \f$R^{Dim}\f$

  bool use_pos = false;///<If true, pos will be used in primitive generation
  bool use_vel = false;///<If true, vel will be used in primitive generation 
  bool use_acc = false;///<If true, acc will be used in primitive generation 
  bool use_jrk = false;///<If true, jrk will be used in primitive generation 

  ///Print all the useful attributes
  void print(std::string str = "") const {
    if(!str.empty())
      std::cout << str << std::endl;
    std::cout << "pos: " << pos.transpose() << std::endl;
    std::cout << "vel: " << vel.transpose() << std::endl;
    std::cout << "acc: " << acc.transpose() << std::endl;
    std::cout << "jrk: " << jrk.transpose() << std::endl;
    std::cout << "use_pos: " << use_pos << std::endl;
    std::cout << "use_vel: " << use_vel << std::endl;
    std::cout << "use_acc: " << use_acc << std::endl;
    std::cout << "use_jrk: " << use_jrk << std::endl;
  }

  /**
   * @brief  Check if two waypoints are equivalent
   *
   * We compare the attriute if the corresponding flag `use_xxx` of either Waypoint is true.
   */
  bool operator==(const Waypoint<Dim>& n) const {
    /*
    return this->pos == n.pos &&
      this->vel == n.vel &&
      this->acc == n.acc &&
      this->jrk == n.jrk;
      */
    if((this->use_pos || n.use_pos) && this->pos != n.pos)
      return false;
    if((this->use_vel || n.use_vel) && this->vel != n.vel)
      return false;
    if((this->use_acc || n.use_acc) && this->acc != n.acc)
      return false;
    if((this->use_jrk || n.use_jrk) && this->jrk != n.jrk)
      return false;
    return true;
  }

  ///Check if two waypoints are not equivalent
  bool operator!=(const Waypoint<Dim>& n) {
    return !(*this == n);
  }
};

/**
 * @brief Primitive1D class
 *
 * Assume the 1D primitive is the n-th order polynomial with n = 5 as 
 * \f$p(t) = \frac{c(0)}{120}t^5+\frac{c(1)}{24}t^4+\frac{c(2)}{6}t^3+\frac{c(3)}{2}t^2+c(4)t+c(5) = 0\f$
 */
class Primitive1D {
  public:
    /**
     *@brief Empty constructor
     */
    Primitive1D();
    /**
     * @brief Construct from known coefficients
     * @param coeff[0] is the coefficient of the highest order
     */
    Primitive1D(const Vec6f& coeff);
    /**
     * @brief Construct 1D primitive from an initial state (p) and an input control (u)
     */
    Primitive1D(decimal_t p, decimal_t u);
    /**
     * @brief Construct 1D primitive from an initial state (p, v) and an input control (u)
     */
    Primitive1D(Vec2f state, decimal_t u);
    /**
     * @brief Construct 1D primitive from an initial state (p, v, a) and an input control (u)
     */
    Primitive1D(Vec3f state, decimal_t u);
    /**
     * @brief Construct 1D primitive from an initial state (p, v, a, j) and an input control (u)
     */
    Primitive1D(Vec4f state, decimal_t u);
    /**
    * @brief Construct 1D primitive from an initial state (p1) to a goal state (p2), given duration t
     */
    Primitive1D(decimal_t p1, decimal_t p2,  decimal_t t);
    /**
     * @brief Construct 1D primitive from an initial state (p1, v1) to a goal state (p2, v2), given duration t 
     */
    Primitive1D(decimal_t p1, decimal_t v1, decimal_t p2, decimal_t v2, decimal_t t);
    /**
     * @brief Construct 1D primitive from an initial state (p1, v1, a1) to a goal state (p2, v2, a2), given duration t
     */
    Primitive1D(decimal_t p1, decimal_t v1, decimal_t a1, decimal_t p2, decimal_t v2, decimal_t a2, decimal_t t);
    /**
     * @brief Return total efforts of 1D primitive for the given duration: \f$J(t, i) = \int_0^t |p^{i}(t)|^2dt\f$
     * @param t assume the duration is from 0 to t
     * @param i effort is defined as \f$i\f$-th derivative of polynomial
     */
    decimal_t J(decimal_t t, int i) const;
    /**
     * @brief Return coffecients
     */
    Vec6f coeff() const;
    /** 
     * @brief Return (p, v, a) at t, deault v, a are zeros
     */
    Vec4f evaluate(decimal_t t) const;
    /**
     * @brief Return extrema of velocity, velocities at both ends (0, t) are considered
     */
    std::vector<decimal_t> extrema_vel(decimal_t t) const;
    /**
     * @brief Return extrema of acceleration, accelerations at both ends (0, t) are considered
     */
    std::vector<decimal_t> extrema_acc(decimal_t t) const;
    /**
     * @brief Return extrema of jerk, jerk at both ends (0, t) are considered
     */
    std::vector<decimal_t> extrema_jrk(decimal_t t) const;
 
  public:
    /**@brief Coefficients*/
    Vec6f c;
};


/** 
 * @brief Primitive class
 *
 * Contains \f$n\f$ 1D primitives corresponding to each axis individually.
 */
template <int Dim>
class Primitive {
 public:
  /**
   * @brief Empty constructor
   */
  Primitive();
  /**
   * @brief Construct from an initial state p and an input control u for a given duration t
   */
  Primitive(const Waypoint<Dim>& p, const Vecf<Dim>& u, decimal_t t);
  /**
   * @brief Construct from an initial state p1 and a goal state p2 for a given duration t
   */
  Primitive(const Waypoint<Dim>& p1, const Waypoint<Dim>& p2, decimal_t t);
  /**
   * @brief Construct from given coefficients and duration
   */
  Primitive(const vec_E<Vec6f>& cs, decimal_t t);
  /**
   * @brief Return state at t
   *
   * Note: no flag `use_xxx` set in the returned waypoint
   */
  Waypoint<Dim> evaluate(decimal_t t) const;
  /** 
   * @brief Return duration
   */
  decimal_t t() const;
  /**
   * @brief Retrieve the 1D primitive
   * @param k indicates the retrieved dimension: 0-x, 1-y, 2-z
   */
  Primitive1D traj(int k) const;
  /**
   * @brief Return max velocity along k-th dimension
   */
  decimal_t max_vel(int k) const;
  /**
   * @brief Return max accleration along k-th dimension
   */
  decimal_t max_acc(int k) const;
  /**
   * @brief Return max jerk along k-th dimension
   */
  decimal_t max_jrk(int k) const;
  /**
   * @brief Check if the max velocity magnitude is within the threshold
   * @param mv is the max threshold 
   */
  bool valid_vel(decimal_t mv) const;
  /**
   * @brief Check if the max acceleration magnitude is within the threshold
   * @param ma is the max threshold 
   */
  bool valid_acc(decimal_t ma) const;
  /**
   * @brief Check if the max jerk magnitude is within the threshold
   * @param mj is the max threshold 
   */
  bool valid_jrk(decimal_t mj) const;
  /**
   * @brief Return total efforts of primitive for the given duration: \f$J(i) = \int_0^t |p^{i}(t)|^2dt\f$
   *
   * Return J is the summation of efforts in all three dimensions
   * @param i effort is defined as \f$i\f$-th derivative of polynomial
   */
  decimal_t J(int i) const;
  /**
   * @brief Sample N states using uniformed time
   */
  vec_E<Waypoint<Dim>> sample(int N) const;
  /**
   * @brief Retrieve coefficients
   */
  vec_E<Vec6f> coeffs() const;
 protected:
  ///Duration
  decimal_t t_;
 public:
  ///By default, primitive class contains `Dim` 1D primitive
  std::array<Primitive1D, Dim> prs_;
};

///Waypoint for 2D
typedef Waypoint<2> Waypoint2;

///Waypoint for 3D
typedef Waypoint<3> Waypoint3;

///Primitive for 2D
typedef Primitive<2> Primitive2;

///Primitive for 3D
typedef Primitive<3> Primitive3;
#endif
