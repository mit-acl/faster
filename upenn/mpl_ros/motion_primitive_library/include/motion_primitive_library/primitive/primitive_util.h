/**
 * @file primitive_util.h
 * @brief Simple primitive utils
 */
#ifndef PRIMITIVE_UTIL_H
#define PRIMITIVE_UTIL_H

#include <motion_primitive_library/primitive/primitive.h>
#include <motion_primitive_library/primitive/trajectory.h>

///Print all coefficients in primitive p
template <int Dim>
void print_coeffs(const Primitive<Dim>& p) {
  printf("coeffs: t = %f\n", p.t());
  for(int i = 0; i < Dim; i++)
    std::cout << i << ":     " << p.traj(i).coeff().transpose() << std::endl;
}

///Print max dynamic infomation in primitive p
template <int Dim>
void print_max(const Primitive<Dim>& p) {
  Vecf<Dim> max_v, max_a, max_j;
  for(int i = 0; i < Dim; i++) {
    max_v(i) = p.max_vel(i);
    max_a(i) = p.max_acc(i);
    max_j(i) = p.max_jrk(i);
  }
  std::cout << "max_vel: ", max_v.transpose() << std::endl;;
  std::cout << "max_acc: ", max_a.transpose() << std::endl;;
  std::cout << "max_jrk: ", max_j.transpose() << std::endl;;
}


/**
 * @brief estimate the ellipsoid in 3D
 * @param axe the length of semi-axes of the ellipsoid
 * @param pos center of ellipsoid 
 * @param acc acceleration
 */
template <int Dim>
Ellipsoid generate_ellipsoid(const Vec3f& axe, const Vec3f& pos, const Vec3f& acc) {
  const Vec3f b3 = (acc + 9.81 * Vec3f::UnitZ()).normalized();
  const Vec3f bc(std::cos(0), std::sin(0), 0);
  const Vec3f b2 = b3.cross(bc).normalized();
  const Vec3f b1 = b2.cross(b3).normalized();
  Mat3f R;
  R << b1, b2, b3;

  Mat3f C = Mat3f::Identity();
  C(0, 0) = axe(0);
  C(1, 1) = axe(1);
  C(2, 2) = axe(2);
  C = R * C * R.transpose();
  return std::make_pair(C, pos);
}

///Convert any vector to Vec3f
template <int Dim>
Vec3f to3D(const Vecf<Dim>& p) {
  Vec3f p3 = Vec3f::Zero();
  for(int i = 0; i < Dim; i++) 
    p3(i) = p(i);
  return p3;
}

///Sample N ellipsoids along the primitive
template <int Dim>
vec_Ellipsoid sample_ellipsoids(const Primitive<Dim>& pr, const Vec3f& axe, int N) {
  vec_Ellipsoid Es;
  decimal_t dt = pr.t() / N;
  for(decimal_t t = 0; t <= pr.t(); t+= dt) {
    const Waypoint<Dim> pt = pr.evaluate(t);
    Es.push_back(generate_ellipsoid<Dim>(axe, to3D(pt.pos), to3D(pt.acc)));
  }

  return Es;
}

///Sample N ellipsoids along the trajectory
template <int Dim>
vec_Ellipsoid sample_ellipsoids(const Trajectory<Dim>& traj, const Vec3f& axe, int N) {
  vec_Ellipsoid Es;

  decimal_t dt = traj.getTotalTime() / N;
  for(decimal_t t = 0; t <= traj.getTotalTime(); t+= dt) {
    Waypoint<Dim> pt;
    if(traj.evaluate(t, pt)) 
      Es.push_back(generate_ellipsoid<Dim>(axe, to3D(pt.pos), to3D(pt.acc)));
  }

  return Es;
}

///Approximate the maximum roll/pitch along the trajectory
template <int Dim>
void max_attitude(const Trajectory<Dim>& traj, int N) {
  decimal_t dt = traj.getTotalTime() / N;
  decimal_t max_roll = 0;
  decimal_t max_roll_time = 0;
  decimal_t max_pitch = 0;
  decimal_t max_pitch_time = 0;
  for(decimal_t t = 0; t <= traj.getTotalTime(); t+= dt) {
    Waypoint<Dim> pt;
    if(traj.evaluate(t, pt)) {
      const Vec3f b3 = (to3D(pt.acc) + 9.81 * Vec3f::UnitZ()).normalized();
      decimal_t roll = std::atan2(b3(1), b3(2));
      decimal_t pitch= std::atan2(b3(0), b3(2));
      if(std::fabs(roll) > std::fabs(max_roll)) {
        max_roll = roll;
        max_roll_time = t;
      }
      if(std::fabs(pitch) > std::fabs(max_pitch)) {
        max_pitch = pitch;
        max_pitch_time = t;
      }

    }
  }

  printf("max roll: %f at [%f]\n", max_roll * 180/M_PI, max_roll_time);
  printf("max pitch: %f at [%f]\n", max_pitch * 180/M_PI, max_pitch_time);
}


#endif

