/**
 * @file geometric_utils.h
 * @brief basic geometry utils
 */
#ifndef DECOMP_GEOMETRIC_UTILS_H
#define DECOMP_GEOMETRIC_UTILS_H

#include <decomp_util/data_utils.h>
#include <iostream>
#include <Eigen/Eigenvalues>

///Compensate for numerical error
constexpr decimal_t epsilon_ = 1e-6; // numerical calculation effect

//**** Calculate eigen values
Vec3f eigen_value(const Mat3f& A);

//**** Sort poits in an order
vec_Vec3f sort_pts(const vec_Vec3f &pts);

inline decimal_t dist(const Vec3f& p, const Face& v){
  return std::fabs(v.n.dot(p - v.p));
}

//**** Determine if a point p is inside polytope
bool inside_polytope(const Vec3f &p,
                     const Polyhedron &Vs,
                     decimal_t epsilon = epsilon_);

//**** Determine if a point p is inside polytop
bool inside_polytope(const Vec3f &p,
                     const LinearConstraint3f& C);

///Find the closest point, return the half-plane
Face closest_obstacle(const Ellipsoid &E, const vec_Vec3f &O);
///Calculate points inside the given polyhedron
vec_Vec3f ps_in_polytope(const Polyhedron &Vs, const vec_Vec3f &O);
//**** Find normals
//*** used for visualization
vec_E<pair_Vec3f> cal_normals(const Polyhedron &vts);

//**** Construct Ax <= b
//*** p0 must be inside polytope
LinearConstraint3f cal_Axb(const Vec3f& p0,
                           const Polyhedron &Vs);

//**** Find intersection between two Line
//*** return false if they are not intersected
bool intersect(const pair_Vec3f &v1, const pair_Vec3f &v2, Vec3f &pi);

//**** Find intersection between multiple lines
vec_Vec3f line_intersects(const vec_E<pair_Vec3f> &lines);

//**** Find extreme points of polytope
vec_E<vec_Vec3f> cal_extreme_points(const Polyhedron &vts);

//**** Find intersect polygon between a plane and polytope
vec_Vec3f plane_polytope_intersection(const Face &plane,
                                      const Polyhedron& vts);

//**** uniformly sample path into many segments
vec_Vec3f path_downsample(const vec_Vec3f& ps, decimal_t d);
vec_Vec3f path_downsample_i(const vec_Vec3f& ps, int cnt);

//**** crop path
vec_Vec3f path_crop(const vec_Vec3f& path, decimal_t d);

//**** find the intersection of two polytopes
Polyhedron polytope_intersection(const Polyhedron& vs1,
    const Polyhedron& vs2);

//**** Create triangles from a face
vec_E<vec_Vec3f> chop_triangle(const vec_Vec3f& pts);

//**** Calculate the volume of a polytope
decimal_t cal_volume(const vec_E<vec_Vec3f>& fs, const Vec3f& pt_inside);

//**** Find the centroid of a polygon
Vec3f cal_centroid_2d(const vec_Vec3f &pts, const Face&p);


//**** Calculate centroid of a polytope
Vec3f cal_centroid_3d(const vec_E<vec_Vec3f>& fs, const Vec3f& pt_inside);

decimal_t cal_closest_dist(const Vec3f& pt, const Polyhedron& vs);


Quatf vec_to_quaternion(const Vec3f &v);

bool inside_ellipsoid(const Ellipsoid &E,
                      const vec_Vec3f &O);

bool closest_pt(const Ellipsoid &E,
    const vec_Vec3f &O,
    Vec3f &best_v, int& id);

vec_Vec3f ps_in_ellipsoid(const Ellipsoid &E,
                          const vec_Vec3f &O);

#endif
