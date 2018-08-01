#ifndef GEOMETRIC_UTILS_H
#define GEOMETRIC_UTILS_H

#include <decomp_util/data_utils.h>
//#include <poly_utils/data_type.h>

//**** Sort poits in an order
vec_Vec3f sort_pts(const vec_Vec3f &pts);

//**** Determine if a point p is inside polytope
bool inside_polytope(const Vec3f &p,
                     const Polyhedron &Vs,
                     decimal_t epsilon = 0);

//**** Find intersection between two Line
//*** return false if they are not intersected
bool intersect(const std::pair<Vec3f, Vec3f> &v1, 
    const std::pair<Vec3f, Vec3f> &v2, Vec3f &pi);

//**** Find intersection between multiple lines
vec_Vec3f line_intersects(const vec_E<std::pair<Vec3f, Vec3f>> &lines);

//**** Find extreme points of polytope
std::pair<BoundVec3f, std::vector<bool>> cal_extreme_points(const Polyhedron &vts);

Vec3f intersect(const pair_Vec3f &v1, const pair_Vec3f &v2);
vec_Vec3f inflate(const vec_Vec3f& c1, double r);
#endif
