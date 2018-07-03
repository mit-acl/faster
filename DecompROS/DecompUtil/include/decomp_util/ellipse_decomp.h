/**
 * @file ellipse_decomp.h
 * @brief EllipseDecomp Class
 */
#ifndef ELLIPSE_DECOMP_H
#define ELLIPSE_DECOMP_H

#define USE_CENTROID 1

#include <decomp_util/data_type.h>
#include <time.h>
#include <memory>
#include "line_segment.h"
#include "max_ellipsoid.hpp"

/**
 * @brief EllipseDecomp Class
 *
 * EllipseDecomp takes input as a given path and find the Safe Flight Corridor around it using Ellipsoids
 */
class EllipseDecomp {
public:
  ///Simple constructor
  EllipseDecomp(bool verbose = false);
  /**
   * @brief Basic constructor
   * @param origin The origin of the global bounding box
   * @param dim The dimension of the global bounding box
   */
  EllipseDecomp(const Vec3f &origin, const Vec3f &dim, bool verbose = false);
  ///Set obstacle points
  void set_obstacles(const vec_Vec3f &obs) { obs_ = obs; }
  ///Set dimension of virtual bounding box
  void set_virtual_box(const Vec3f& v) { virtual_ = v; }

  ///Get the path that is used for dilation
  vec_Vec3f get_dilate_path() const { return dilate_path_; }
  ///Get the new path in the center of the Safe Flight Corridor
  vec_Vec3f get_center_path() const { return center_path_; }
  ///Get the Safe Flight Corridor
  Polyhedra get_polyhedra() const { return polyhedrons_; }
  ///Get the intersected part of Safe Flight Corridor
  Polyhedra get_intersect_polyhedra() const { return intersect_polyhedrons_; }
  ///Get the ellipsoids
  vec_Ellipsoid get_ellipsoids() const { return ellipsoids_; }
  ///Get the constraints of SFC as \f$Ax\leq b \f$
  vec_LinearConstraint3f get_constraints();
  ///Calculate the total volume of the SFC
  decimal_t get_corridor_volume();
  ///Calculate the total volume of the ellipsoids
  decimal_t get_ellipsoid_volume();

  ///Clean both dilation and obstacles, reset the whole class
  void clean();

  /**
   * @brief Decomposition thread
   * @param poses The path to dilate
   * @param offset_x offset added to the long semi-axis, default is 0
   */
  bool decomp(const vec_Vec3f &poses, double offset_x = 0);

  /**
   * @brief Shrink the safe flight corridor
   * @param path path that used to shirnk
   * @param shrink_distance shrinking distance, should be positive
   */
  void shrink(const vec_Vec3f &path, double shrink_distance);

protected:
  void clear();
  void add_bounding(Polyhedron &Vs);

  vec_Vec3f cal_centers(const Polyhedra &Vs);

  vec_Vec3f dilate_path_;
  vec_Vec3f center_path_;
  vec_Vec3f obs_;

  vec_Ellipsoid ellipsoids_;
  Polyhedra polyhedrons_;
  Polyhedra intersect_polyhedrons_;
  std::vector<std::shared_ptr<LineSegment>> lines_;

  Vec3f min_; // bounding box params
  Vec3f max_;
  bool has_bounding_box_ = false;

  bool verbose_ = false;

  Vec3f virtual_ = Vec3f(0, 0, 0);
};
#endif
