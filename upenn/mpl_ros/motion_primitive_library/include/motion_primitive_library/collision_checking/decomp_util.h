/**
 * @file decomp_util.h
 * @brief decomp util util class
 */
#ifndef DECOMP_UTIL_H
#define DECOMP_UTIL_H
#include <motion_primitive_library/primitive/primitive.h>
#include <motion_primitive_library/primitive/primitive_util.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/make_shared.hpp>

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
typedef pcl::KdTreeFLANN<PCLPoint> KDTree;

/**
 * @brief Collision checking inside a Safe Flight Corridor (SFC)
 *
 * SFC is an ordered collection of convex polyhedra that models free space
 */
class DecompUtil {
  public:
    /**
     * @brief Simple constructor
     * @param r robot radius
     * @param h robot height, default as 0.1m
     */
    DecompUtil(decimal_t r, decimal_t h = 0.1);
    ///Set obstacles
    void setObstacles(const vec_Vec3f& obs);
    ///Set bounding box
    void set_region(const Vec3f& ori, const Vec3f& dim);
    ///Get bounding box
    Polyhedron virtual_wall(const Vec3f& pos);

    ///Get polyhedra
    Polyhedra polyhedra();
    ///Check if a primitive is inside the SFC from \f$t: 0 \rightarrow dt\f$
    bool isFree(const Primitive3& pr);
    ///Convert obstacle points into pcl point cloud
    PCLPointCloud toPCL(const vec_Vec3f &obs);
 private:
    ///Check if a point pt is inside the given polyhedron
    bool insidePolyhedron(const Vec3f &pt, const Polyhedron &Vs);
    ///Check if a point in O is inside the given ellipsoid
    bool insideEllipsoid(const Ellipsoid& E, const vec_Vec3f& O);

    ///robot size: axe = (r, r, h)
    Vec3f axe_;
    ///obstacle points
    vec_Vec3f obs_;
    ///obstacles in kd tree form
    KDTree kdtree_;
    ///Bounding box
    Polyhedron Vs_;
};
#endif


