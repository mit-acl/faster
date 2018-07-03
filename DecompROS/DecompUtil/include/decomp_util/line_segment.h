/**
 * @file line_segment.h
 * @brief LineSegment Class
 */
#ifndef LINE_SEGMENT_H
#define LINE_SEGMENT_H

#include <decomp_util/data_type.h>
#include <decomp_util/geometry_utils.h>

/**
 * @brief Line Segment Class
 *
 * The basic element in EllipseDecomp
 */
class LineSegment {
  public:
    ///Simple constructor
    LineSegment();
    /**
     * @brief Basic constructor
     * @param p1 One end of the line seg
     * @param p2 The other end of the line seg
     */
    LineSegment(const Vec3f &p1, const Vec3f &p2);
    /**
     * @brief Adding virtual bounding box around line seg
     * @param x Distance in x axis
     * @param y Distance in y axis
     * @param v Distance in z axis
     *
     * This virtual bounding box is parallel to the line segment, the x,y,z axes are not w.r.t the world coordinate system, but instead, x-axis is parallel to the line, y-axis is perpendicular to the line and world z-axis, z-axis is perpendiculat to the line and y-axis
     */
    void set_virtual_dim(decimal_t x, decimal_t y, decimal_t z);
    ///Import obstacle points
    void set_obstacles(const vec_Vec3f &obs);
    ///Retieve obstacel points
    vec_Vec3f obs() const { return obs_; }
    ///Retrieve ellipsoid
    Ellipsoid ellipsoid() const { return ellipsoid_; }
    ///Retrieve polyhedron
    Polyhedron polyhedron() const { return polyhedron_; }
    ///Calculate the volume of polyhedron
    decimal_t polyhedron_volume();
    ///Calculate the volume of ellipsoid
    decimal_t ellipsoid_volume();

    /**
     * @brief Infalte the line segment
     * @param radius the offset added to the long semi-axis
     */
    void dilate(decimal_t radius);
    /**
     * @brief Shrink the polyhedron
     * @param p1 One end of the line seg
     * @param p2 The other end of the line seg
     * @param shrink_distance Shrink distance
     */
    void shrink(const Vec3f& p1, const Vec3f& p2, double shrink_distance);
    /**
     * @brief Adjust the norm of half plane to prevent cutting off the line seg whhile shrinking
     *
     * Details are introduced in the related paper
     */
    void adjust(Face& v);

  protected:
    void add_virtual_wall(Polyhedron &Vs);
    void cal_spheroid(const Vec3f& pw, const Quatf& qi, const Vec3f& center,
        Vec3f& axes, Quatf& qf);

    Ellipsoid find_ellipsoid(const Vec3f &p1, const Vec3f &p2, double offset_x = 0);
    Polyhedron find_polyhedron(const Ellipsoid &E);

    // Input:
    Vec3f p1_;
    Vec3f p2_;
    vec_Vec3f obs_;

    // Output:
    Ellipsoid ellipsoid_;
    Polyhedron polyhedron_;

    decimal_t virtual_x_ = 5;
    decimal_t virtual_y_ = 5;
    decimal_t virtual_z_ = 2;
};
#endif
