/**
 * @file seed_decomp.h
 * @brief SeedDecomp Class
 */
#ifndef SEED_DECOMP_H
#define SEED_DECOMP_H

#include <decomp_util/data_type.h>
#include <decomp_util/geometry_utils.h>


/**
 * @brief Seed Decomp Class
 *
 * Dilate around the given point
 */
class SeedDecomp{
  public:
    ///Simple constructor
    SeedDecomp() {};
    /**
     * @brief Basic constructor
     * @param p1 One end of the line seg
     * @param p2 The other end of the line seg
     */
    SeedDecomp(const Vec3f &p);

    /**
     * @brief Adding virtual bounding box around line seg
     * @param x Length in x axis
     * @param y Length in y axis
     * @param z Length in z axis
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
    /**
     * @brief Inflate the seed with a sphere
     * @param radius Robot radius
     */
    void dilate(decimal_t radius);
    /**
     * @brief Inflate the seed with an ellipsoid
     * @param axes length of semi x,y,z-axes 
     * @param R rotation of the ellipsoid
     */
    void dilate(const Vec3f& axes, const Mat3f& R);
    /**
     * @brief Inflate the seed with an ellipsoid
     * @param E ellipsoid 
     */
    void dilate(const Ellipsoid& E);
    /**
     * @brief Shrink the polyhedron 
     * @param thr Shrinking distance
     */
    void shrink(decimal_t thr);
  private:
    ///Add the bounding box
    void add_virtual_wall(Polyhedron &Vs);

    ///Obstacle points
    vec_Vec3f obs_;
    ///Seed location
    Vec3f p_;
    ///Ellipsoid used for inflation
    Ellipsoid ellipsoid_;
    ///The polyhedron of free space
    Polyhedron polyhedron_;
    ///Length of bounding box in x-axis
    decimal_t virtual_x_ = 5;
    ///Length of bounding box in y-axis
    decimal_t virtual_y_ = 5;
    ///Length of bounding box in z-axis
    decimal_t virtual_z_ = 2;
};

#endif 
