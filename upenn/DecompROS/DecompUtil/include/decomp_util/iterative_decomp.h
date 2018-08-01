/**
 * @file iterative_decomp.h
 * @brief IterativeDecomp Class
 */
#ifndef ITERATIVE_DECOMP_H
#define ITERATIVE_DECOMP_H

#include <decomp_util/ellipse_decomp.h>

/**
 * @brief IterativeDecomp Class
 *
 * Iteratively calls ElliseDecomp to form a safer Safe Flight Corridor that is away from obstacles
 */
class IterativeDecomp : public EllipseDecomp
{
  public:
    ///Simple constructor
    IterativeDecomp(bool verbose = false);
    /**
     * @brief Basic constructor
     * @param origin The origin of the global bounding box
     * @param dim The dimension of the global bounding box
     */
    IterativeDecomp(const Vec3f &origin, const Vec3f &dim, bool verbose = false);
    /**
     * @brief Decomposition thread
     * @param poses The path to dilate
     * @param iter_num Max iteration number
     * @param offset_x offset added to the long semi-axis, default is 0
     */
    bool decomp_iter(const vec_Vec3f &poses, int iter_num = 5, double offset_x = 0);
  private:
    ///Remove redundant waypoints
    vec_Vec3f simplify(const vec_Vec3f& path);

};
#endif
