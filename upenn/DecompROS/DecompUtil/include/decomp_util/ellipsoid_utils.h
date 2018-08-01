#include <decomp_util/geometry_utils.h>

bool max_ellipsoid(const Vec3f& pt, const Polyhedron& vs,
                   Ellipsoid& E, decimal_t max_radius = 100);
bool has_inlier(const Vec3f& pt, const Polyhedron& vs,
                const Quatf& qf, const Vec3f axes);
bool estimate_ellipsoid(const Vec3f& pt, const Polyhedron& poly,
                        const Quatf& q, int axes_id,
                        Vec3f& axes, Face& vb);

Ellipsoid find_ellipsoid(const Vec3f& pt, const vec_Vec3f& obs, vec_Vec3f& ps, const Vec3f& fs);
Ellipsoid find_sphere(const Vec3f& pt, const vec_Vec3f& obs, vec_Vec3f& ps, decimal_t f);
