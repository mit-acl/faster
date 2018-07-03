/**
 * @file mp_cloud_util.h
 * @brief motion planning cloud util
 */
#include <motion_primitive_library/planner/env_cloud.h>
#include <motion_primitive_library/planner/mp_base_util.h>

/**
 * @brief Motion primitive planner using point cloud
 */
class MPCloudUtil : public MPBaseUtil<3>
{
  public:
    /**
     * @brief Simple constructor
     * @param verbose enable print out
     */
    MPCloudUtil(bool verbose);
    ///Set map util
    void setMap(const vec_Vec3f& obs, decimal_t r, const Vec3f& ori, const Vec3f& dim);
};
