/**
 * @file sub_voxel_map_util.h
 * @brief SubVoxelMapUtil classes
 */

#ifndef MPL_SUB_VOXEL_MAP_UTIL_H
#define MPL_SUB_VOXEL_MAP_UTIL_H

#include <motion_primitive_library/collision_checking/voxel_map_util.h>

namespace MPL {
class SubVoxelMapUtil : public VoxelMapUtil {
  public:
    SubVoxelMapUtil() {}

    void setMap(const Vec3f& ori, const Vec3i& dim, const std::vector<signed char> &map, decimal_t res) {
      map_ = map;
      dim_ = dim;
      origin_d_ = ori;
      res_ = res;
      dim_low_ = Vec3i::Zero();
      dim_up_ = dim_;
    }


    void create(const Vec3f& pt, const Vec3f& origin, const Vec3f& dim) {
      Vec3i dim1 = floatToInt(pt + origin);
      for(int i = 0; i < 3; i++)
        dim_low_(i) = dim1(i) < 0 ? 0 : dim1(i);

      Vec3i dim2 = floatToInt(pt + origin + dim);
      for(int i = 0; i < 3; i++)
        dim_up_(i) = dim2(i) > dim_(i) ? dim_(i) : dim2(i);
    }

    void info() {
      printf("SubVoxelMap Info ============\n");
      printf("   res: [%f]\n", res_);
      printf("   origin: [%f, %f, %f]\n", origin_d_(0), origin_d_(1), origin_d_(2));
      printf("   dim: [%d, %d, %d]\n", dim_(0), dim_(1), dim_(2));
      printf("   size: [%d]\n", dim_(0) * dim_(1) *dim_(2));
      printf("   dim_low: [%d, %d, %d]\n", dim_low_(0), dim_low_(1), dim_low_(2));
      printf("   dim_up: [%d, %d, %d]\n", dim_up_(0), dim_up_(1), dim_up_(2));
    }

    bool isOutSide(const Vec3i &pn) {
      return pn(0) < dim_low_(0) || pn(0) >= dim_up_(0) ||
        pn(1) < dim_low_(1) || pn(1) >= dim_up_(1) ||
        pn(2) < dim_low_(2) || pn(2) >= dim_up_(2);
    }

    bool isEdge(const Vec3i &n) {
      return n(0) == dim_low_(0) || n(0) == dim_up_(0) - 1 ||
        n(1) == dim_low_(1) || n(1) == dim_up_(1) - 1;
      //n(2) == 0 || n(2) == dim_(2) - 1;
    }

    Vec3i getDimLow() { return dim_low_; }

    Vec3i getDimUp() { return dim_up_; }

    std::vector<signed char> getSubMap() {
      std::vector<signed char> submap;
      Vec3i dim = dim_up_- dim_low_;
      submap.resize(dim(0) * dim(1) * dim(2));
      Vec3i n;
      int cnt = 0;
      for (n(2) = dim_low_(2); n(2) < dim_up_(2); n(2)++) {
        for (n(1) = dim_low_(1); n(1) < dim_up_(1); n(1)++) {
          for (n(0) = dim_low_(0); n(0) < dim_up_(0); n(0)++) {
            submap[cnt] = map_[getIndex(n)];
            cnt ++;
          }
        }
      }
      return submap;
    }


  private:
    Vec3i dim_low_;
    Vec3i dim_up_;
};

}
#endif
