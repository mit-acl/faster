#ifndef VOXEL_GRID_H
#define VOXEL_GRID_H

#include <motion_primitive_library/common/data_type.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <boost/multi_array.hpp>

class VoxelGrid {
 public:
  VoxelGrid(Vec3f origin, Vec3f dim, float res);
  void clear();

  vec_Vec3f getCloud();
  vec_Vec3f getLocalCloud(const Vec3f& pos, const Vec3f& ori, const Vec3f& dim);
  planning_ros_msgs::VoxelMap getMap();
  planning_ros_msgs::VoxelMap getInflatedMap();

  bool allocate(const Vec3f &new_dim_d, const Vec3f &new_ori_d);

  void addCloud(const vec_Vec3f &pts);

  vec_Vec3i addCloud(const vec_Vec3f &pts, const vec_Vec3i& ns);

  void decay();

  void fill(int nx, int ny);
  void fill(int nx, int ny, int nz);
  void clear(int nx, int ny);
 private:
  Vec3i floatToInt(const Vec3f& pt);
  Vec3f intToFloat(const Vec3i &pn);
  bool isOutSide(const Vec3i &pn);

  Vec3i dim_;
  Vec3i origin_;
  Vec3f origin_d_;
  float res_;
  boost::multi_array<char, 3> map_;
  boost::multi_array<char, 3> inflated_map_;

  const char val_free = 0;
  const char val_occ = 100;
  const char val_unknown = -1;
};

#endif
