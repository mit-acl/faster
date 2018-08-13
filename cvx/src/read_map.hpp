#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <jps_basis/data_utils.h>

template <class Ti, class Tf>
class MapReader
{
public:
  MapReader(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr, const Vec3i dim, double res, const Vec3f center_map)
  {
    // **Box of the map --> it's the box with which the map moves.
    // **Center_map --> The center of the box of the map, expressed in global float coordinates
    // **Global float coordinates: (X,Y,Z) of the point relative to the global origin of the world
    // **origin_ --> It's the point of the box of the map that minX, minY, minZ (it's a corner of the box of the map).
    // Expressed in global float coordinates

    //**Cell coordinates: coordinates representing a cell. It's relative to the origin_

    /// **Occupied cell: Cell that has value 100
    /// **Free cell: Cell that has value 0
    /// **Unknown cell: Cell that has value -1

    for (unsigned int i = 0; i < 3; i++)
    {
      dim_(i) = dim[i];
    }

    origin_(0) = center_map[0] - res * dim[0] / 2.0;
    origin_(1) = center_map[1] - res * dim[1] / 2.0;
    origin_(2) = center_map[2] - res * dim[2] / 2.0;

    resolution_ = res;
    data_.resize(dim[0] * dim[1] * dim[2], 0);

    for (size_t i = 0; i < pclptr->points.size(); ++i)
    {
      // Let's find the coordinates of the point expresed in a system of coordinates that has as origin the (minX,
      // minY, minZ) point of the map
      double x = std::round((pclptr->points[i].x - origin_(0)) / res - 0.5);
      double y = std::round((pclptr->points[i].y - origin_(1)) / res - 0.5);
      double z = std::round((pclptr->points[i].z - origin_(2)) / res - 0.5);
      // this next formula works only when x, y, z are in cell coordinates (relative to the origin of the map)
      int id = x + dim_(0) * y + dim_(0) * dim_(1) * z;
      data_[id] = 100;
    }
  }

  Tf origin()
  {
    return origin_;
  }
  Ti dim()
  {
    return dim_;
  }

  double resolution()
  {
    return resolution_;
  }
  std::vector<signed char> data()
  {
    return data_;
  }

private:
  Tf origin_;
  Ti dim_;

  double resolution_;
  std::vector<signed char> data_;
};
