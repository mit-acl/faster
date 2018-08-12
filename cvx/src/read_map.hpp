#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <jps_basis/data_utils.h>

template <class Ti, class Tf>
class MapReader
{
public:
  MapReader(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr, const Vec3f start, const Vec3f goal, const Vec3i dim,
            double res, const Vec3f center_map, bool verbose = false)
  {
    for (unsigned int i = 0; i < 3; i++)
    {
      /*      start_(i) = start[i];
            goal_(i) = goal[i];*/

      dim_(i) = dim[i];
    }

    // The origin of the map is the point (minX, minY, minZ) del map (it's a corner of the 3D cube defined by the
    // dimensions of the map) All the points in the map will be expressed in that frame
    origin_(0) = center_map[0] - res * dim[0] / 2;
    origin_(1) = center_map[1] - res * dim[1] / 2;
    origin_(2) = center_map[2] - res * dim[2] / 2;

    resolution_ = res;
    data_.resize(dim[0] * dim[1] * dim[2], 0);

    // printf("dimension de data_=%f\n", dim[0] * dim[1] * dim[2]);
    for (size_t i = 0; i < pclptr->points.size(); ++i)
    {
      double x = pclptr->points[i].x - center_map[0] + res * dim[0] / 2;  // x of the point expresed in a system of
                                                                          // coordinates that has as origin the (minX,
                                                                          // minY, minZ) point of the map
      double y = pclptr->points[i].y - center_map[1] + res * dim[1] / 2;
      double z = pclptr->points[i].z - center_map[2] + res * dim[2] / 2;
      int id = x + dim_(0) * y + dim_(0) * dim_(1) * z;
      // printf("intentando acceder a %f\n", x + dim_(0) * y + dim_(0) * dim_(1) * z);
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
  /*  double start(int i)
    {
      return start_(i);
    }*/
  /*  double goal(int i)
    {
      return goal_(i);
    }*/
  double resolution()
  {
    return resolution_;
  }
  std::vector<signed char> data()
  {
    return data_;
  }

private:
  /*  Tf start_;
    Tf goal_;*/
  Tf origin_;
  Ti dim_;

  double resolution_;
  std::vector<signed char> data_;
};
