// This file is a mofified version of https://github.com/sikang/motion_primitive_library/blob/master/test/read_map.hpp

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <jps_basis/data_utils.h>

#include <memory>

template <class Ti, class Tf>
class MapReader
{
public:
  MapReader(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr, int cells_x, int cells_y, int cells_z, double res,
            const Vec3f center_map, double z_ground, double z_max, double inflation)
  {
    // printf("reading_map\n");
    // **Box of the map --> it's the box with which the map moves.
    // **Center_map --> The center of the box of the map, expressed in global float coordinates
    // **Global float coordinates: (X,Y,Z) of the point relative to the global origin of the world
    // **origin_ --> It's the point of the box of the map that minX, minY, minZ (it's a corner of the box of the map).
    // Expressed in global float coordinates

    //**Cell coordinates: coordinates representing a cell. It's relative to the origin_, and they are always positive
    // numbers

    /// **Occupied cell: Cell that has value 100
    /// **Free cell: Cell that has value 0
    /// **Unknown cell: Cell that has value -1

    /// z_ground [m] is used to limit the map size and not include points that are below the ground (--> + efficiency
    /// and safety of the trajectories)

    /// inflation [m] is used to inflate the obstacles that amount. Also the map is inflated in x and y with 2*(that
    /// amount)

    // printf("In reader2\n");
    Vec3i dim(cells_x, cells_y, cells_z);
    // printf("dim[0] before is %d\n", dim[0]);

    dim[0] = dim[0] + (int)(5 * (inflation * 1.0) / res);
    dim[1] = dim[1] + (int)(5 * (inflation * 1.0) / res);

    // printf("dim[0] is %d\n", dim[0]);

    int dim2_down = dim[2] / 2.0;
    int dim2_up = dim[2] / 2.0;

    // printf("reading_map1\n");
    if (center_map[2] - res * dim[2] / 2.0 < 0)
    {
      // printf("modyfing");
      dim2_down = (int)((center_map[2] - z_ground) / res) + 1;  //+1 to avoid problems when taking off
      // dim2_up = (int)(dim[2] / 2.0);
    }

    if (center_map[2] + res * dim[2] / 2.0 > z_max)
    {
      // printf("modyfing");
      // dim2_down = (int)((center_map[2] - z_ground) / res) + 1;  //+1 to avoid problems when taking off
      dim2_up = (int)((z_max - center_map[2]) / res);  //+1 to avoid problems when taking off
      dim2_up = (dim2_up > 0) ? dim2_up : 1;           // Force it to be >= 1
      // dim[2] = dim2_down + dim2_up;
    }
    // printf("z_max is %f\n", z_max);
    // printf("Dim_up is %d\n", dim2_up);
    dim[2] = dim2_down + dim2_up;
    // printf("*******Dim is\n");
    // std::cout << dim << std::endl;
    // printf("reading_map2\n");
    origin_(0) = center_map[0] - res * dim[0] / 2.0;
    origin_(1) = center_map[1] - res * dim[1] / 2.0;
    origin_(2) = center_map[2] - res * dim2_down;

    /*        double or2 = origin_(2);*/

    // printf("*******origin_ before is\n");
    // std::cout << origin_ << std::endl;

    //  printf("*******z_ground before is\n");
    //  std::cout << z_ground << std::endl;

    /*        printf("*******FIRST ? before is\n");
            std::cout << (int)((center_map[2] - z_ground) / res + dim[2] / 2.0) << std::endl;*/

    // origin_(2) = (or2 < z_ground) ? z_ground : origin_(2);
    // dim[2] = (or2 < z_ground) ? (int)((center_map[2] - z_ground) / res + dim[2] / 2.0) : dim[2];
    // printf("reading_map3\n");
    // printf("reading_map2\n");
    for (unsigned int i = 0; i < 3; i++)
    {
      dim_(i) = dim[i];
    }
    // printf("reading_map4\n");

    // printf("dim2down=%d\n", dim2_down);
    // printf("dim2_up=%d\n", dim2_up);

    // printf("*******Dim_ is\n");
    // std::cout << dim_ << std::endl;
    // printf("reading_map3\n");
    resolution_ = res;
    data_.resize(dim[0] * dim[1] * dim[2], 0);
    int total_size = dim[0] * dim[1] * dim[2];
    // printf("In reader3, size=%f, %f, %f\n", dim[0], dim[1], dim[2]);
    // printf("reading_map4\n");
    for (size_t i = 0; i < pclptr->points.size(); ++i)
    {
      // Let's find the cell coordinates of the point expresed in a system of coordinates that has as origin the (minX,
      // minY, minZ) point of the map
      int x = std::round((pclptr->points[i].x - origin_(0)) / res - 0.5);
      int y = std::round((pclptr->points[i].y - origin_(1)) / res - 0.5);
      int z = std::round((pclptr->points[i].z - origin_(2)) / res - 0.5);

      // Force them to be positive:
      x = (x > 0) ? x : 0;
      y = (y > 0) ? y : 0;
      z = (z > 0) ? z : 0;
      // this next formula works only when x, y, z are in cell coordinates (relative to the origin of the map)
      int id = x + dim_(0) * y + dim_(0) * dim_(1) * z;

      if (id < 0)
      {
        printf("JPS Reader: There is sth wrong, id= %d\n", id);
        /*        std::cout << "Center_map\n" << center_map << std::endl;
                std::cout << "Origin\n" << origin_(0) << ", " << origin_(1) << ", " << origin_(2) << std::endl;
                std::cout << "dim" << dim << std::endl;
                std::cout << "id=" << id << std::endl;
                std::cout << "XYZCells=" << x << ", " << y << ", " << z << std::endl;
                std::cout << "XYZ=" << pclptr->points[i].x << ", " << pclptr->points[i].y << ", " << pclptr->points[i].z
                          << std::endl;*/
      }
      if (id >= 0 && id < total_size)
      {
        data_[id] = 100;
      }

      // now let's inflate the voxels around that point
      int m = (int)floor((inflation / res));
      // m is the amount of cells to inflate in each direction

      for (int ix = x - m; ix <= x + m; ix++)
      {
        for (int iy = y - m; iy <= y + m; iy++)
        {
          for (int iz = z - m; iz <= z + m; iz++)
          {
            int id_infl = ix + dim_(0) * iy + dim_(0) * dim_(1) * iz;
            if (id_infl >= 0 && id_infl < total_size)  // Ensure we are inside the map
            {
              data_[id_infl] = 100;
            }
          }
        }
      }
    }
    // printf("finished reading map\n");
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
