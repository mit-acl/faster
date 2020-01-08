/**
 * @file map_util.h
 * @brief MapUtil classes
 */
#ifndef JPS_MAP_UTIL_H
#define JPS_MAP_UTIL_H

#include <iostream>
#include <jps_basis/data_type.h>
#include "ros/ros.h"
#include <pcl/kdtree/kdtree_flann.h>

namespace JPS
{
/// The type of map data Tmap is defined as a 1D array
using Tmap = std::vector<char>;
/**
 * @biref The map util class for collision checking
 * @param Dim is the dimension of the workspace
 */
template <int Dim>
class MapUtil
{
public:
  /// Simple constructor
  MapUtil()
  {
  }

  void readMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr, int cells_x, int cells_y, int cells_z, double res,
               const Vec3f &center_map, double z_ground, double z_max, double inflation)
  {
    // printf("reading_map\n");
    // **Box of the map --> it's the box with which the map moves.
    // **Center_map --> The center of the box of the map, expressed in global float coordinates
    // **Global float coordinates: (X,Y,Z) of the point relative to the global origin of the world
    // **origin_d_ --> It's the point of the box of the map that minX, minY, minZ (it's a corner of the box of the map).
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
    map_.clear();
    Vec3i dim(cells_x, cells_y, cells_z);
    // printf("dim[0] before is %d\n", dim[0]);

    dim(0) = dim(0) + (int)(5 * (inflation * 1.0) / res);
    dim(1) = dim(1) + (int)(5 * (inflation * 1.0) / res);

    // printf("dim[0] is %d\n", dim[0]);

    int dim2_down = dim(2) / 2.0;
    int dim2_up = dim(2) / 2.0;

    //// printf("reading_map1\n");
    if ((center_map[2] - res * dim(2) / 2.0) < z_ground)
    {
      // std::cout << "center_map(2)=" << center_map(2) << std::endl;
      // std::cout << "z_ground=" << z_ground << std::endl;
      // std::cout << "((center_map(2) - z_ground) / res)=" << (int)((center_map(2) - z_ground) / res) << std::endl;
      // printf("modyfing");
      dim2_down = std::max((int)((center_map(2) - z_ground) / res), 0);  //+1 to avoid problems when taking off
      // dim2_up = (int)(dim[2] / 2.0);
    }

    if (center_map(2) + res * dim(2) / 2.0 > z_max)
    {
      // printf("modyfing");
      // dim2_down = (int)((center_map[2] - z_ground) / res) + 1;  //+1 to avoid problems when taking off
      dim2_up = (int)((z_max - center_map(2)) / res);  //+1 to avoid problems when taking off
      dim2_up = (dim2_up > 0) ? dim2_up : 1;           // Force it to be >= 1
      // dim[2] = dim2_down + dim2_up;
    }
    // printf("z_max is %f\n", z_max);
    // printf("Dim_down is %d\n", dim2_down);
    // printf("Dim_up is %d\n", dim2_up);
    dim(2) = dim2_down + dim2_up;
    /*    printf("*******Dim is\n");
        std::cout << dim.transpose() << std::endl;*/
    /*
        printf("*******Center antes is\n");
        std::cout << center_map.transpose() << std::endl;*/
    // printf("reading_map2\n");
    origin_d_(0) = center_map(0) - res * dim(0) / 2.0;
    origin_d_(1) = center_map(1) - res * dim(1) / 2.0;
    origin_d_(2) = center_map(2) - res * dim2_down;

    // printf("*******Corner despues is\n");
    // std::cout << origin_d_.transpose() << std::endl;

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
      dim_(i) = dim(i);
    }
    // printf("reading_map4\n");

    // printf("dim2down=%d\n", dim2_down);
    // printf("dim2_up=%d\n", dim2_up);

    // printf("*******Dim_ is\n");
    // std::cout << dim_ << std::endl;
    // printf("reading_map3\n");
    res_ = res;
    map_.resize(dim(0) * dim(1) * dim(2), 0);
    int total_size = dim(0) * dim(1) * dim(2);
    // printf("In reader3, size=%f, %f, %f\n", dim[0], dim[1], dim[2]);
    // printf("reading_map4\n");
    for (size_t i = 0; i < pclptr->points.size(); ++i)
    {
      // Let's find the cell coordinates of the point expresed in a system of coordinates that has as origin the (minX,
      // minY, minZ) point of the map
      int x = std::round((pclptr->points[i].x - origin_d_(0)) / res - 0.5);
      int y = std::round((pclptr->points[i].y - origin_d_(1)) / res - 0.5);
      int z = std::round((pclptr->points[i].z - origin_d_(2)) / res - 0.5);

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
        map_[id] = 100;
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
              map_[id_infl] = 100;
            }
          }
        }
      }
    }
    // printf("finished reading map\n");
  }

  /// Get map data
  Tmap getMap()
  {
    return map_;
  }
  /// Get resolution
  decimal_t getRes()
  {
    return res_;
  }
  /// Get dimensions
  Veci<Dim> getDim()
  {
    return dim_;
  }
  /// Get origin
  Vecf<Dim> getOrigin()
  {
    return origin_d_;
  }
  /// Get index of a cell
  int getIndex(const Veci<Dim> &pn)
  {
    return Dim == 2 ? pn(0) + dim_(0) * pn(1) : pn(0) + dim_(0) * pn(1) + dim_(0) * dim_(1) * pn(2);
  }

  /// Check if the given cell is outside of the map in i-the dimension
  bool isOutsideXYZ(const Veci<Dim> &n, int i)
  {
    return n(i) < 0 || n(i) >= dim_(i);
  }
  /// Check if the cell is free by index
  bool isFree(int idx)
  {
    return map_[idx] == val_free;
  }
  /// Check if the cell is unknown by index
  bool isUnknown(int idx)
  {
    return map_[idx] == val_unknown;
  }
  /// Check if the cell is occupied by index
  bool isOccupied(int idx)
  {
    return map_[idx] > val_free;
  }

  void setOccupied(const Veci<Dim> &pn)
  {
    int total_size = dim_(0) * dim_(1) * dim_(2);
    int index = getIndex(pn);
    if (index >= 0 && index < total_size)
    {  // check that the point is inside the map
      map_[getIndex(pn)] = 100;
    }
  }

  void setFree(const Veci<Dim> &pn)
  {
    int total_size = dim_(0) * dim_(1) * dim_(2);
    int index = getIndex(pn);
    if (index >= 0 && index < total_size)
    {  // check that the point is inside the map
      map_[index] = val_free;
    }
  }

  // set Free all the voxels that are in a 3d cube centered at center and with side/2=d
  void setFreeVoxelAndSurroundings(const Veci<Dim> &center, const float d)
  {
    // std::cout << "Center is" << center.transpose() << std::endl;
    int n_voxels = std::round(d / res_ + 0.5);  // convert distance to number of voxels
    for (int ix = -n_voxels; ix <= n_voxels; ix++)
    {
      for (int iy = -n_voxels; iy <= n_voxels; iy++)
      {
        for (int iz = -n_voxels; iz <= n_voxels; iz++)
        {
          Veci<Dim> voxel = center + Veci<Dim>(ix, iy, iz);  // Int coordinates of the voxel I'm going to clear

          // std::cout << "Clearing" << voxel.transpose() << std::endl;
          setFree(voxel);
        }
      }
    }
  }

  /// Check if the cell is outside by coordinate
  bool isOutside(const Veci<Dim> &pn)
  {
    for (int i = 0; i < Dim; i++)
      if (pn(i) < 0 || pn(i) >= dim_(i))
        return true;
    return false;
  }
  /// Check if the given cell is free by coordinate
  bool isFree(const Veci<Dim> &pn)
  {
    if (isOutside(pn))
      return false;
    else
      return isFree(getIndex(pn));
  }
  /// Check if the given cell is occupied by coordinate
  bool isOccupied(const Veci<Dim> &pn)
  {
    if (isOutside(pn))
      return false;
    else
      return isOccupied(getIndex(pn));
  }
  /// Check if the given cell is unknown by coordinate
  bool isUnknown(const Veci<Dim> &pn)
  {
    if (isOutside(pn))
      return false;
    return map_[getIndex(pn)] == val_unknown;
  }

  /**
   * @brief Set map
   *
   * @param ori origin position
   * @param dim number of cells in each dimension
   * @param map array of cell values
   * @param res map resolution
   */
  void setMap(const Vecf<Dim> &ori, const Veci<Dim> &dim, const Tmap &map, decimal_t res)
  {
    map_ = map;
    dim_ = dim;
    origin_d_ = ori;
    res_ = res;
  }

  /// Print basic information about the util
  void info()
  {
    Vecf<Dim> range = dim_.template cast<decimal_t>() * res_;
    std::cout << "MapUtil Info ========================== " << std::endl;
    std::cout << "   res: [" << res_ << "]" << std::endl;
    std::cout << "   origin: [" << origin_d_.transpose() << "]" << std::endl;
    std::cout << "   range: [" << range.transpose() << "]" << std::endl;
    std::cout << "   dim: [" << dim_.transpose() << "]" << std::endl;
  };

  /// Float position to discrete cell coordinate
  Veci<Dim> floatToInt(const Vecf<Dim> &pt)
  {
    Veci<Dim> pn;
    for (int i = 0; i < Dim; i++)
      pn(i) = std::round((pt(i) - origin_d_(i)) / res_ - 0.5);
    return pn;
  }
  /// Discrete cell coordinate to float position
  Vecf<Dim> intToFloat(const Veci<Dim> &pn)
  {
    // return pn.template cast<decimal_t>() * res_ + origin_d_;
    return (pn.template cast<decimal_t>() + Vecf<Dim>::Constant(0.5)) * res_ + origin_d_;
  }

  /// Raytrace from float point pt1 to pt2
  vec_Veci<Dim> rayTrace(const Vecf<Dim> &pt1, const Vecf<Dim> &pt2)
  {
    Vecf<Dim> diff = pt2 - pt1;
    decimal_t k = 0.8;
    int max_diff = (diff / res_).template lpNorm<Eigen::Infinity>() / k;
    decimal_t s = 1.0 / max_diff;
    Vecf<Dim> step = diff * s;

    vec_Veci<Dim> pns;
    Veci<Dim> prev_pn = Veci<Dim>::Constant(-1);
    for (int n = 1; n < max_diff; n++)
    {
      Vecf<Dim> pt = pt1 + step * n;
      Veci<Dim> new_pn = floatToInt(pt);
      if (isOutside(new_pn))
        break;
      if (new_pn != prev_pn)
        pns.push_back(new_pn);
      prev_pn = new_pn;
    }
    return pns;
  }

  /// Check if the ray from p1 to p2 is occluded
  bool isBlocked(const Vecf<Dim> &p1, const Vecf<Dim> &p2, int8_t val = 100)
  {
    vec_Veci<Dim> pns = rayTrace(p1, p2);
    for (const auto &pn : pns)
    {
      if (map_[getIndex(pn)] >= val)
        return true;
    }
    return false;
  }

  /// Get occupied voxels
  vec_Vecf<Dim> getCloud()
  {
    vec_Vecf<Dim> cloud;
    Veci<Dim> n;
    if (Dim == 3)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          for (n(2) = 0; n(2) < dim_(2); n(2)++)
          {
            if (isOccupied(getIndex(n)))
              cloud.push_back(intToFloat(n));
          }
        }
      }
    }
    else if (Dim == 2)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          if (isOccupied(getIndex(n)))
            cloud.push_back(intToFloat(n));
        }
      }
    }

    return cloud;
  }

  /// Get free voxels
  vec_Vecf<Dim> getFreeCloud()
  {
    vec_Vecf<Dim> cloud;
    Veci<Dim> n;
    if (Dim == 3)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          for (n(2) = 0; n(2) < dim_(2); n(2)++)
          {
            if (isFree(getIndex(n)))
              cloud.push_back(intToFloat(n));
          }
        }
      }
    }
    else if (Dim == 2)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          if (isFree(getIndex(n)))
            cloud.push_back(intToFloat(n));
        }
      }
    }

    return cloud;
  }

  /// Get unknown voxels
  vec_Vecf<Dim> getUnknownCloud()
  {
    vec_Vecf<Dim> cloud;
    Veci<Dim> n;
    if (Dim == 3)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          for (n(2) = 0; n(2) < dim_(2); n(2)++)
          {
            if (isUnknown(getIndex(n)))
              cloud.push_back(intToFloat(n));
          }
        }
      }
    }
    else if (Dim == 2)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          if (isUnknown(getIndex(n)))
            cloud.push_back(intToFloat(n));
        }
      }
    }

    return cloud;
  }

  /// Dilate occupied cells
  void dilate(const vec_Veci<Dim> &dilate_neighbor)
  {
    Tmap map = map_;
    Veci<Dim> n = Veci<Dim>::Zero();
    if (Dim == 3)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          for (n(2) = 0; n(2) < dim_(2); n(2)++)
          {
            if (isOccupied(getIndex(n)))
            {
              for (const auto &it : dilate_neighbor)
              {
                if (!isOutside(n + it))
                  map[getIndex(n + it)] = val_occ;
              }
            }
          }
        }
      }
    }
    else if (Dim == 2)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          if (isOccupied(getIndex(n)))
          {
            for (const auto &it : dilate_neighbor)
            {
              if (!isOutside(n + it))
                map[getIndex(n + it)] = val_occ;
            }
          }
        }
      }
    }

    map_ = map;
  }

  /// Free unknown voxels
  void freeUnknown()
  {
    Veci<Dim> n;
    if (Dim == 3)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          for (n(2) = 0; n(2) < dim_(2); n(2)++)
          {
            if (isUnknown(getIndex(n)))
              map_[getIndex(n)] = val_free;
          }
        }
      }
    }
    else if (Dim == 2)
    {
      for (n(0) = 0; n(0) < dim_(0); n(0)++)
      {
        for (n(1) = 0; n(1) < dim_(1); n(1)++)
        {
          if (isUnknown(getIndex(n)))
            map_[getIndex(n)] = val_free;
        }
      }
    }
  }

  /// Map entity
  Tmap map_;

protected:
  /// Resolution
  decimal_t res_;
  /// Origin, float type
  Vecf<Dim> origin_d_;
  /// Dimension, int type
  Veci<Dim> dim_;
  /// Assume occupied cell has value 100
  int8_t val_occ = 100;
  /// Assume free cell has value 0
  int8_t val_free = 0;
  /// Assume unknown cell has value -1
  int8_t val_unknown = -1;
};

typedef MapUtil<2> OccMapUtil;

typedef MapUtil<3> VoxelMapUtil;
}  // namespace JPS

#endif
