/**
 * @file distance_map_planner.h
 * @brief distance map planner!
 */
#ifndef DMPLANNER_H
#define DMPLANNER_H

#include "graph_search.h"
#include <jps_collision/map_util.h>
#include <jps_basis/data_type.h>

class GraphSearch;
/**
 * @brief Abstract base for planning
 */
template <int Dim> class DMPlanner {
public:
  /**
   * @brief Simple constructor
   * @param verbose enable debug mode
   */
  DMPlanner(bool verbose = false);

  /// Set map util for collistion checking
  void setMapUtil(const std::shared_ptr<JPS::MapUtil<Dim>> &map_util);

  /**
   * @brief set a prior path and get region around it
   * @param path prior path
   * @param r radius in x-y plane
   * @param h height in z axis, if 2d, it's not used
   * @param dense if true, dont need to do rayTrace
   *
   * it returns the inflated region
   */

  std::vector<bool> setPath(const vec_Vecf<Dim> &path, const Vecf<Dim>& radius,
                            bool dense);

  void setSearchRadius(const Vecf<Dim>& r);
  void setPotentialRadius(const Vecf<Dim>& r);
  void setPotentialMapRange(const Vecf<Dim>& r);
  void setEps(double eps);
  void setCweight(double c);
  void setPow(int pow);

  /**
   * @brief Status of the planner
   *
   * 0 --- exit normally;
   * -1 --- no path found;
   * 1, 2 --- start or goal is not free.
   */
  int status();
  /// Get the modified path
  vec_Vecf<Dim> getPath();
  /// Get the raw path
  vec_Vecf<Dim> getRawPath();
  /// Get the nodes in open set
  vec_Vecf<Dim> getOpenSet() const;
  /// Get the nodes in close set
  vec_Vecf<Dim> getCloseSet() const;
  /// Get all the nodes
  vec_Vecf<Dim> getAllSet() const;
  /// Get the potential cloud
  vec_Vec3f getCloud(double h_max = 1);
  /// Get the searching region
  vec_Vecf<Dim> getSearchRegion();

  /// Must be called before run the planning thread
  void updateMap();
  /// Create the mask for potential distance field
  void createMask(int pow);
  /**
   * @brief Generate distance map
   * @param pos center of the distance map
   * @param range the range for local distance map, if range is zero, do global generation
   */
  void updateDistanceMap(const Vecf<Dim>& pos, const Vecf<Dim>& range);


  /// Need to be specified in Child class, main planning function
  bool plan(const Vecf<Dim> &start, const Vecf<Dim> &goal,
            decimal_t eps = 1, decimal_t cweight = 0.1);

  bool computePath(const Vecf<Dim>& start, const Vecf<Dim>& goal, const vec_Vecf<Dim>& path);
protected:
  /// remove redundant points on the same line
  vec_Vecf<Dim> removeLinePts(const vec_Vecf<Dim> &path);
  /// Remove some corner waypoints
  vec_Vecf<Dim> removeCornerPts(const vec_Vecf<Dim> &path);
  /// check availability
  bool checkAvailability(const Veci<Dim> &pn);

  /// Assume using 3D voxel map for all 2d and 3d planning
  std::shared_ptr<JPS::MapUtil<Dim>> map_util_;
  /// The planner back-end
  std::shared_ptr<DMP::GraphSearch> graph_search_;
  /// Mask for generating potential field around obstacle
  vec_E<std::pair<Veci<Dim>, int8_t>> mask_;
  /// tunnel for visualization
  std::vector<bool> search_region_;
  /// 1-D map array
  std::vector<int8_t> cmap_;

  /// Enabled for printing info
  bool planner_verbose_;
  /// Raw path from planner
  vec_Vecf<Dim> raw_path_;
  /// Modified path for future usage
  vec_Vecf<Dim> path_;
  /// Flag indicating the success of planning
  int status_ = 0;
  /// max potential value
  int8_t H_MAX{100};
  /// heuristic weight
  double eps_{0.0};
  /// potential weight
  double cweight_{0.1};
  /// radius of distance field
  Vecf<Dim> potential_radius_{Vecf<Dim>::Zero()};
  /// radius of searching tunnel
  Vecf<Dim> search_radius_{Vecf<Dim>::Zero()};
  /// xy range of local distance map
  Vecf<Dim> potential_map_range_{Vecf<Dim>::Zero()};
  /// power index for creating mask
  int pow_{1};
};

/// Planner for 2D OccMap
typedef DMPlanner<2> DMPlanner2D;

/// Planner for 3D VoxelMap
typedef DMPlanner<3> DMPlanner3D;

#endif
