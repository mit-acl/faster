#include <jps_planner/distance_map_planner/distance_map_planner.h>

template <int Dim>
DMPlanner<Dim>::DMPlanner(bool verbose): planner_verbose_(verbose) {
  planner_verbose_ = verbose;
  if(planner_verbose_)
    printf(ANSI_COLOR_CYAN "DMP PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
}

template <int Dim>
void DMPlanner<Dim>::setMapUtil(const std::shared_ptr<JPS::MapUtil<Dim>> &map_util) {
  map_util_ = map_util;
}

template <int Dim>
void DMPlanner<Dim>::setSearchRadius(const Vecf<Dim>& r) {
  search_radius_ = r;
}

template <int Dim>
void DMPlanner<Dim>::setPotentialRadius(const Vecf<Dim>& r) {
  potential_radius_ = r;
}

template <int Dim>
void DMPlanner<Dim>::setPotentialMapRange(const Vecf<Dim>& r) {
  potential_map_range_ = r;
}

template <int Dim>
void DMPlanner<Dim>::setEps(double eps) {
  eps_ = eps;
}

template <int Dim>
void DMPlanner<Dim>::setCweight(double c) {
  cweight_ = c;
}

template <int Dim>
void DMPlanner<Dim>::setPow(int pow) {
  pow_ = pow;
}

template <int Dim>
int DMPlanner<Dim>::status() { return status_; }

template <int Dim>
vec_Vecf<Dim> DMPlanner<Dim>::getPath() { return path_; }

template <int Dim>
vec_Vecf<Dim> DMPlanner<Dim>::getRawPath() { return raw_path_; }

template <int Dim>
vec_Vecf<Dim> DMPlanner<Dim>::removeCornerPts(const vec_Vecf<Dim> &path) {
  if (path.size() < 3)
    return path;

  int8_t val = 1;
  // cut zigzag segment
  vec_Vecf<Dim> optimized_path;
  Vecf<Dim> pose1 = path[0];
  Vecf<Dim> pose2 = path[1];
  Vecf<Dim> prev_pose = pose1;
  optimized_path.push_back(pose1);
  decimal_t cost1, cost2, cost3;

  if (!map_util_->isBlocked(pose1, pose2, val))
    cost1 = (pose1 - pose2).norm();
  else
    cost1 = std::numeric_limits<decimal_t>::infinity();

  for (unsigned int i = 1; i < path.size() - 1; i++) {
    pose1 = path[i];
    pose2 = path[i + 1];
    if (!map_util_->isBlocked(pose1, pose2, val))
      cost2 = (pose1 - pose2).norm();
    else
      cost2 = std::numeric_limits<decimal_t>::infinity();

    if (!map_util_->isBlocked(prev_pose, pose2, val))
      cost3 = (prev_pose - pose2).norm();
    else
      cost3 = std::numeric_limits<decimal_t>::infinity();

    if (cost3 < cost1 + cost2) {
      cost1 = cost3;
      //printf("skip point [%d]: [%f, %f], cost1: %f, cost2: %f, cost3: %f\n", i, path[i](0), path[i](1), cost1, cost2, cost3);
    }
    else {
      optimized_path.push_back(path[i]);
      cost1 = (pose1 - pose2).norm();
      prev_pose = pose1;
    }
  }

  optimized_path.push_back(path.back());
  return optimized_path;
}

template <int Dim>
vec_Vecf<Dim> DMPlanner<Dim>::removeLinePts(const vec_Vecf<Dim> &path) {
  if (path.size() < 3)
    return path;

  vec_Vecf<Dim> new_path;
  new_path.push_back(path.front());
  for (unsigned int i = 1; i < path.size() - 1; i++) {
    Vecf<Dim> p = (path[i + 1] - path[i]) - (path[i] - path[i - 1]);
    if(Dim == 3) {
      if (fabs(p(0)) + fabs(p(1)) + fabs(p(2)) > 1e-2)
        new_path.push_back(path[i]);
    }
    else {
      if (fabs(p(0)) + fabs(p(1)) > 1e-2)
        new_path.push_back(path[i]);
    }
  }
  new_path.push_back(path.back());
  return new_path;
}


template <int Dim>
vec_Vecf<Dim> DMPlanner<Dim>::getOpenSet() const {
  vec_Vecf<Dim> ps;
  const auto ss = graph_search_->getOpenSet();
  for(const auto& it: ss) {
    if(Dim == 3) {
      Veci<Dim> pn;
      pn << it->x, it->y, it->z;
      ps.push_back(map_util_->intToFloat(pn));
    }
    else
      ps.push_back(map_util_->intToFloat(Veci<Dim>(it->x, it->y)));
  }
  return ps;
}

template <int Dim>
vec_Vecf<Dim> DMPlanner<Dim>::getCloseSet() const {
  vec_Vecf<Dim> ps;
  const auto ss = graph_search_->getCloseSet();
  for(const auto& it: ss) {
    if(Dim == 3) {
      Veci<Dim> pn;
      pn << it->x, it->y, it->z;
      ps.push_back(map_util_->intToFloat(pn));
    }
    else
      ps.push_back(map_util_->intToFloat(Veci<Dim>(it->x, it->y)));
  }
  return ps;
}

template <int Dim>
vec_Vecf<Dim> DMPlanner<Dim>::getAllSet() const {
  vec_Vecf<Dim> ps;
  const auto ss = graph_search_->getAllSet();
  for(const auto& it: ss) {
    if(Dim == 3) {
      Veci<Dim> pn;
      pn << it->x, it->y, it->z;
      ps.push_back(map_util_->intToFloat(pn));
    }
    else
      ps.push_back(map_util_->intToFloat(Veci<Dim>(it->x, it->y)));
  }
  return ps;
}

template <int Dim>
void DMPlanner<Dim>::updateMap() {
  cmap_ = map_util_->getMap();
}


template <int Dim>
std::vector<bool> DMPlanner<Dim>::setPath(const vec_Vecf<Dim>& path,
                                          const Vecf<Dim>& radius, bool dense) {
  // create cells along path
  vec_Veci<Dim> ps;
  if(!dense) {
    for(unsigned int i = 1; i < path.size(); i++) {
      auto pns = map_util_->rayTrace(path[i-1], path[i]);
      //ps.push_back(map_util_->floatToInt(path[i-1]));
      ps.insert(ps.end(), pns.begin(), pns.end());
      ps.push_back(map_util_->floatToInt(path[i]));
    }
  }
  else {
    for(const auto& pt: path)
      ps.push_back(map_util_->floatToInt(pt));
  }

  // create mask
  vec_Veci<Dim> ns;
  int rn = std::ceil(radius(0) / map_util_->getRes());
  if(Dim == 2) {
    for(int nx = -rn; nx <= rn; nx++) {
      for(int ny = -rn; ny <= rn; ny++) {
        if(std::hypot(nx, ny) > rn)
          continue;
        ns.push_back(Veci<Dim>(nx, ny));
      }
    }
  }
  else {
    int hn = std::ceil(radius(2) / map_util_->getRes());
    for(int nx = -rn; nx <= rn; nx++) {
      for(int ny = -rn; ny <= rn; ny++) {
        for(int nz = -hn; nz <= hn; nz++) {
          if(std::hypot(nx, ny) > rn)
            continue;
          Veci<Dim> n;
          n << nx, ny, nz;
          ns.push_back(n);
        }
      }
    }
  }

  // create in_region map
  auto dim = map_util_->getDim();
  std::vector<bool> in_region;
  if(Dim == 2)
    in_region.resize(dim(0) * dim(1), false);
  else
    in_region.resize(dim(0) * dim(1) * dim(2), false);


  for(const auto& it: ps) {
    for(const auto& n: ns) {
      Veci<Dim> pn = it + n;
      if(map_util_->isOutside(pn))
        continue;
      int idx = Dim == 2? pn(0) + dim(0) * pn(1) :
        pn(0) + dim(0) * pn(1) + dim(0) * dim(1) * pn(2);
      if(!in_region[idx]) {
        in_region[idx] = true;
      }
    }

  }

  return in_region;
}


template <int Dim>
vec_Vecf<Dim> DMPlanner<Dim>::getSearchRegion() {
  auto dim = map_util_->getDim();
  vec_Vecf<Dim> pts;

  if(Dim == 2) {
    for(int nx = 0; nx < dim(0); nx ++) {
      for(int ny = 0; ny < dim(1); ny ++) {
        int idx = nx + dim(0) * ny;
        if(search_region_[idx])
          pts.push_back(map_util_->intToFloat(Veci<Dim>(nx, ny)));
      }
    }
  }
  else {
    for(int nx = 0; nx < dim(0); nx ++) {
      for(int ny = 0; ny < dim(1); ny ++) {
        for(int nz = 0; nz < dim(2); nz ++) {
          int idx = nx + dim(0) * ny + dim(0) * dim(1) * nz;
          if(search_region_[idx]) {
            Veci<Dim> n;
            n << nx, ny, nz;
            pts.push_back(map_util_->intToFloat(n));
          }
        }
      }
    }

  }

  return pts;
}


template <int Dim>
bool DMPlanner<Dim>::checkAvailability(const Veci<Dim>& pn) {
  if (map_util_->isUnknown(pn)) {
    if(planner_verbose_)
      printf(ANSI_COLOR_RED "point is unknown!\n" ANSI_COLOR_RESET);
    return false;
  }
  if (map_util_->isOutside(pn)) {
    if(planner_verbose_) {
      printf(ANSI_COLOR_RED "point is outside!\n" ANSI_COLOR_RESET);
      std::cout << "coordinate: " << pn.transpose() << std::endl;
      std::cout <<"Map origin: " << map_util_->getOrigin().transpose() << std::endl;
      std::cout <<"Map dim: " << map_util_->getDim().transpose() << std::endl;
    }
    return false;
  }
  if (cmap_[map_util_->getIndex(pn)] == 100) {
    if(planner_verbose_)
      printf(ANSI_COLOR_RED "point is occupied!\n" ANSI_COLOR_RESET);
    return false;
  }
  return true;
}

template <int Dim>
vec_Vec3f DMPlanner<Dim>::getCloud(double h_max) {
  auto data = map_util_->getMap();
  auto dim = map_util_->getDim();
  vec_Vec3f ps;

  Veci<Dim> n;
  if(Dim == 2) {
    for(n(0) = 0; n(0) < dim(0); n(0)++) {
      for(n(1) = 0; n(1) < dim(1); n(1)++) {
        int idx = map_util_->getIndex(n);
        if(data[idx] > 0) {
          double h = (double) data[idx] * h_max / H_MAX;
          Vecf<Dim> pt2d = map_util_->intToFloat(n);
          ps.push_back(Vec3f(pt2d(0), pt2d(1), h));
        }
      }
    }
  }
  else {
    for(n(0) = 0; n(0) < dim(0); n(0)++) {
      for(n(1) = 0; n(1) < dim(1); n(1)++) {
        for(n(2) = 0; n(2) < dim(2); n(2)++) {
          int idx = map_util_->getIndex(n);
          if(data[idx] > 0)  {
            auto pt = map_util_->intToFloat(n);
            Vec3f p;
            p << pt(0), pt(1), pt(2);
            ps.push_back(p);
          }
        }
      }
    }
  }

  return ps;
}


template <int Dim>
void DMPlanner<Dim>::createMask(int pow) {
  mask_.clear();
  // create mask
  double res = map_util_->getRes();
  double h_max = H_MAX;
  int rn = std::ceil(potential_radius_(0) / res);
  //printf("rn: %d\n", rn);
  //printf("hn: %d\n", hn);
  Veci<Dim> n;
  if(Dim == 2) {
    for(n(0) = -rn; n(0) <= rn; n(0)++) {
      for(n(1) = -rn; n(1) <= rn; n(1)++) {
        if(std::hypot(n(0), n(1)) > rn)
          continue;
        double h = h_max * std::pow((1 - (double) std::hypot(n(0), n(1)) / rn), pow);
        if(h > 1e-3)
          mask_.push_back(std::make_pair(n, (int8_t) h));
      }
    }
  }
  else {
    int hn = std::ceil(potential_radius_(2) / res);
    for(n(0) = -rn; n(0) <= rn; n(0)++) {
      for(n(1) = -rn; n(1) <= rn; n(1)++) {
        for(n(2) = -hn; n(2) <= hn; n(2)++) {
          if(std::hypot(n(0), n(1)) > rn)
            continue;
          double h = h_max * std::pow((1 - (double) std::hypot(n(0), n(1)) / rn) * (1 - (double) std::abs(n(2)) / hn), pow);
          if(h > 1e-3)
            mask_.push_back(std::make_pair(n, (int8_t) h));
        }
      }
    }
  }
}


template <int Dim>
void DMPlanner<Dim>::updateDistanceMap(const Vecf<Dim>& pos, const Vecf<Dim>& range) {
  // compute a 2D local distance map
  const auto dim = map_util_->getDim();
  Veci<Dim> coord1 = Veci<Dim>::Zero();
  Veci<Dim> coord2 = dim;
  if(range.norm() > 0) {
    coord1 = map_util_->floatToInt(pos - range);
    coord2 = map_util_->floatToInt(pos + range);
    for(int i = 0; i < Dim; i++) {
      if(coord1(i) < 0)
        coord1(i) = 0;
      else if(coord1(i) >= dim(i))
        coord1(i) = dim(i) - 1;

      if(coord2(i) < 0)
        coord2(i) = 0;
      else if(coord2(i) >= dim(i))
        coord2(i) = dim(i) - 1;
    }
  }

  std::vector<int8_t> map = map_util_->getMap();
  auto distance_map = map;

  Veci<Dim> n;
  if(Dim == 2) {
    for(n(0) = coord1(0); n(0) < coord2(0); n(0)++) {
      for(n(1) = coord1(1); n(1) < coord2(1); n(1)++) {
        int idx = map_util_->getIndex(n);
        if(map[idx] > 0) {
          distance_map[idx] = H_MAX;
          for(const auto& it: mask_) {
            const Veci<Dim> new_n = n + it.first;

            if(!map_util_->isOutside(new_n)) {
              const int new_idx = map_util_->getIndex(new_n);
              distance_map[new_idx] = std::max(distance_map[new_idx], it.second);
            }
          }
        }
      }
    }
  }
  else {
    for(n(0) = coord1(0); n(0) < coord2(0); n(0)++) {
      for(n(1) = coord1(1); n(1) < coord2(1); n(1)++) {
        for(n(2) = coord1(2); n(2) < coord2(2); n(2)++) {
          int idx = map_util_->getIndex(n);
          if(map[idx] > 0) {
            distance_map[idx] = H_MAX;
            for(const auto& it: mask_) {
              const Veci<Dim> new_n = n + it.first;

              if(!map_util_->isOutside(new_n)) {
                const int new_idx = map_util_->getIndex(new_n);
                distance_map[new_idx] = std::max(distance_map[new_idx], it.second);
              }
            }
          }
        }
      }
    }
  }

  map_util_->setMap(map_util_->getOrigin(), dim, distance_map, map_util_->getRes());
}

template <int Dim>
bool DMPlanner<Dim>::plan(const Vecf<Dim> &start, const Vecf<Dim> &goal, decimal_t eps, decimal_t cweight) {
  if(planner_verbose_){
    std::cout <<"Start: " << start.transpose() << std::endl;
    std::cout <<"Goal:  " << goal.transpose() << std::endl;
    std::cout <<"Epsilon:  " << eps << std::endl;
    std::cout <<"cweigth:  " << cweight << std::endl;
  }

  path_.clear();
  raw_path_.clear();
  status_ = 0;

  /// check if the map exists
  if(cmap_.empty()) {
    if(planner_verbose_)
      printf(ANSI_COLOR_RED "need to set cmap, call updateMap()!\n" ANSI_COLOR_RESET);
    status_ = -1;
    return false;
  }


  /// check availability of start
  const Veci<Dim> start_int = map_util_->floatToInt(start);
  if(!checkAvailability(start_int)) {
    if(planner_verbose_)
      printf(ANSI_COLOR_RED "start is invalid!\n" ANSI_COLOR_RESET);
    status_ = 1;
    return false;
  }

  /// check availability of goal
  const Veci<Dim> goal_int = map_util_->floatToInt(goal);
  if(!checkAvailability(goal_int)) {
    if(planner_verbose_)
      printf(ANSI_COLOR_RED "goal is invalid!\n" ANSI_COLOR_RESET);
    status_ = 2;
    return false;
  }

  const Veci<Dim> dim = map_util_->getDim();

  if (Dim == 3) {
    graph_search_ = std::make_shared<DMP::GraphSearch>(
        cmap_.data(), dim(0), dim(1), dim(2), eps, cweight, planner_verbose_);
    graph_search_->plan(start_int(0), start_int(1), start_int(2), goal_int(0),
                        goal_int(1), goal_int(2), search_region_);
  } else {
    graph_search_ = std::make_shared<DMP::GraphSearch>(
        cmap_.data(), dim(0), dim(1), eps, cweight, planner_verbose_);
    graph_search_->plan(start_int(0), start_int(1), goal_int(0), goal_int(1),
                        search_region_);
  }

  const auto path = graph_search_->getPath();
  if (path.size() < 1) {
    if(planner_verbose_)
      std::cout << ANSI_COLOR_RED "Cannot find a path from " << start.transpose() <<" to " << goal.transpose() << " Abort!" ANSI_COLOR_RESET << std::endl;
    status_ = -1;
    return false;
  }

  //**** raw path, s --> g
  vec_Vecf<Dim> ps;
  for (const auto &it : path) {
    if(Dim == 3) {
      Veci<Dim> pn;
      pn << it->x, it->y, it->z;
      ps.push_back(map_util_->intToFloat(pn));
    }
    else
      ps.push_back(map_util_->intToFloat(Veci<Dim>(it->x, it->y)));
  }

  raw_path_ = ps;
  std::reverse(std::begin(raw_path_), std::end(raw_path_));

  // Simplify the raw path
  path_ = removeLinePts(raw_path_);
  path_ = removeCornerPts(path_);
  std::reverse(std::begin(path_), std::end(path_));
  path_ = removeCornerPts(path_);
  std::reverse(std::begin(path_), std::end(path_));

  return true;
}

template <int Dim>
bool DMPlanner<Dim>::computePath(const Vecf<Dim>& start, const Vecf<Dim>& goal, const vec_Vecf<Dim>& path) {
  if(planner_verbose_) {
    printf("****************[DistanceMapPlanner]***************\n");
    printf("eps: %f\n", eps_);
    printf("cweight: %f\n", cweight_);
    printf("pow: %d\n", pow_);
    std::cout << "search_radius: " << search_radius_.transpose() << std::endl;
    std::cout << "potential_radius: " << potential_radius_.transpose() << std::endl;
    std::cout << "potential map range: " << potential_map_range_.transpose() << std::endl;;
    printf("****************[DistanceMapPlanner]***************\n");
  }
  createMask(pow_);

  updateDistanceMap(start, potential_map_range_);

  updateMap();
  search_region_ = setPath(path, search_radius_, false); // sparse input path

  return plan(start, goal, eps_, cweight_);
}


template class DMPlanner<2>;

template class DMPlanner<3>;

