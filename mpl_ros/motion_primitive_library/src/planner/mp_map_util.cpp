#include <motion_primitive_library/planner/mp_map_util.h>

using namespace MPL;

template <int Dim>
MPMapUtil<Dim>::MPMapUtil(bool verbose) {
  this->planner_verbose_ = verbose;
  if(this->planner_verbose_)
    printf(ANSI_COLOR_CYAN "[MPPlanner] PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
}

template <int Dim>
void MPMapUtil<Dim>::setMapUtil(std::shared_ptr<MapUtil<Dim>>& map_util) {
  this->ENV_.reset(new MPL::env_map<Dim>(map_util));
  map_util_ = map_util;
}

template <int Dim>
vec_Vecf<Dim> MPMapUtil<Dim>::getLinkedNodes() const {
  lhm_.clear();
  vec_Vecf<Dim> linked_pts;
  for(const auto& it: this->ss_ptr_->hm_) {
    if(!it.second)
      continue;
    //check pred array
    for(unsigned int i = 0; i < it.second->pred_hashkey.size(); i++) {
      Key key = it.second->pred_hashkey[i];
      Primitive<Dim> pr;
      this->ENV_->forward_action( this->ss_ptr_->hm_[key]->coord, it.second->pred_action_id[i], pr );
      decimal_t max_v = 0;
      if(Dim == 2)
        max_v = std::max(pr.max_vel(0), pr.max_vel(1));
      else if(Dim == 3)
        max_v = std::max(std::max(pr.max_vel(0), pr.max_vel(1)), pr.max_vel(2));
      int n = 1.0 * std::ceil(max_v * pr.t() / map_util_->getRes());
      int prev_id = -1;
      vec_E<Waypoint<Dim>> ws = pr.sample(n);
      for(const auto& w: ws) {
        int id = map_util_->getIndex(map_util_->floatToInt(w.pos));
        if(id != prev_id) {
          linked_pts.push_back(map_util_->intToFloat(map_util_->floatToInt(w.pos)));
          lhm_[id].push_back(std::make_pair(it.second->hashkey, i));
          prev_id = id;
        }
      }
    }
  }

  return linked_pts;
}


template <int Dim>
vec_E<Primitive<Dim>> MPMapUtil<Dim>::updateBlockedNodes(const vec_Veci<Dim>& blocked_pns) {
  std::vector<std::pair<Key, int>> blocked_nodes;
  for(const auto& it: blocked_pns) {
    int id = map_util_->getIndex(it);
    auto search = lhm_.find(id);
    if(search != lhm_.end()) {
      for(const auto& node: lhm_[id]) 
        blocked_nodes.push_back(node);
    }
  }

  return this->ss_ptr_->increaseCost(blocked_nodes, this->ENV_);
}


template <int Dim>
vec_E<Primitive<Dim>> MPMapUtil<Dim>::updateClearedNodes(const vec_Veci<Dim>& cleared_pns) {
  std::vector<std::pair<Key, int>> cleared_nodes;
  for(const auto& it: cleared_pns) {
    int id = map_util_->getIndex(it);
    auto search = lhm_.find(id);
    if(search != lhm_.end()) {
      for(const auto& node: lhm_[id]) 
        cleared_nodes.push_back(node);
    }
  }
  
  return this->ss_ptr_->decreaseCost(cleared_nodes, this->ENV_);
}

template class MPMapUtil<2>;

template class MPMapUtil<3>;
