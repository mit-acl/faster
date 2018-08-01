#include <motion_primitive_library/planner/state_space.h>

using namespace MPL;

template <int Dim>
void StateSpace<Dim>::getSubStateSpace(int time_step) {
  if(best_child_.empty())
    return;

  StatePtr<Dim> currNode_ptr = best_child_[time_step];
  currNode_ptr->pred_action_cost.clear();
  currNode_ptr->pred_action_id.clear();
  currNode_ptr->pred_hashkey.clear();
  currNode_ptr->t = 0;

  for(auto& it: hm_) {
    it.second->g = std::numeric_limits<decimal_t>::infinity();
    it.second->rhs = std::numeric_limits<decimal_t>::infinity();
    it.second->pred_action_cost.clear();
    it.second->pred_action_id.clear();
    it.second->pred_hashkey.clear();
    it.second->t = 0;
  }

  currNode_ptr->g = 0;
  currNode_ptr->rhs = 0;

  hashMap<Dim> new_hm;
  priorityQueue<State<Dim>> epq;
  currNode_ptr->heapkey = epq.push(std::make_pair(currNode_ptr->rhs, currNode_ptr));
  new_hm[currNode_ptr->hashkey] = currNode_ptr;

  while(!epq.empty()) {
    currNode_ptr = epq.top().second; epq.pop();

    if(currNode_ptr->t == max_t_) {
      currNode_ptr->iterationclosed = false;
      currNode_ptr->g = std::numeric_limits<decimal_t>::infinity();
      currNode_ptr->succ_coord.clear();
      currNode_ptr->succ_hashkey.clear();
      currNode_ptr->succ_action_cost.clear();
      currNode_ptr->succ_action_id.clear();
    }

    for(unsigned int i = 0; i < currNode_ptr->succ_hashkey.size(); i++) {
      Key succ_key = currNode_ptr->succ_hashkey[i];

      StatePtr<Dim>& succNode_ptr = new_hm[succ_key];
      if(!succNode_ptr) 
        succNode_ptr = hm_[succ_key];

      int id = -1;
      for(unsigned int i = 0; i < succNode_ptr->pred_hashkey.size(); i++) {
        if(succNode_ptr->pred_hashkey[i] == currNode_ptr->hashkey) {
          id = i;
          break;
        }
      }
      if(id == -1) {
        succNode_ptr->pred_hashkey.push_back(currNode_ptr->hashkey);
        succNode_ptr->pred_action_cost.push_back(currNode_ptr->succ_action_cost[i]);
        succNode_ptr->pred_action_id.push_back(currNode_ptr->succ_action_id[i]);
      }

      decimal_t tentative_rhs = currNode_ptr->rhs + currNode_ptr->succ_action_cost[i];

      if(tentative_rhs < succNode_ptr->rhs) {
        succNode_ptr->t = currNode_ptr->t + dt_;
        succNode_ptr->rhs = tentative_rhs;
        if(succNode_ptr->iterationclosed) {
          succNode_ptr->g = succNode_ptr->rhs; // set g == rhs
          succNode_ptr->heapkey = epq.push( std::make_pair(succNode_ptr->rhs, succNode_ptr) );
        }

      }
    }

  }

  hm_ = new_hm;
  pq_.clear();
  for(auto& it: hm_) {
    if(it.second->iterationopened && !it.second->iterationclosed) 
      it.second->heapkey = pq_.push( std::make_pair(calculateKey(it.second), it.second) );
  }

  //checkValidation(hm_);
}


template <int Dim>
vec_E<Primitive<Dim>> StateSpace<Dim>::increaseCost(std::vector<std::pair<Key, int> > states, const std::shared_ptr<env_base<Dim>>& ENV) {
  vec_E<Primitive<Dim>> prs;
  for(const auto& affected_node: states) {
    //update edge
    StatePtr<Dim>& succNode_ptr = hm_[affected_node.first];
    const int i = affected_node.second; // i-th pred
    if(!std::isinf(succNode_ptr->pred_action_cost[i])) {
      succNode_ptr->pred_action_cost[i] = std::numeric_limits<decimal_t>::infinity();
      updateNode(succNode_ptr);

      Key parent_key = succNode_ptr->pred_hashkey[i];
      Primitive<Dim> pr;
      ENV->forward_action( hm_[parent_key]->coord, succNode_ptr->pred_action_id[i], pr);
      prs.push_back(pr);

      int succ_act_id = hm_[affected_node.first]->pred_action_id[i];
      for(unsigned int j = 0; j < hm_[parent_key]->succ_action_id.size(); j++) {
        if(succ_act_id == hm_[parent_key]->succ_action_id[j]) {
          hm_[parent_key]->succ_action_cost[j] = std::numeric_limits<decimal_t>::infinity();
          break;
        }
      }
    }
  }

  return prs;
}

template <int Dim>
vec_E<Primitive<Dim>> StateSpace<Dim>::decreaseCost(std::vector<std::pair<Key, int> > states, const std::shared_ptr<env_base<Dim>>& ENV) {
  vec_E<Primitive<Dim>> prs;
  for(const auto& affected_node: states) {
    StatePtr<Dim>& succNode_ptr = hm_[affected_node.first];
    const int i = affected_node.second;
    if(std::isinf(succNode_ptr->pred_action_cost[i])) {
      Key parent_key = succNode_ptr->pred_hashkey[i];
      Primitive<Dim> pr;
      ENV->forward_action( hm_[parent_key]->coord, succNode_ptr->pred_action_id[i], pr );
      if(ENV->is_free(pr)) {
        prs.push_back(pr);
        succNode_ptr->pred_action_cost[i] = pr.J(ENV->wi_) + ENV->w_*dt_;
        updateNode(succNode_ptr);
        int succ_act_id = succNode_ptr->pred_action_id[i];
        for(unsigned int j = 0; j < hm_[parent_key]->succ_action_id.size(); j++) {
          if(succ_act_id == hm_[parent_key]->succ_action_id[j]) {
            hm_[parent_key]->succ_action_cost[j] = succNode_ptr->pred_action_cost[i];
            break;
          }
        }

      }
    }
  }

  return prs;
}

template <int Dim>
void StateSpace<Dim>::updateNode(StatePtr<Dim>& currNode_ptr) {
  // if currNode is not start, update its rhs
  // start rhs is assumed to be 0
  if(currNode_ptr->rhs != 0) {
    currNode_ptr->rhs = std::numeric_limits<decimal_t>::infinity();
    for(unsigned int i = 0; i < currNode_ptr->pred_hashkey.size(); i++) {
      Key pred_key = currNode_ptr->pred_hashkey[i];
      if(currNode_ptr->rhs > hm_[pred_key]->g + currNode_ptr->pred_action_cost[i]) {
        currNode_ptr->rhs = hm_[pred_key]->g + currNode_ptr->pred_action_cost[i];
        currNode_ptr->t = hm_[pred_key]->t + dt_;
      }
    }
  }

  // if currNode is in openset, remove it
  if(currNode_ptr->iterationopened && !currNode_ptr->iterationclosed ) {
    pq_.erase(currNode_ptr->heapkey);
    currNode_ptr->iterationclosed = true;
  }

  // if currNode's g value is not equal to its rhs, put it into openset
  // if(currNode_ptr->g != currNode_ptr->rhs || !currNode_ptr->iterationopened) {
  if(currNode_ptr->g != currNode_ptr->rhs) {
    decimal_t fval = calculateKey(currNode_ptr);
    currNode_ptr->heapkey = pq_.push( std::make_pair(fval, currNode_ptr));
    currNode_ptr->iterationopened = true;
    currNode_ptr->iterationclosed = false;
  }
}

template <int Dim>
bool StateSpace<Dim>::isBlocked() {
  for(const auto& ptr: best_child_) {
    if(ptr->g != ptr->rhs)
      return true;
  }
  return false;
}


template <int Dim>
decimal_t StateSpace<Dim>::calculateKey(const StatePtr<Dim>& node) {
  return std::min(node->g, node->rhs) + eps_ * node->h;
}


template <int Dim>
void StateSpace<Dim>::checkValidation(const hashMap<Dim>& hm) {
  //****** Check if there is null element in succ graph
  for(const auto& it: hm) {
    if(!it.second) 
      std::cout << "error!!! null element at key: " << it.first << std::endl;
  }

  /*
  for(const auto& it: pq_) {
    if(it.second->t >= 9)
      printf(ANSI_COLOR_RED "error!!!!!!!! t: %f, g: %f, rhs: %f, h: %f\n" ANSI_COLOR_RESET,
          it.second->t, it.second->g, it.second->rhs, it.second->h);
  }
  */

  //****** Check rhs and g value of close set
  printf("Check rhs and g value of closeset\n");
  int close_cnt = 0;
  for(const auto& it: hm) {
    if(it.second->iterationopened && it.second->iterationclosed) {
      printf("g: %f, rhs: %f\n", it.second->g, it.second->rhs);
      close_cnt ++;
    }
  }
 
  // Check rhs and g value of open set
  printf("Check rhs and g value of openset\n");
  int open_cnt = 0;
  for(const auto& it: hm) {
    if(it.second->iterationopened && !it.second->iterationclosed) {
      printf("g: %f, rhs: %f\n", it.second->g, it.second->rhs);
      open_cnt ++;
    }
  }

  // Check rhs and g value of null set
  printf("Check rhs and g value of nullset\n");
  int null_cnt = 0;
  for(const auto& it: hm) {
    if(!it.second->iterationopened) {
      printf("g: %f, rhs: %f\n", it.second->g, it.second->rhs);
      null_cnt ++;
    }
  }

  printf("hm: [%zu], open: [%d], closed: [%d], null: [%d]\n", 
      hm.size(), open_cnt, close_cnt, null_cnt);
}

namespace MPL
{
template struct StateSpace<2>;
template struct StateSpace<3>;
}
