#include <motion_primitive_library/planner/mp_base_util.h>

using namespace MPL;

template <int Dim>
MPBaseUtil<Dim>::MPBaseUtil() {
  planner_verbose_ = false;
}

template <int Dim>
bool MPBaseUtil<Dim>::initialized() {
  return !(ss_ptr_ == nullptr);
}

template <int Dim>
void MPBaseUtil<Dim>::reset() {
  ss_ptr_ = nullptr;
  traj_ = Trajectory<Dim>();
  ws_.clear();
}

template <int Dim>
void MPBaseUtil<Dim>::setLPAstar(bool use_lpastar) {
  use_lpastar_ =  use_lpastar;
  if(use_lpastar_)
    printf("[MPBaseUtil] use Lifelong Planning A*\n");
  else
    printf("[MPBaseUtil] use normal A*\n");
}

template <int Dim>
void MPBaseUtil<Dim>::setEpsilon(decimal_t eps) {
  epsilon_ = eps;
  if(planner_verbose_)
    printf("[MPBaseUtil] set epsilon: %f\n", epsilon_);
}

template <int Dim>
void MPBaseUtil<Dim>::setMaxNum(int num) {
  max_num_ = num;
  if(planner_verbose_)
    printf("[MPBaseUtil] set max num: %d\n", max_num_);
}

template <int Dim>
void MPBaseUtil<Dim>::setDt(decimal_t dt) {
  ENV_->set_dt(dt);
  if(planner_verbose_)
    printf("[MPBaseUtil] set dt: %f\n", dt);
}

template <int Dim>
void MPBaseUtil<Dim>::setW(decimal_t w) {
  ENV_->set_w(w);
  if(planner_verbose_)
    printf("[MPBaseUtil] set w: %f\n", w);
}

template <int Dim>
void MPBaseUtil<Dim>::setAlpha(int alpha) {
  ENV_->set_alpha(alpha);
  if(planner_verbose_)
    printf("[MPBaseUtil] set alpha: %d\n", alpha);
}

template <int Dim>
void MPBaseUtil<Dim>::setU(const vec_Vecf<Dim>& U) {
  ENV_->set_U(U);
}

template <int Dim>
void MPBaseUtil<Dim>::setVmax(decimal_t v_max) {
  ENV_->set_v_max(v_max);
  if(planner_verbose_)
    printf("[MPBaseUtil] set v_max: %f\n", v_max);
}

template <int Dim>
void MPBaseUtil<Dim>::setAmax(decimal_t a_max) {
  ENV_->set_a_max(a_max);
  if(planner_verbose_)
    printf("[MPBaseUtil] set a_max: %f\n", a_max);
}

template <int Dim>
void MPBaseUtil<Dim>::setJmax(decimal_t j_max) {
  ENV_->set_j_max(j_max);
  if(planner_verbose_)
    printf("[MPBaseUtil] set j_max: %f\n", j_max);
}

template <int Dim>
void MPBaseUtil<Dim>::setUmax(decimal_t u_max) {
  ENV_->set_u_max(u_max);
  if(planner_verbose_)
    printf("[MPBaseUtil] set u_max: %f\n", u_max);
}

template <int Dim>
void MPBaseUtil<Dim>::setTmax(decimal_t t) {
  //ENV_->set_t_max(t);
  max_t_ = t;
  if(planner_verbose_)
    printf("[MPBaseUtil] set max time: %f\n", t);
}

template <int Dim>
void MPBaseUtil<Dim>::setPriorTrajectory(const Trajectory<Dim>& traj) {
  ENV_->set_prior_trajectory(traj);
  if(planner_verbose_)
    printf("[MPBaseUtil] set prior trajectory\n");
}

template <int Dim>
void MPBaseUtil<Dim>::setTol(decimal_t tol_dis, decimal_t tol_vel, decimal_t tol_acc) {
  ENV_->set_tol_dis(tol_dis);
  ENV_->set_tol_vel(tol_vel);
  ENV_->set_tol_acc(tol_acc);
  if(planner_verbose_) {
    printf("[MPBaseUtil] set tol_dis: %f\n", tol_dis);
    printf("[MPBaseUtil] set tol_vel: %f\n", tol_vel);
    printf("[MPBaseUtil] set tol_acc: %f\n", tol_acc);
  }
}


template <int Dim>
vec_E<Primitive<Dim>> MPBaseUtil<Dim>::getPrimitivesToGoal() const { 
  vec_E<Primitive<Dim>> prs;
  if(ss_ptr_->best_child_.empty())
    return prs;

  std::unordered_map<Key, bool> added;

  auto currNode_ptr = ss_ptr_->best_child_.back();
  std::queue<StatePtr<Dim>> q;
  q.push(currNode_ptr);
  while( !q.empty()) {
    int size = q.size();
    for(int i = 0; i < size; i++) {
      currNode_ptr = q.front(); q.pop();
      for(unsigned int j = 0; j < currNode_ptr->pred_hashkey.size(); j++) {
        Key pred_key = currNode_ptr->pred_hashkey[j];
        Key key_pair = currNode_ptr->hashkey + pred_key;
        if(added.count(key_pair) == 1 || std::isinf(currNode_ptr->pred_action_cost[j])) // skip the pred if the cost is inf
          continue;
        q.push(ss_ptr_->hm_[pred_key]);
        added[key_pair] = true;
        int action_idx = currNode_ptr->pred_action_id[j];
        Primitive<Dim> pr;
        ENV_->forward_action( ss_ptr_->hm_[pred_key]->coord, action_idx, pr );
        prs.push_back(pr);
      }
    }
  }

  if(planner_verbose_)
    printf("number of states in hm: %zu, number of prs connet to the goal: %zu\n", 
        ss_ptr_->hm_.size(), prs.size());
 
  return prs;
}



template <int Dim>
vec_E<Primitive<Dim>> MPBaseUtil<Dim>::getValidPrimitives() const { 
  vec_E<Primitive<Dim>> prs;
  for(const auto& it: ss_ptr_->hm_) {
   if(it.second && !it.second->pred_hashkey.empty()) {
      for(unsigned int i = 0; i < it.second->pred_hashkey.size(); i++) {
        Key key = it.second->pred_hashkey[i];
        //if(!ss_ptr_->hm_[key] || std::isinf(it.second->pred_action_cost[i])) 
        if(std::isinf(it.second->pred_action_cost[i])) 
          continue;
        Primitive<Dim> pr;
        ENV_->forward_action( ss_ptr_->hm_[key]->coord, it.second->pred_action_id[i], pr );
        prs.push_back(pr);
      }
    }
  }

  if(planner_verbose_)
    printf("number of states in hm: %zu, number of valid prs: %zu\n", 
        ss_ptr_->hm_.size(), prs.size());
 
  return prs;
}

template <int Dim>
vec_E<Primitive<Dim>> MPBaseUtil<Dim>::getAllPrimitives() const { 
  vec_E<Primitive<Dim>> prs;
  for(const auto& it: ss_ptr_->hm_) {
    if(it.second && !it.second->pred_hashkey.empty()) {
      for(unsigned int i = 0; i < it.second->pred_hashkey.size(); i++) {
        Key key = it.second->pred_hashkey[i];
        Primitive<Dim> pr;
        ENV_->forward_action( ss_ptr_->hm_[key]->coord, it.second->pred_action_id[i], pr );
        prs.push_back(pr);
      }
    }
  }

  if(planner_verbose_) 
    printf("number of states in hm: %zu, number of prs: %zu\n", 
        ss_ptr_->hm_.size(), prs.size());

  return prs;
}

template <int Dim>
vec_E<Waypoint<Dim>> MPBaseUtil<Dim>::getWs() const {
  return ws_; 
}

template <int Dim>
Trajectory<Dim> MPBaseUtil<Dim>::getTraj() const {
  return traj_;
}

template <int Dim>
vec_Vecf<Dim> MPBaseUtil<Dim>::getOpenSet() const {
  vec_Vecf<Dim> ps;
  for(const auto& it: ss_ptr_->pq_)
    ps.push_back(it.second->coord.pos);
  return ps;
}

template <int Dim>
vec_Vecf<Dim> MPBaseUtil<Dim>::getCloseSet() const {
  vec_Vecf<Dim> ps;
  for(const auto& it: ss_ptr_->hm_) {
    if(it.second && it.second->iterationclosed)
      ps.push_back(it.second->coord.pos);
  }
  return ps;
}

template <int Dim>
vec_Vecf<Dim> MPBaseUtil<Dim>::getNullSet() const {
  vec_Vecf<Dim> ps;
  for(const auto& it: ss_ptr_->hm_) {
    if(it.second && !it.second->iterationopened)
      ps.push_back(it.second->coord.pos);
  }
  return ps;
}

template <int Dim>
vec_Vecf<Dim> MPBaseUtil<Dim>::getExpandedNodes() const {
  return ENV_->expanded_nodes_;
}

template <int Dim>
int MPBaseUtil<Dim>::getExpandedNum() const {
  return ss_ptr_->expand_iteration_;
}

template <int Dim>
void MPBaseUtil<Dim>::getSubStateSpace(int time_step) {
  ss_ptr_->getSubStateSpace(time_step);
}

template <int Dim>
void MPBaseUtil<Dim>::checkValidation() {
  ss_ptr_->checkValidation(ss_ptr_->hm_);
}

template <int Dim>
bool MPBaseUtil<Dim>::plan(const Waypoint<Dim> &start, const Waypoint<Dim> &goal) {
  if(planner_verbose_) {
    start.print("Start:");
    goal.print("Goal:");
  }

  if(!ENV_->is_free(start.pos)) {
    printf(ANSI_COLOR_RED "[MPPlanner] start is not free!" ANSI_COLOR_RESET "\n");
    return false;
  }

  if(start.use_pos && start.use_vel && start.use_acc && start.use_jrk) {
    ENV_->set_wi(4);
    if(planner_verbose_)
      printf("[MPBaseUtil] set effort in snap\n");
  }
  else if(start.use_pos && start.use_vel && start.use_acc && !start.use_jrk) {
    ENV_->set_wi(3);
    if(planner_verbose_)
      printf("[MPBaseUtil] set effort in jrk\n");
  }
  else if(start.use_pos && start.use_vel && !start.use_acc && !start.use_jrk) {
    ENV_->set_wi(2);
    if(planner_verbose_)
      printf("[MPBaseUtil] set effort in acc\n");
  }
  else if(start.use_pos && !start.use_vel && !start.use_acc && !start.use_jrk) {
    ENV_->set_wi(1);
    if(planner_verbose_)
      printf("[MPBaseUtil] set effort in vel\n");
  }
  else {
    if(planner_verbose_) {
      printf(ANSI_COLOR_RED "[MPBaseUtil] fail to set effort, pos/vel/acc/jrk --> %d/%d/%d/%d\n" ANSI_COLOR_RESET,
          start.use_pos, start.use_vel, start.use_acc, start.use_jrk);
    }
    return false;
  }


  std::unique_ptr<MPL::GraphSearch<Dim>> planner_ptr(new MPL::GraphSearch<Dim>(planner_verbose_));

  // If use A*, reset the state space 
  if(!use_lpastar_) 
    ss_ptr_.reset(new MPL::StateSpace<Dim>(epsilon_));
  else {
    // If use LPA*, reset the state space only at the initial planning
    if(!initialized()) {
      if(planner_verbose_)
        printf(ANSI_COLOR_CYAN "[MPPlanner] reset planner state space!" ANSI_COLOR_RESET "\n");
      ss_ptr_.reset(new MPL::StateSpace<Dim>(epsilon_));
    }
  }

  ENV_->set_goal(goal);

  ENV_->expanded_nodes_.clear();

  ss_ptr_->dt_ = ENV_->get_dt();
  if(use_lpastar_)
    planner_ptr->LPAstar(start, ENV_->state_to_idx(start), ENV_, ss_ptr_, traj_, max_num_, max_t_);
  else
    planner_ptr->Astar(start, ENV_->state_to_idx(start), ENV_, ss_ptr_, traj_, max_num_, max_t_);

  if (traj_.segs.empty()) {
    if(planner_verbose_)
      printf(ANSI_COLOR_RED "[MPPlanner] Cannot find a traj!" ANSI_COLOR_RESET "\n");
    return false;
  }

  ws_.clear();
  ws_.push_back(start);
  decimal_t time = 0;
  std::vector<decimal_t> dts = traj_.getSegsT();
  for (const auto &t : dts) {
    time += t;
    Waypoint<Dim> waypoint;
    traj_.evaluate(time, waypoint);
    waypoint.use_pos = start.use_pos;
    waypoint.use_vel = start.use_vel;
    waypoint.use_acc = start.use_acc;
    waypoint.use_jrk = start.use_jrk;
    ws_.push_back(waypoint);
  }

  return true;
}

template class MPBaseUtil<2>;

template class MPBaseUtil<3>;
