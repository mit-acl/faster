/**
 * @file env_map.h
 * @biref environment for planning in voxel map
 */

#ifndef ENV_MP_H
#define ENV_MP_H
#include <motion_primitive_library/planner/env_base.h>
#include <motion_primitive_library/primitive/primitive.h>
#include <motion_primitive_library/collision_checking/map_util.h>
#include <unordered_map>

namespace MPL {
  /**
   * @brief Voxel map environment
   */
  template <int Dim>
  class env_map : public env_base<Dim>
  {
    ///Lookup table for voxels
    using lookUpTable = std::unordered_map<Key, vec_Vecf<Dim>>;


    public:
    lookUpTable collision_checking_table_;

    ///Collision checking util
    std::shared_ptr<MapUtil<Dim>> map_util_;

    ///Constructor with map util as input
    env_map(std::shared_ptr<MapUtil<Dim>> map_util)
      : map_util_(map_util) {}

    ///Check if state hit the goal region, use L-1 norm
    bool is_goal(const Waypoint<Dim>& state) const {
      bool goaled = (state.pos - this->goal_node_.pos).template lpNorm<Eigen::Infinity>() <= this->tol_dis;
      if(goaled && this->goal_node_.use_vel && this->tol_vel > 0) 
        goaled = (state.vel - this->goal_node_.vel).template lpNorm<Eigen::Infinity>() <= this->tol_vel;
      if(goaled && this->goal_node_.use_acc && this->tol_acc > 0) 
        goaled = (state.acc - this->goal_node_.acc).template lpNorm<Eigen::Infinity>() <= this->tol_acc;
      if(goaled) {
        auto pns = map_util_->rayTrace(state.pos, this->goal_node_.pos);
        for(const auto& it: pns) {
          if(map_util_->isOccupied(it))
            return false;
        }
      }
      return goaled;
    }



    ///Check if a point is in free space
    bool is_free(const Vecf<Dim>& pt) const {
      return map_util_->isFree(map_util_->floatToInt(pt));
    }

    /**
     * @brief Check if the primitive is in free space
     *
     * Sample points along the primitive, and check each point for collision; the number of sampling is calculated based on the maximum velocity and resolution of the map.
     */
    bool is_free(const Primitive<Dim>& pr) const {
      decimal_t max_v = 0;
      if(Dim == 2) 
        max_v = std::max(pr.max_vel(0), pr.max_vel(1));
      else if(Dim == 3)
        max_v = std::max(std::max(pr.max_vel(0), pr.max_vel(1)), pr.max_vel(2));
      int n = std::ceil(max_v * pr.t() / map_util_->getRes());
      vec_E<Waypoint<Dim>> pts = pr.sample(n);
      for(const auto& pt: pts) {
        Veci<Dim> pn = map_util_->floatToInt(pt.pos);
        if(map_util_->isOccupied(pn) || map_util_->isOutside(pn))
          return false;
      }

      return true;
    }

    /**
     * @brief Get successor
     *
     * @param curr The node to expand
     * @param succ The array stores valid successors
     * @param succ_idx The array stores successors' Key
     * @param succ_cost The array stores cost along valid edges
     * @param action_idx The array stores corresponding idx of control for each successor
     *
     * When goal is outside, extra step is needed for finding optimal trajectory.
     * Only return the primitive satisfies valid dynamic constriants (include the one hits obstacles).
     */
    void get_succ( const Waypoint<Dim>& curr, 
        vec_E<Waypoint<Dim>>& succ,
        std::vector<Key>& succ_idx,
        std::vector<decimal_t>& succ_cost,
        std::vector<int>& action_idx) const
    {
      succ.clear();
      succ_idx.clear();
      succ_cost.clear();
      action_idx.clear();

      this->expanded_nodes_.push_back(curr.pos);

      const Veci<Dim> pn = map_util_->floatToInt(curr.pos);
      if(map_util_->isOutside(pn))
        return;

      for(unsigned int i = 0; i < this->U_.size(); i++) {
        Primitive<Dim> pr(curr, this->U_[i], this->dt_);
        Waypoint<Dim> tn = pr.evaluate(this->dt_);
        if(tn == curr) 
          continue;
        if(pr.valid_vel(this->v_max_) && 
           pr.valid_acc(this->a_max_) && 
           pr.valid_jrk(this->j_max_)) {
          tn.use_pos = curr.use_pos;
          tn.use_vel = curr.use_vel;
          tn.use_acc = curr.use_acc;
          tn.use_jrk = curr.use_jrk;

          succ.push_back(tn);
          succ_idx.push_back(this->state_to_idx(tn));
          decimal_t cost = is_free(pr) ? pr.J(this->wi_) + 
            this->w_*this->dt_: std::numeric_limits<decimal_t>::infinity();
          succ_cost.push_back(cost);
          action_idx.push_back(i);
        }
      }

    }

  };

}

#endif
