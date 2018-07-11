/**
 * @file env_cloud.h
 * @biref environment for planning using point cloud
 */

#ifndef ENV_CLOUD_H
#define ENV_CLOUD_H
#include <motion_primitive_library/planner/env_base.h>
#include <motion_primitive_library/primitive/primitive.h>
#include <motion_primitive_library/collision_checking/decomp_util.h>

namespace MPL
{
/**
 * @brief Point cloud environment
 */
class env_cloud : public env_base<3>
{
protected:
  std::unique_ptr<DecompUtil> map_util_;

public:
  /// Simple constructor
  env_cloud()
  {
  }
  /// Simple constructor
  env_cloud(const vec_Vec3f& obs, decimal_t r, decimal_t h, const Vec3f& ori, const Vec3f& dim)
  {
    map_util_.reset(new DecompUtil(r, h));
    map_util_->setObstacles(obs);
    map_util_->set_region(ori, dim);
  }

  ~env_cloud()
  {
  }

  /// Check if a point is in free space
  bool is_free(const Vec3f& pt) const
  {
    return true;
    // return map_util_->isFree(pt);
  }

  /**
   * @brief Get successor
   * @param curr The node to expand
   * @param succ The array stores valid successors
   * @param succ_idx The array stores successors' Key
   * @param succ_cost The array stores cost along valid edges
   * @param action_idx The array stores corresponding idx of control for each successor
   *
   * When goal is outside, extra step is needed for finding optimal trajectory
   * Here we use Heuristic function and multiply with 2
   */
  void get_succ(const Waypoint3& curr, vec_E<Waypoint3>& succ, std::vector<Key>& succ_idx,
                std::vector<decimal_t>& succ_cost, std::vector<int>& action_idx) const
  {
    succ.clear();
    succ_idx.clear();
    succ_cost.clear();
    action_idx.clear();

    expanded_nodes_.push_back(curr.pos);
    // ws_.push_back(curr);
    for (int i = 0; i < (int)U_.size(); i++)
    {
      Primitive3 pr(curr, U_[i], dt_);  // Primitive obtained applying the input U_[i] to the current node
      Waypoint3 tn = pr.evaluate(dt_);  // State in that primitive when t=dt
      if (pr.valid_vel(v_max_) && pr.valid_acc(a_max_))
      {  // Aqui se podria anadir lo que yo quiero (planear hasta una distancia con jerk y hasta otra con accel??)
        bool valid = map_util_->isFree(pr);  // if that state is collision free
        if (valid)
        {
          if (tn.pos)
            // primitives_.push_back(pr);
            tn.use_pos = curr.use_pos;
          tn.use_vel = curr.use_vel;
          tn.use_acc = curr.use_acc;
          tn.use_jrk = curr.use_jrk;

          succ.push_back(tn);                         // insert that state to the end of the vector succ
          succ_idx.push_back(state_to_idx(tn));       // insert its index
          succ_cost.push_back(pr.J(wi_) + w_ * dt_);  // and its cost
          action_idx.push_back(i);
        }
      }
    }

    // if(t_max_ > 0 && curr.t >= t_max_)
    // return;
  }
};
}

#endif
