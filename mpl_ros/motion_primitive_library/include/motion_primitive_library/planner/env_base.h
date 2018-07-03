/**
 * @file env_base.h
 * @brief environment base class
 */

#ifndef ENV_BASE_H
#define ENV_BASE_H

#include <memory>
#include <vector>
#include <motion_primitive_library/primitive/primitive.h>
#include <motion_primitive_library/primitive/trajectory.h>

namespace MPL {
  
/**
 * @brief Key for node
 *
 * We use string as the Key for indexing, by default the Key refers to 'pos-vel-acc-...'
 */
typedef std::string Key; 

/**
 * @brief Base environment class
 */
template <int Dim>
class env_base
{
  public:
    ///Simple constructor
    env_base() {}

    ///Check if state hit the goal region, use L-1 norm
    virtual bool is_goal(const Waypoint<Dim>& state) const {
      bool goaled = (state.pos - goal_node_.pos).template lpNorm<Eigen::Infinity>() <= tol_dis;
      if(goaled && goal_node_.use_vel && tol_vel > 0) 
        goaled = (state.vel - goal_node_.vel).template lpNorm<Eigen::Infinity>() <= tol_vel;
      if(goaled && goal_node_.use_acc && tol_acc > 0) 
        goaled = (state.acc - goal_node_.acc).template lpNorm<Eigen::Infinity>() <= tol_acc;
     return goaled;
    }

    /**
     * @brief Heuristic function 
     * @param Waypoint current state coord
     * @param t current state time
     */
    decimal_t get_heur(const Waypoint<Dim>& state, decimal_t t) const
    {
      if(goal_node_ == state) 
        return 0;
      Waypoint<Dim> goal_node = goal_node_;
      t += alpha_ * dt_;
      if(!prior_traj_.segs.empty() && t < prior_traj_.getTotalTime()) {
        prior_traj_.evaluate(t, goal_node);
        goal_node.use_pos = goal_node_.use_pos;
        goal_node.use_vel = goal_node_.use_vel;
        goal_node.use_acc = goal_node_.use_acc;
        goal_node.use_jrk = goal_node_.use_jrk;
        return cal_heur(state, goal_node) + w_ * (prior_traj_.getTotalTime() - t);
      }

      return cal_heur(state, goal_node);
    }

    /// calculate the cost from state to goal
    decimal_t cal_heur(const Waypoint<Dim>& state, const Waypoint<Dim>& goal) const
    {
      //return 0;
      //return w_*(state.pos - goal.pos).norm();
      //If in acceleration control space
      if(state.use_pos && state.use_vel && state.use_acc && !state.use_jrk &&
         goal.use_pos && goal.use_vel && goal.use_acc && !goal.use_jrk) {
 
        const Vecf<Dim> dp = goal.pos - state.pos;
        const Vecf<Dim> v0 = state.vel;
        const Vecf<Dim> v1 = goal.vel;
        const Vecf<Dim> a0 = state.acc;
        const Vecf<Dim> a1 = goal.acc;
        decimal_t a = w_;
        decimal_t b = 0;
        decimal_t c = -9*a0.dot(a0)+6*a0.dot(a1)-9*a1.dot(a1);
        decimal_t d = -144*a0.dot(v0)-96*a0.dot(v1)+96*a1.dot(v0)+144*a1.dot(v1);
        decimal_t e = 360*(a0-a1).dot(dp)-576*v0.dot(v0)-1008*v0.dot(v1)-576*v1.dot(v1);
        decimal_t f = 2880*dp.dot(v0+v1);
        decimal_t g = -3600*dp.dot(dp);

        std::vector<decimal_t> ts = solve(a, b, c, d, e, f, g);

        decimal_t t_bar = (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
        ts.push_back(t_bar);
        decimal_t min_cost = std::numeric_limits<decimal_t>::max();
        for(auto t: ts) {
          if(t < t_bar)
           continue;
          decimal_t cost = a*t-c/t-d/2/t/t-e/3/t/t/t-f/4/t/t/t/t-g/5/t/t/t/t/t;
          if(cost < min_cost) 
            min_cost = cost;
        }
        return min_cost;
      }

      else if(state.use_pos && state.use_vel && state.use_acc && !state.use_jrk &&
         goal.use_pos && goal.use_vel && !goal.use_acc && !goal.use_jrk) {
        const Vecf<Dim> dp = goal.pos - state.pos;
        const Vecf<Dim> v0 = state.vel;
        const Vecf<Dim> v1 = goal.vel;
        const Vecf<Dim> a0 = state.acc;

        decimal_t a = w_;
        decimal_t b = 0;
        decimal_t c = -8*a0.dot(a0);
        decimal_t d = -112*a0.dot(v0)-48*a0.dot(v1);
        decimal_t e = 240*a0.dot(dp)-384*v0.dot(v0)-432*v0.dot(v1)-144*v1.dot(v1);
        decimal_t f = dp.dot(1600*v0+960*v1);
        decimal_t g = -1600*dp.dot(dp);

        std::vector<decimal_t> ts = solve(a, b, c, d, e, f, g);

        decimal_t t_bar = (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
        ts.push_back(t_bar);
        decimal_t min_cost = std::numeric_limits<decimal_t>::max();
        for(auto t: ts) {
          if(t < t_bar)
            continue;
          decimal_t cost = a*t-c/t-d/2/t/t-e/3/t/t/t-f/4/t/t/t/t-g/5/t/t/t/t/t;
          if(cost < min_cost)
            min_cost = cost;
          //printf("t: %f, cost: %f\n",t, cost);
        }
        return min_cost;
      }

      else if(state.use_pos && state.use_vel && state.use_acc && !state.use_jrk &&
         goal.use_pos && !goal.use_vel && !goal.use_acc && !goal.use_jrk) {
        const Vecf<Dim> dp = goal.pos - state.pos;
        const Vecf<Dim> v0 = state.vel;
        const Vecf<Dim> a0 = state.acc;

        decimal_t a = w_;
        decimal_t b = 0;
        decimal_t c = -5*a0.dot(a0);
        decimal_t d = -40*a0.dot(v0);
        decimal_t e = 60*a0.dot(dp)-60*v0.dot(v0);
        decimal_t f = 160*dp.dot(v0);
        decimal_t g = -100*dp.dot(dp);

        std::vector<decimal_t> ts = solve(a, b, c, d, e, f, g);

        decimal_t t_bar = (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
        ts.push_back(t_bar);

        decimal_t min_cost = std::numeric_limits<decimal_t>::max();
        for(auto t: ts) {
          if(t < t_bar)
            continue;
          decimal_t cost = a*t-c/t-d/2/t/t-e/3/t/t/t-f/4/t/t/t/t-g/5/t/t/t/t/t;
          if(cost < min_cost) 
            min_cost = cost;
        }
        return min_cost;
      }


      else if(state.use_pos && state.use_vel && !state.use_acc && !state.use_jrk &&
              goal.use_pos && goal.use_vel && !goal.use_acc && !goal.use_jrk) {
        const Vecf<Dim> dp = goal.pos - state.pos;
        const Vecf<Dim> v0 = state.vel;
        const Vecf<Dim> v1 = goal.vel;

        decimal_t c1 = -36*dp.dot(dp);
        decimal_t c2 = 24*(v0+v1).dot(dp);
        decimal_t c3 = -4*(v0.dot(v0)+v0.dot(v1)+v1.dot(v1));
        decimal_t c4 = 0;
        decimal_t c5 = w_;

        std::vector<decimal_t> ts = quartic(c5, c4, c3, c2, c1);
        decimal_t t_bar = (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
        ts.push_back(t_bar);

        decimal_t cost = std::numeric_limits<decimal_t>::max();
        for(auto t: ts) {
          if(t < t_bar)
            continue;
          decimal_t c = -c1/3/t/t/t-c2/2/t/t-c3/t+w_*t;
          if(c < cost)
            cost = c;
        }

        return cost;
      }

      else if(state.use_pos && state.use_vel && !state.use_acc && !state.use_jrk &&
          goal.use_pos && !goal.use_vel && !goal.use_acc && !goal.use_jrk) {
        const Vecf<Dim> dp = goal.pos - state.pos;
        const Vecf<Dim> v0 = state.vel;

        decimal_t c1 = -9*dp.dot(dp);
        decimal_t c2 = 12*v0.dot(dp);
        decimal_t c3 = -3*v0.dot(v0);
        decimal_t c4 = 0;
        decimal_t c5 = w_;

        std::vector<decimal_t> ts = quartic(c5, c4, c3, c2, c1);
        decimal_t t_bar = (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
        ts.push_back(t_bar);

        decimal_t cost = std::numeric_limits<decimal_t>::max();
        for(auto t: ts) {
          if(t < t_bar)
            continue;
          decimal_t c = -c1/3/t/t/t-c2/2/t/t-c3/t+w_*t;
          if(c < cost)
            cost = c;
        }

        return cost;
      }
 
      else if(state.use_pos && !state.use_vel && !state.use_acc && !state.use_jrk &&
              goal.use_pos && !goal.use_vel && !goal.use_acc && !goal.use_jrk)
        return (w_ + 1) * (state.pos - goal.pos).norm();
      else
        return w_*(state.pos - goal.pos).norm() / v_max_;
    }

    ///Replace the original cast function
    inline Veci<Dim> round(const Vecf<Dim>& vec, decimal_t res) const {
      Veci<Dim> vecI;
      for(int i = 0; i < Dim; i++)
        vecI(i) = std::round(vec(i) / res);
      return vecI;
    }

    ///Convert a vec to a string
    std::string toString(const Veci<Dim>& vec) const {
      std::string str;
      for(int i = 0; i < Dim; i++)
        str += std::to_string(vec(i)) + "-";
      return str;
    }

    ///Genegrate Key from state
    Key state_to_idx(const Waypoint<Dim>& state) const {
      const Veci<Dim> pi = round(state.pos, ds_);
      if(state.use_pos && state.use_vel && !state.use_acc ) {
        const Veci<Dim> vi = round(state.vel, dv_);
        return toString(pi) + toString(vi);
      }
      else if(state.use_pos && state.use_vel && state.use_acc && !state.use_jrk) {
        const Veci<Dim> vi = round(state.vel, dv_);
        const Veci<Dim> ai = round(state.acc, da_);
        return toString(pi) + toString(vi) + toString(ai);
      }
      else if(state.use_pos && state.use_vel && state.use_acc && state.use_jrk) {
        const Veci<Dim> vi = round(state.vel, dv_);
        const Veci<Dim> ai = round(state.acc, da_);
        const Veci<Dim> ji = round(state.jrk, dj_);
        return toString(pi) + toString(vi) + toString(ai) + toString(ji);
      }
      else 
        return toString(pi);
    }

    ///Recover trajectory
    void forward_action( const Waypoint<Dim>& curr, int action_id, Primitive<Dim>& pr) const
    {
      pr = Primitive<Dim>(curr, U_[action_id], dt_);
    }

    ///Set max U in each axis
    void set_U(const vec_Vecf<Dim>& U) {
      U_ = U;
    }

    ///Set max vel in each axis
    void set_v_max(decimal_t v) {
      v_max_ = v;
    }

    ///Set max acc in each axis
    void set_a_max(decimal_t a) {
      a_max_ = a;
    }

    ///Set max acc in each axis
    void set_j_max(decimal_t j) {
      j_max_ = j;
    }

    ///Set max control in each axis
    void set_u_max(decimal_t u) {
      u_max_ = u;
    }

    ///Set max amount of time step to explore 
    void set_t_max(decimal_t t) {
      t_max_ = t;
    }

    ///Set prior trajectory 
    void set_prior_trajectory(const Trajectory<Dim>& traj) {
      prior_traj_ = traj;
    }

    ///Set dt for primitive
    void set_dt(decimal_t dt) {
      dt_ = dt;
    }

    ///Set distance tolerance for goal region
    void set_tol_dis(decimal_t dis) {
      tol_dis = dis;
    }

    ///Set velocity tolerance for goal region
    void set_tol_vel(decimal_t vel) {
      tol_vel = vel;
    }

    ///Set acceleration tolerance for goal region
    void set_tol_acc(decimal_t acc) {
      tol_acc = acc;
    }

    ///set weight for cost in time, usually no need to change
    void set_w(decimal_t w) {
      w_ = w;
    }

    ///Set derivative order for cost in effort, dont need to set manually
    void set_wi(int wi) {
      wi_ = wi;
    }

    ///Set alpha
    void set_alpha(int alpha) {
      alpha_ = alpha;
    }

    ///Set goal state
    void set_goal(const Waypoint<Dim>& state) {
      goal_node_ = state;
    }

    ///Print out params
    void info() {
      printf(ANSI_COLOR_YELLOW "\n");
      printf("++++++++++ PLANNER +++++++++++\n");
      printf("+    alpha: %d                 +\n", alpha_);
      printf("+       dt: %.2f               +\n", dt_);
      printf("+        w: %.2f               +\n", w_);
      printf("+       wi: %d                 +\n", wi_);
      printf("+    v_max: %.2f               +\n", v_max_);
      printf("+    a_max: %.2f               +\n", a_max_);
      printf("+    j_max: %.2f               +\n", j_max_);
      printf("+    u_max: %.2f               +\n", u_max_);
      printf("+    t_max: %.2f               +\n", t_max_);
      printf("+    U num: %zu                +\n", U_.size());
      printf("+  tol_dis: %.2f               +\n", tol_dis);
      printf("+  tol_vel: %.2f               +\n", tol_vel);
      printf("+  tol_acc: %.2f               +\n", tol_acc);
      printf("++++++++++ PLANNER +++++++++++\n");
      printf(ANSI_COLOR_RESET "\n");
    }

    ///Check if a point is in free space
    virtual bool is_free(const Vecf<Dim>& pt) const { 
      printf("Used Null is_free() for pt\n");
      return true; 
    }

    ///Check if a primitive is in free space
    virtual bool is_free(const Primitive<Dim>& pr) const { 
      printf("Used Null is_free() for pr\n");
      return true; 
    }

    ///Retrieve dt
    decimal_t get_dt() const {
      return dt_;
    }

    /**
     * @brief Get successor
     * @param curr The node to expand
     * @param succ The array stores valid successors
     * @param succ_idx The array stores successors' Key
     * @param succ_cost The array stores cost along valid edges
     * @param action_idx The array stores corresponding idx of control for each successor
     */
    virtual void get_succ( const Waypoint<Dim>& curr, 
        vec_E<Waypoint<Dim>>& succ,
        std::vector<Key>& succ_idx,
        std::vector<decimal_t>& succ_cost,
        std::vector<int>& action_idx) const
    {
      printf("Used Null get_succ()\n");
      succ.push_back(curr);
      succ_idx.push_back( state_to_idx(curr) );
      succ_cost.push_back(0);
      action_idx.push_back(0);
    }

    ///weight of time cost
    decimal_t w_ = 10; 
    ///order of derivatives for effort
    int wi_; 
    ///heuristic time offset
    int alpha_ = 0;

    ///tolerance of position for goal region
    decimal_t tol_dis = 0.0;
    ///tolerance of velocity for goal region, 0 means no tolerance
    decimal_t tol_vel = 0.0;
    ///tolerance of acceleration for goal region, 0 means no tolerance
    decimal_t tol_acc = 0.0;
    ///max control input
    decimal_t u_max_;
    ///max velocity
    decimal_t v_max_ = -1;
    ///max acceleration
    decimal_t a_max_ = -1;
    ///max jerk
    decimal_t j_max_ = -1;
    ///max execution time
    decimal_t t_max_ = -1;
    ///duration of primitive
    decimal_t dt_ = 1.0;
    ///grid size in position
    decimal_t ds_ = 0.01;
    ///grid size in velocity
    decimal_t dv_ = 0.1;
    ///grid size in acceleration
    decimal_t da_ = 0.1;
    ///grid size in jerk
    decimal_t dj_ = 0.1;
    ///expanded nodes
    mutable vec_Vecf<Dim> expanded_nodes_;
    ///Array of constant control input
    vec_Vecf<Dim> U_;
    ///Goal node
    Waypoint<Dim> goal_node_;
    ///Prior trajectory
    Trajectory<Dim> prior_traj_;

};
}


#endif
