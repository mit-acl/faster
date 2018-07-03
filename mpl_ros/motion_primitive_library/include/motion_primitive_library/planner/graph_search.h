/**
 * @file graph_search.h
 * @brief backend of graph search, implemetation of A* and Lifelong Planning A*
 */

#ifndef GRAPH_SEARCH_H
#define GRAPH_SEARCH_H

#include <motion_primitive_library/planner/state_space.h>
#include <motion_primitive_library/primitive/trajectory.h>

namespace MPL
{

  /**
   * @brief GraphSearch class
   *
   * Implement A* and Lifelong Planning A*
   */
  template <int Dim>
  class GraphSearch
  {
    public:
      /**
       * @brief Simple empty constructor
       *
       * @param verbose enable print out debug infos, default is set to False
       */
      GraphSearch(bool verbose = false) : verbose_(verbose) {};

      /**
       * @brief Astar graph search
       *
       * @param start_coord start state
       * @param start_key key of the start state 
       * @param ENV object of `env_base' class
       * @param ss_ptr workspace input
       * @param traj output trajectory
       * @param max_expand max number of expanded states, default value is -1 which means there is no limitation
       * @param max_t max time horizon of expanded states, default value is -1 which means there is no limitation
       */
      decimal_t Astar(const Waypoint<Dim>& start_coord, Key start_key,
          const std::shared_ptr<env_base<Dim>>& ENV, std::shared_ptr<StateSpace<Dim>>& ss_ptr, 
          Trajectory<Dim>& traj, int max_expand = -1, decimal_t max_t = 0);
      /**
       * @brief Lifelong Planning Astar graph search
       *
       * @param start_coord start state
       * @param start_key key of the start state 
       * @param ENV object of `env_base' class
       * @param ss_ptr workspace input
       * @param traj output trajectory
       * @param max_expand max number of expanded states, default value is -1 which means there is no limitation
       * @param max_t max time horizon of expanded states, default value is -1 which means there is no limitation
       */
      decimal_t LPAstar(const Waypoint<Dim>& start_coord, Key start_key, 
          const std::shared_ptr<env_base<Dim>>& ENV, std::shared_ptr<StateSpace<Dim>>& ss_ptr, 
          Trajectory<Dim>& traj, int max_expand = -1, decimal_t max_t = 0);
   private:
      ///Recover trajectory 
      Trajectory<Dim> recoverTraj(StatePtr<Dim> ptr, std::shared_ptr<StateSpace<Dim>> ss_ptr, 
          const std::shared_ptr<env_base<Dim>>& ENV, const Key& start_idx);
      ///Verbose flag
      bool verbose_ = false;
 };
}
#endif
