/**
 * @file state_space.h
 * @brief state space class for graph search
 */

#include <boost/heap/d_ary_heap.hpp>      // boost::heap::d_ary_heap
#include <memory>                         // std::shared_ptr
#include <limits>                         // std::numeric_limits
#include <vector>                         // std::vector
#include <unordered_map> // std::unordered_map
#include <motion_primitive_library/primitive/primitive.h> 
#include <motion_primitive_library/planner/env_base.h> 

namespace MPL
{
  ///Key for hashmap
  typedef std::string Key;

  ///Heap element comparison
  template <class state>
  struct compare_pair
  {
    bool operator()(const std::pair<decimal_t, std::shared_ptr<state>>& p1, 
                    const std::pair<decimal_t, std::shared_ptr<state>>& p2) const
    {
      if( p1.first == p2.first )
      {
        // if equal compare gvals
        return std::min(p1.second->g, p1.second->rhs) > std::min(p2.second->g, p2.second->rhs);
      }
      return p1.first > p2.first;
    }
  };  

  ///Define priority queue
  template <class state>
  using priorityQueue = boost::heap::d_ary_heap<std::pair<decimal_t,std::shared_ptr<state>>, boost::heap::mutable_<true>, boost::heap::arity<2>, boost::heap::compare< compare_pair<state> >>;


  ///Lattice of the graph in graph search
  template <int Dim>
  struct State
  {
    /// hash key in the hashmap
    Key hashkey; // discrete coordinates of this node
    /// state
    Waypoint<Dim> coord; 
    /// minimum arrival time
    decimal_t t;
    /// coordinates of successors
    vec_E<Waypoint<Dim>> succ_coord;
    /// hashkey of successors
    std::vector<Key> succ_hashkey;
    /// action id of successors
    std::vector<int> succ_action_id;
    /// action cost of successors
    std::vector<decimal_t> succ_action_cost;
    /// hashkey of predecessors
    std::vector<Key> pred_hashkey;
    /// action id of predecessors
    std::vector<int> pred_action_id;
    /// action cost of predecessors
    std::vector<decimal_t> pred_action_cost;

    /// pointer to heap location
    typename priorityQueue<State<Dim>>::handle_type heapkey;

    // plan data
    /// start-to-state g value
    decimal_t g = std::numeric_limits<decimal_t>::infinity();
    /// rhs value based on g value
    decimal_t rhs = std::numeric_limits<decimal_t>::infinity();
    /// heuristic cost
    decimal_t h = std::numeric_limits<decimal_t>::infinity();
    /// label check if the state has been in the open set
    bool iterationopened = false;
    /// label check if the state has been closed
    bool iterationclosed = false;

    /// Simple constructor
    State( Key hashkey, const Waypoint<Dim>& coord )
      : hashkey(hashkey), coord(coord)
    {}

  };

  ///Declare StatePtr
  template <int Dim>
  using StatePtr = std::shared_ptr<State<Dim>>;
  
  ///Define hashmap type
  template <int Dim>
  using hashMap = std::unordered_map<Key, StatePtr<Dim>>;

  ///State space
  template <int Dim>
  struct StateSpace
  {
    ///Priority queue, open set
    priorityQueue<State<Dim>> pq_;
    ///Hashmap, stores all the nodes
    hashMap<Dim> hm_;
    ///Heuristic weight, default as 1
    decimal_t eps_;
    ///Execution time for each primitive
    decimal_t dt_;
    ///The best trajectory from previous plan
    vec_E<StatePtr<Dim>> best_child_;
    ///Maximum time of the valid trajectories
    decimal_t max_t_ = std::numeric_limits<decimal_t>::infinity();
    ///Number of expansion iteration
    int expand_iteration_ = 0;

    ///Simple constructor
    StateSpace(decimal_t eps = 1): eps_(eps){}

    /**
     * @brief Get the subtree
     * @param time_step indicates the root of the subtree (best_child_[time_step])
    */
    void getSubStateSpace(int time_step);
    
    /**
     * @brief Update goal
     * @param ENV pointer of `env_base' class
     * @param goal if changed, use new goal to calculate heuristic
     */
    void updateGoal(std::shared_ptr<env_base<Dim>>& ENV, const Waypoint<Dim>& goal);
    ///Increase the cost of actions 
    vec_E<Primitive<Dim>> increaseCost(std::vector<std::pair<Key, int> > states, const std::shared_ptr<env_base<Dim>>& ENV);
    ///Decrease the cost of actions
    vec_E<Primitive<Dim>> decreaseCost(std::vector<std::pair<Key, int> > states, const std::shared_ptr<env_base<Dim>>& ENV);
    ///Update the node in the graph
    void updateNode(StatePtr<Dim>& currNode_ptr);

    ///Calculate the fval as min(rhs, g) + h
    decimal_t calculateKey(const StatePtr<Dim>& node);

    ///Check if the trajectory is blocked by new obstacle
    bool isBlocked();
    ///Internal function to check if the graph is valid
    void checkValidation(const hashMap<Dim>& hm);
  };



}
