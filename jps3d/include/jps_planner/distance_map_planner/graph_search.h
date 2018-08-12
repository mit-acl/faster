/**
 * @file graph_search.h
 * @brief backend of graph search for distance map
 */

#ifndef DMP_GRAPH_SEARCH_H
#define DMP_GRAPH_SEARCH_H

#include <boost/heap/d_ary_heap.hpp>      // boost::heap::d_ary_heap
#include <memory>                         // std::shared_ptr
#include <limits>                         // std::numeric_limits
#include <vector>                         // std::vector
#include <unordered_map>                  // std::unordered_map

namespace DMP
{
  ///Heap element comparison
  template <class T>
  struct compare_state
  {
    bool operator()(T a1, T a2) const
    {
      double f1 = a1->g + a1->h;
      double f2 = a2->g + a2->h;
      if( ( f1 >= f2 - 0.000001) && (f1 <= f2 +0.000001) )
        return a1->g < a2->g; // if equal compare gvals
      return f1 > f2;
    }
  };


  ///Define priority queue
  struct State; // forward declaration
  ///State pointer
  using StatePtr = std::shared_ptr<State>;
  using priorityQueue = boost::heap::d_ary_heap<StatePtr, boost::heap::mutable_<true>,
                        boost::heap::arity<2>, boost::heap::compare< compare_state<StatePtr> >>;

  ///Node of the graph in graph search
  struct State
  {
    /// ID
    int id;
    /// Coord
    int x, y, z = 0;
    /// id of predicessors
    int parentId = -1;

    /// pointer to heap location
    priorityQueue::handle_type heapkey;

    /// g cost
    double g = std::numeric_limits<double>::infinity();
    /// heuristic cost
    double h;
    /// if has been opened
    bool opened = false;
    /// if has been closed
    bool closed = false;

    /// 2D constructor
    State(int id, int x, int y)
      : id(id), x(x), y(y)
    {}

    /// 3D constructor
    State(int id, int x, int y, int z)
      : id(id), x(x), y(y), z(z)
    {}

  };


  /**
   * @brief GraphSearch class
   *
   * Implement A* and Jump Point Search
   */
  class GraphSearch
  {
    public:
     /**
       * @brief 2D graph search constructor 
       *
       * @param cMap 1D array stores the occupancy, with the order equal to \f$x + xDim * y\f$
       * @param xDim map length
       * @param yDim map width
       * @param eps weight of heuristic, optional, default as 1
       * @param cweight weight of distance cost, optional, default as 0.1
       * @param verbose flag for printing debug info, optional, default as false
       */
      GraphSearch(const int8_t* cMap, int xDim, int yDim, double eps = 1, double cweight = 0.1, bool verbose = false);
      /**
       * @brief 3D graph search constructor 
       *
       * @param cMap 1D array stores the occupancy, with the order equal to \f$x + xDim * y + xDim * yDim * z\f$
       * @param xDim map length
       * @param yDim map width
       * @param zDim map height
       * @param eps weight of heuristic, optional, default as 1
       * @param cweight weight of distance cost, optional, default as 0.1
       * @param verbose flag for printing debug info, optional, default as False
       */
      GraphSearch(const int8_t* cMap, int xDim, int yDim, int zDim, double eps = 1, double cweight = 0.1, bool verbose = false);

      /** 
       * @brief start 2D planning thread
       *
       * @param xStart start x coordinate
       * @param yStart start y coordinate
       * @param xGoal goal x coordinate
       * @param yGoal goal y coordinate
       * @param in_region a region that is valid for searching, empty means no boundary
       */
      bool plan(int xStart, int yStart, int xGoal, int yGoal, std::vector<bool> in_region = std::vector<bool>());
      /** 
       * @brief start 3D planning thread
       *
       * @param xStart start x coordinate
       * @param yStart start y coordinate
       * @param zStart start z coordinate
       * @param xGoal goal x coordinate
       * @param yGoal goal y coordinate
       * @param zGoal goal z coordinate
       * @param in_region a region that is valid for searching, empty means no boundary
       */
      bool plan(int xStart, int yStart, int zStart, int xGoal, int yGoal, int zGoal, std::vector<bool> in_region = std::vector<bool>());

      /// Get the optimal path
      std::vector<StatePtr> getPath() const;

      /// Get the states in open set
      std::vector<StatePtr> getOpenSet() const;

      /// Get the states in close set
      std::vector<StatePtr> getCloseSet() const;

      /// Get the states in hash map
      std::vector<StatePtr> getAllSet() const;

    private:
      /// Main planning loop
      bool plan(StatePtr& currNode_ptr, int start_id, int goal_id);
      /// Get successor function for A*
      void getSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs);
      /// Recover the optimal path
      std::vector<StatePtr> recoverPath(StatePtr node, int id);

      /// Get subscript
      int coordToId(int x, int y) const;
      /// Get subscript
      int coordToId(int x, int y, int z) const;

      /// Check if (x, y) is free
      bool isFree(int x, int y) const;
      /// Check if (x, y, z) is free
      bool isFree(int x, int y, int z) const;

      /// Clculate heuristic
      double getHeur(int x, int y) const;
      /// Clculate heuristic
      double getHeur(int x, int y, int z) const;

      const int8_t* cMap_;
      int xDim_, yDim_, zDim_;
      /// weight of heuristic
      double eps_;
      /// weight of distance map
      double cweight_;
      bool verbose_;

      const int8_t val_free_ = 0;
      const int8_t val_occ_ = 100;
      int xGoal_, yGoal_, zGoal_;
      bool use_2d_;
      bool global_;

      priorityQueue pq_;
      std::vector<StatePtr> hm_;
      std::vector<bool> seen_;
      std::vector<bool> in_region_;

      std::vector<StatePtr> path_;

      std::vector<std::vector<int>> ns_;
 };
}
#endif
