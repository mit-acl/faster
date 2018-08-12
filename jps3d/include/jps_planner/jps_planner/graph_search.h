/**
 * @file graph_search.h
 * @brief backend of graph search, implementation of A* and JPS
 */

#ifndef JPS_GRAPH_SEARCH_H
#define JPS_GRAPH_SEARCH_H

#include <boost/heap/d_ary_heap.hpp>      // boost::heap::d_ary_heap
#include <memory>                         // std::shared_ptr
#include <limits>                         // std::numeric_limits
#include <vector>                         // std::vector
#include <unordered_map>                  // std::unordered_map

namespace JPS
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
    /// direction
    int dx, dy, dz;                            // discrete coordinates of this node
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
    State(int id, int x, int y, int dx, int dy )
      : id(id), x(x), y(y), dx(dx), dy(dy)
    {}

    /// 3D constructor
    State(int id, int x, int y, int z, int dx, int dy, int dz )
      : id(id), x(x), y(y), z(z), dx(dx), dy(dy), dz(dz)
    {}

  };

  ///Search and prune neighbors for JPS 2D
  struct JPS2DNeib {
    // for each (dx,dy) these contain:
    //    ns: neighbors that are always added
    //    f1: forced neighbors to check
    //    f2: neighbors to add if f1 is forced
    int ns[9][2][8];
    int f1[9][2][2];
    int f2[9][2][2];
    // nsz contains the number of neighbors for the four different types of moves:
    // no move (norm 0):        8 neighbors always added
    //                          0 forced neighbors to check (never happens)
    //                          0 neighbors to add if forced (never happens)
    // straight (norm 1):       1 neighbor always added
    //                          2 forced neighbors to check
    //                          2 neighbors to add if forced
    // diagonal (norm sqrt(2)): 3 neighbors always added
    //                          2 forced neighbors to check
    //                          2 neighbors to add if forced
    static constexpr int nsz[3][2] = {{8, 0}, {1, 2}, {3, 2}};

    void print();
    JPS2DNeib();
    private:
    void Neib(int dx, int dy, int norm1, int dev, int& tx, int& ty);
    void FNeib(int dx, int dy, int norm1, int dev,
        int& fx, int& fy, int& nx, int& ny);
  };


  ///Search and prune neighbors for JPS 3D
  struct JPS3DNeib {
    // for each (dx,dy,dz) these contain:
    //    ns: neighbors that are always added
    //    f1: forced neighbors to check
    //    f2: neighbors to add if f1 is forced
    int ns[27][3][26];
    int f1[27][3][12];
    int f2[27][3][12];
    // nsz contains the number of neighbors for the four different types of moves:
    // no move (norm 0):        26 neighbors always added
    //                          0 forced neighbors to check (never happens)
    //                          0 neighbors to add if forced (never happens)
    // straight (norm 1):       1 neighbor always added
    //                          8 forced neighbors to check
    //                          8 neighbors to add if forced
    // diagonal (norm sqrt(2)): 3 neighbors always added
    //                          8 forced neighbors to check
    //                          12 neighbors to add if forced
    // diagonal (norm sqrt(3)): 7 neighbors always added
    //                          6 forced neighbors to check
    //                          12 neighbors to add if forced
    static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
    JPS3DNeib();
    private:
    void Neib(int dx, int dy, int dz, int norm1, int dev, int& tx, int& ty, int& tz);
    void FNeib( int dx, int dy, int dz, int norm1, int dev,
        int& fx, int& fy, int& fz,
        int& nx, int& ny, int& nz);
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
       * @param verbose flag for printing debug info, optional, default as false
       */
      GraphSearch(const char* cMap, int xDim, int yDim, double eps = 1, bool verbose = false);
      /**
       * @brief 3D graph search constructor
       *
       * @param cMap 1D array stores the occupancy, with the order equal to \f$x + xDim * y + xDim * yDim * z\f$
       * @param xDim map length
       * @param yDim map width
       * @param zDim map height
       * @param eps weight of heuristic, optional, default as 1
       * @param verbose flag for printing debug info, optional, default as False
       */
      GraphSearch(const char* cMap, int xDim, int yDim, int zDim, double eps = 1, bool verbose = false);

      /**
       * @brief start 2D planning thread
       *
       * @param xStart start x coordinate
       * @param yStart start y coordinate
       * @param xGoal goal x coordinate
       * @param yGoal goal y coordinate
       * @param useJps if true, enable JPS search; else the planner is implementing A*
       * @param maxExpand maximum number of expansion allowed, optional, default is -1, means no limitation
       */
      bool plan(int xStart, int yStart, int xGoal, int yGoal, bool useJps, int maxExpand = -1);
      /**
       * @brief start 3D planning thread
       *
       * @param xStart start x coordinate
       * @param yStart start y coordinate
       * @param zStart start z coordinate
       * @param xGoal goal x coordinate
       * @param yGoal goal y coordinate
       * @param zGoal goal z coordinate
       * @param useJps if true, enable JPS search; else the planner is implementing A*
       * @param maxExpand maximum number of expansion allowed, optional, default is -1, means no limitation
       */
      bool plan(int xStart, int yStart, int zStart, int xGoal, int yGoal, int zGoal, bool useJps, int maxExpand = -1);

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
      bool plan(StatePtr& currNode_ptr, int max_expand, int start_id, int goal_id);
      /// Get successor function for A*
      void getSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs);
      /// Get successor function for JPS
      void getJpsSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs);
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

      /// Check if (x, y) is occupied
      bool isOccupied(int x, int y) const;
      /// Check if (x, y, z) is occupied
      bool isOccupied(int x, int y, int z) const;

      /// Clculate heuristic
      double getHeur(int x, int y) const;
      /// Clculate heuristic
      double getHeur(int x, int y, int z) const;

      /// Determine if (x, y) has forced neighbor with direction (dx, dy)
      bool hasForced(int x, int y, int dx, int dy);
      /// Determine if (x, y, z) has forced neighbor with direction (dx, dy, dz)
      bool hasForced(int x, int y, int z, int dx, int dy, int dz);

      /// 2D jump, return true iff finding the goal or a jump point
      bool jump(int x, int y, int dx, int dy, int& new_x, int& new_y);
      /// 3D jump, return true iff finding the goal or a jump point
      bool jump(int x, int y, int z, int dx, int dy, int dz, int& new_x, int& new_y, int& new_z);

      /// Initialize 2D jps arrays
      void init2DJps();

      const char* cMap_;
      int xDim_, yDim_, zDim_;
      double eps_;
      bool verbose_;

      const char val_free_ = 0;
      int xGoal_, yGoal_, zGoal_;
      bool use_2d_;
      bool use_jps_ = false;

      priorityQueue pq_;
      std::vector<StatePtr> hm_;
      std::vector<bool> seen_;

      std::vector<StatePtr> path_;

      std::vector<std::vector<int>> ns_;
      std::shared_ptr<JPS2DNeib> jn2d_;
      std::shared_ptr<JPS3DNeib> jn3d_;
 };
}
#endif
