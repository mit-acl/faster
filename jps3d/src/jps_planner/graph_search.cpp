#include <jps_planner/jps_planner/graph_search.h>
#include <cmath>

using namespace JPS;

GraphSearch::GraphSearch(const char* cMap, int xDim, int yDim, double eps, bool verbose) :
  cMap_(cMap), xDim_(xDim), yDim_(yDim), eps_(eps), verbose_(verbose)
{
  hm_.resize(xDim_ * yDim_);
  seen_.resize(xDim_ * yDim_, false);

  for(int x = -1; x <= 1; x ++) {
    for(int y = -1; y <= 1; y ++) {
      if(x == 0 && y == 0) continue;
      ns_.push_back(std::vector<int>{x, y});
    }
  }

  jn2d_ = std::make_shared<JPS2DNeib>();
}

GraphSearch::GraphSearch(const char* cMap, int xDim, int yDim, int zDim, double eps, bool verbose) :
  cMap_(cMap), xDim_(xDim), yDim_(yDim), zDim_(zDim), eps_(eps), verbose_(verbose)
{
  hm_.resize(xDim_ * yDim_ * zDim_);
  seen_.resize(xDim_ * yDim_ * zDim_, false);

  // Set 3D neighbors
  for(int x = -1; x <= 1; x ++) {
    for(int y = -1; y <= 1; y ++) {
      for(int z = -1; z <= 1; z ++) {
        if(x == 0 && y == 0 && z == 0) continue;
        ns_.push_back(std::vector<int>{x, y, z});
      }
    }
  }
  jn3d_ = std::make_shared<JPS3DNeib>();
}


inline int GraphSearch::coordToId(int x, int y) const {
  return x + y*xDim_;
}

inline int GraphSearch::coordToId(int x, int y, int z) const {
  return x + y*xDim_ + z*xDim_*yDim_;
}

inline bool GraphSearch::isFree(int x, int y) const {
  return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ &&
    cMap_[coordToId(x, y)] == val_free_;
}

inline bool GraphSearch::isFree(int x, int y, int z) const {
  return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ && z >= 0 && z < zDim_ &&
    cMap_[coordToId(x, y, z)] == val_free_;
}

inline bool GraphSearch::isOccupied(int x, int y) const {
  return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ &&
    cMap_[coordToId(x, y)] > val_free_;
}

inline bool GraphSearch::isOccupied(int x, int y, int z) const {
  return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ && z >= 0 && z < zDim_ &&
    cMap_[coordToId(x, y, z)] > val_free_;
}

inline double GraphSearch::getHeur(int x, int y) const {
  return eps_ * std::sqrt((x - xGoal_) * (x - xGoal_) + (y - yGoal_) * (y - yGoal_));
}

inline double GraphSearch::getHeur(int x, int y, int z) const {
  return eps_ * std::sqrt((x - xGoal_) * (x - xGoal_) + (y - yGoal_) * (y - yGoal_) + (z - zGoal_) * (z - zGoal_));
}

bool GraphSearch::plan(int xStart, int yStart, int xGoal, int yGoal, bool useJps, int maxExpand)
{
  use_2d_ = true;
  pq_.clear();
  path_.clear();
  hm_.resize(xDim_ * yDim_);
  seen_.resize(xDim_ * yDim_, false);
  // Set jps
  use_jps_ = useJps;

  // Set goal
  int goal_id = coordToId(xGoal, yGoal);
  xGoal_ = xGoal; yGoal_ = yGoal;

  // Set start node
  int start_id = coordToId(xStart, yStart);
  StatePtr currNode_ptr = std::make_shared<State>(State(start_id, xStart, yStart, 0, 0));
  currNode_ptr->g = 0;
  currNode_ptr->h = getHeur(xStart, yStart);

  return plan(currNode_ptr, maxExpand, start_id, goal_id);
}

bool GraphSearch::plan(int xStart, int yStart, int  zStart, int xGoal, int yGoal, int zGoal, bool useJps, int maxExpand)
{
  use_2d_ = false;
  pq_.clear();
  path_.clear();
  hm_.resize(xDim_ * yDim_ * zDim_);
  seen_.resize(xDim_ * yDim_ * zDim_, false);

  // Set jps
  use_jps_ = useJps;

  // Set goal
  int goal_id = coordToId(xGoal, yGoal, zGoal);
  xGoal_ = xGoal; yGoal_ = yGoal; zGoal_ = zGoal;
  // Set start node
  int start_id = coordToId(xStart, yStart, zStart);
  StatePtr currNode_ptr = std::make_shared<State>(State(start_id, xStart, yStart, zStart, 0, 0, 0));
  currNode_ptr->g = 0;
  currNode_ptr->h = getHeur(xStart, yStart, zStart);

  return plan(currNode_ptr, maxExpand, start_id, goal_id);
}

bool GraphSearch::plan(StatePtr& currNode_ptr, int maxExpand, int start_id, int goal_id) {
  // Insert start node
  currNode_ptr->heapkey = pq_.push(currNode_ptr);
  currNode_ptr->opened = true;
  hm_[currNode_ptr->id] = currNode_ptr;
  seen_[currNode_ptr->id] = true;

  int expand_iteration = 0;
  while(true)
  {
    expand_iteration++;
    // get element with smallest cost
    currNode_ptr = pq_.top(); pq_.pop();
    currNode_ptr->closed = true; // Add to closed list

    if(currNode_ptr->id == goal_id) {
      if(verbose_)
        printf("Goal Reached!!!!!!\n\n");
      break;
    }

    //printf("expand: %d, %d\n", currNode_ptr->x, currNode_ptr->y);
    std::vector<int> succ_ids;
    std::vector<double> succ_costs;
    // Get successors
    if(!use_jps_)
      getSucc(currNode_ptr, succ_ids, succ_costs);
    else
      getJpsSucc(currNode_ptr, succ_ids, succ_costs);

    // if(verbose_)
   // printf("size of succs: %zu\n", succ_ids.size());
    // Process successors
    for( int s = 0; s < (int) succ_ids.size(); s++ )
    {
      //see if we can improve the value of succstate
      StatePtr& child_ptr = hm_[succ_ids[s]];
      double tentative_gval = currNode_ptr->g + succ_costs[s];

      if( tentative_gval < child_ptr->g )
      {
        child_ptr->parentId = currNode_ptr->id;  // Assign new parent
        child_ptr->g = tentative_gval;    // Update gval

        //double fval = child_ptr->g + child_ptr->h;

        // if currently in OPEN, update
        if( child_ptr->opened && !child_ptr->closed) {
          pq_.increase( child_ptr->heapkey );       // update heap
          child_ptr->dx = (child_ptr->x - currNode_ptr->x);
          child_ptr->dy = (child_ptr->y - currNode_ptr->y);
          child_ptr->dz = (child_ptr->z - currNode_ptr->z);
          if(child_ptr->dx != 0)
            child_ptr->dx /= std::abs(child_ptr->dx);
          if(child_ptr->dy != 0)
            child_ptr->dy /= std::abs(child_ptr->dy);
           if(child_ptr->dz != 0)
            child_ptr->dz /= std::abs(child_ptr->dz);
        }
        // if currently in CLOSED
        else if( child_ptr->opened && child_ptr->closed)
        {
          printf("ASTAR ERROR!\n");
        }
        else // new node, add to heap
        {
          //printf("add to open set: %d, %d\n", child_ptr->x, child_ptr->y);
          child_ptr->heapkey = pq_.push(child_ptr);
          child_ptr->opened = true;
        }
      } //
    } // Process successors


    if(maxExpand > 0 && expand_iteration >= maxExpand) {
      if(verbose_)
        printf("MaxExpandStep [%d] Reached!!!!!!\n\n", maxExpand);
      return false;
    }

    if( pq_.empty()) {
      if(verbose_)
        printf("Priority queue is empty!!!!!!\n\n");
      return false;
    }
  }

  if(verbose_) {
    printf("goal g: %f, h: %f!\n", currNode_ptr->g, currNode_ptr->h);
    printf("Expand [%d] nodes!\n", expand_iteration);
  }

  path_ = recoverPath(currNode_ptr, start_id);

  return true;
}


std::vector<StatePtr> GraphSearch::recoverPath(StatePtr node, int start_id) {
  std::vector<StatePtr> path;
  path.push_back(node);
  while(node && node->id != start_id) {
    node = hm_[node->parentId];
    path.push_back(node);
  }

  return path;
}

void GraphSearch::getSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs) {
  if(use_2d_) {
    for(const auto& d: ns_) {
      int new_x = curr->x + d[0];
      int new_y = curr->y + d[1];
      if(!isFree(new_x, new_y))
        continue;

      int new_id = coordToId(new_x, new_y);
      if(!seen_[new_id]) {
        seen_[new_id] = true;
        hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, d[0], d[1]);
        hm_[new_id]->h = getHeur(new_x, new_y);
      }

      succ_ids.push_back(new_id);
      succ_costs.push_back(std::sqrt(d[0]*d[0]+d[1]*d[1]));
    }
  }
  else {
    for(const auto& d: ns_) {
      int new_x = curr->x + d[0];
      int new_y = curr->y + d[1];
      int new_z = curr->z + d[2];
      if(!isFree(new_x, new_y, new_z))
        continue;

      int new_id = coordToId(new_x, new_y, new_z);
      if(!seen_[new_id]) {
        seen_[new_id] = true;
        hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, new_z, d[0], d[1], d[2]);
        hm_[new_id]->h = getHeur(new_x, new_y, new_z);
      }

      succ_ids.push_back(new_id);
      succ_costs.push_back(std::sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]));
    }
  }
}

void GraphSearch::getJpsSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs) {
  if(use_2d_) {
    const int norm1 = std::abs(curr->dx)+std::abs(curr->dy);
    int num_neib = jn2d_->nsz[norm1][0];
    int num_fneib = jn2d_->nsz[norm1][1];
    int id = (curr->dx+1)+3*(curr->dy+1);

    for( int dev = 0; dev < num_neib+num_fneib; ++dev) {
      int new_x, new_y;
      int dx, dy;
      if(dev < num_neib) {
        dx = jn2d_->ns[id][0][dev];
        dy = jn2d_->ns[id][1][dev];
        if(!jump(curr->x, curr->y, dx, dy, new_x, new_y)) continue;
      }
      else {
        int nx = curr->x + jn2d_->f1[id][0][dev-num_neib];
        int ny = curr->y + jn2d_->f1[id][1][dev-num_neib];
        if(isOccupied(nx,ny)) {
          dx = jn2d_->f2[id][0][dev-num_neib];
          dy = jn2d_->f2[id][1][dev-num_neib];
          if(!jump(curr->x, curr->y, dx, dy, new_x, new_y)) continue;
        }
        else
          continue;
      }

      int new_id = coordToId(new_x, new_y);
      if(!seen_[new_id]) {
        seen_[new_id] = true;
        hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, dx, dy);
        hm_[new_id]->h = getHeur(new_x, new_y);
      }

      succ_ids.push_back(new_id);
      succ_costs.push_back(std::sqrt((new_x - curr->x) * (new_x - curr->x) +
            (new_y - curr->y) * (new_y - curr->y)));
    }
  }
  else {
    const int norm1 = std::abs(curr->dx)+std::abs(curr->dy)+std::abs(curr->dz);
    int num_neib = jn3d_->nsz[norm1][0];
    int num_fneib = jn3d_->nsz[norm1][1];
    int id = (curr->dx+1)+3*(curr->dy+1)+9*(curr->dz+1);

    for( int dev = 0; dev < num_neib+num_fneib; ++dev) {
      int new_x, new_y, new_z;
      int dx, dy, dz;
      if(dev < num_neib) {
        dx = jn3d_->ns[id][0][dev];
        dy = jn3d_->ns[id][1][dev];
        dz = jn3d_->ns[id][2][dev];
        if(!jump(curr->x, curr->y, curr->z,
              dx, dy, dz, new_x, new_y, new_z)) continue;
      }
      else {
        int nx = curr->x + jn3d_->f1[id][0][dev-num_neib];
        int ny = curr->y + jn3d_->f1[id][1][dev-num_neib];
        int nz = curr->z + jn3d_->f1[id][2][dev-num_neib];
        if(isOccupied(nx,ny,nz)) {
          dx = jn3d_->f2[id][0][dev-num_neib];
          dy = jn3d_->f2[id][1][dev-num_neib];
          dz = jn3d_->f2[id][2][dev-num_neib];
          if(!jump(curr->x, curr->y, curr->z,
                dx, dy, dz, new_x, new_y, new_z)) continue;
        }
        else
          continue;
      }

      int new_id = coordToId(new_x, new_y, new_z);
      if(!seen_[new_id]) {
        seen_[new_id] = true;
        hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, new_z, dx, dy, dz);
        hm_[new_id]->h = getHeur(new_x, new_y, new_z);
      }

      succ_ids.push_back(new_id);
      succ_costs.push_back(std::sqrt((new_x - curr->x) * (new_x - curr->x) +
            (new_y - curr->y) * (new_y - curr->y) +
            (new_z - curr->z) * (new_z - curr->z)));
    }

  }
}


bool GraphSearch::jump(int x, int y, int dx, int dy, int& new_x, int& new_y ) {
  new_x = x + dx;
  new_y = y + dy;
  if (!isFree(new_x, new_y))
    return false;

  if (new_x ==  xGoal_ && new_y == yGoal_)
    return true;


  if (hasForced(new_x, new_y, dx, dy))
    return true;

  const int id = (dx+1)+3*(dy+1);
  const int norm1 = std::abs(dx) + std::abs(dy);
  int num_neib = jn2d_->nsz[norm1][0];
  for( int k = 0; k < num_neib-1; ++k )
  {
    int new_new_x, new_new_y;
    if(jump(new_x, new_y, jn2d_->ns[id][0][k], jn2d_->ns[id][1][k],
        new_new_x, new_new_y)) return true;
  }

  return jump(new_x, new_y, dx, dy, new_x, new_y);
}


bool GraphSearch::jump(int x, int y, int z, int dx, int dy, int dz, int& new_x, int& new_y, int& new_z) {
  new_x = x + dx;
  new_y = y + dy;
  new_z = z + dz;
  if (!isFree(new_x, new_y, new_z))
    return false;

  if (new_x ==  xGoal_ && new_y == yGoal_ && new_z == zGoal_)
    return true;

  if (hasForced(new_x, new_y, new_z, dx, dy, dz))
    return true;

  const int id = (dx+1)+3*(dy+1)+9*(dz+1);
  const int norm1 = std::abs(dx) + std::abs(dy) +std::abs(dz);
  int num_neib = jn3d_->nsz[norm1][0];
  for( int k = 0; k < num_neib-1; ++k )
  {
    int new_new_x, new_new_y, new_new_z;
    if(jump(new_x,new_y,new_z,
          jn3d_->ns[id][0][k], jn3d_->ns[id][1][k], jn3d_->ns[id][2][k],
        new_new_x, new_new_y, new_new_z)) return true;
  }


  return jump(new_x, new_y, new_z, dx, dy, dz, new_x, new_y, new_z);
}

inline bool GraphSearch::hasForced(int x, int y, int dx, int dy) {
  const int id = (dx+1)+3*(dy+1);
  for( int fn = 0; fn < 2; ++fn )
  {
    int nx = x + jn2d_->f1[id][0][fn];
    int ny = y + jn2d_->f1[id][1][fn];
    if( isOccupied(nx,ny) )
      return true;
  }
  return false;
}


inline bool GraphSearch::hasForced(int x, int y, int z, int dx, int dy, int dz) {
  int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
  int id = (dx+1)+3*(dy+1)+9*(dz+1);
  switch(norm1)
  {
    case 1:
      // 1-d move, check 8 neighbors
      for( int fn = 0; fn < 8; ++fn )
      {
        int nx = x + jn3d_->f1[id][0][fn];
        int ny = y + jn3d_->f1[id][1][fn];
        int nz = z + jn3d_->f1[id][2][fn];
        if( isOccupied(nx,ny,nz) )
          return true;
      }
      return false;
    case 2:
      // 2-d move, check 8 neighbors
      for( int fn = 0; fn < 8; ++fn )
      {
        int nx = x + jn3d_->f1[id][0][fn];
        int ny = y + jn3d_->f1[id][1][fn];
        int nz = z + jn3d_->f1[id][2][fn];
        if( isOccupied(nx,ny,nz) )
          return true;
      }
      return false;
    case 3:
      // 3-d move, check 6 neighbors
      for( int fn = 0; fn < 6; ++fn )
      {
        int nx = x + jn3d_->f1[id][0][fn];
        int ny = y + jn3d_->f1[id][1][fn];
        int nz = z + jn3d_->f1[id][2][fn];
        if( isOccupied(nx,ny,nz) )
          return true;
      }
      return false;
    default:
      return false;
  }
}


std::vector<StatePtr> GraphSearch::getPath() const {
  return path_;
}

std::vector<StatePtr> GraphSearch::getOpenSet() const {
  std::vector<StatePtr> ss;
  for(const auto& it: hm_) {
    if(it && it->opened && !it->closed)
      ss.push_back(it);
  }
  return ss;
}

std::vector<StatePtr> GraphSearch::getCloseSet() const {
  std::vector<StatePtr> ss;
  for(const auto& it: hm_) {
    if(it && it->closed)
      ss.push_back(it);
  }
  return ss;
}


std::vector<StatePtr> GraphSearch::getAllSet() const {
  std::vector<StatePtr> ss;
  for(const auto& it: hm_) {
    if(it)
      ss.push_back(it);
  }
  return ss;
}

constexpr int JPS2DNeib::nsz[3][2];

JPS2DNeib::JPS2DNeib() {
  int id = 0;
  for(int dy = -1; dy <= 1; ++dy) {
    for(int dx = -1; dx <= 1; ++dx) {
      int norm1 = std::abs(dx) + std::abs(dy);
      for(int dev = 0; dev < nsz[norm1][0]; ++dev)
        Neib(dx,dy,norm1,dev, ns[id][0][dev], ns[id][1][dev]);
      for(int dev = 0; dev < nsz[norm1][1]; ++dev)
      {
        FNeib(dx,dy,norm1,dev,
            f1[id][0][dev],f1[id][1][dev],
            f2[id][0][dev],f2[id][1][dev]);
      }
      id ++;
    }
  }
}

void JPS2DNeib::print() {
  for(int dx = -1; dx <= 1; dx++) {
    for(int dy = -1; dy <= 1; dy++) {
      int id = (dx+1)+3*(dy+1);
      printf("[dx: %d, dy: %d]-->id: %d:\n", dx, dy, id);
      for(unsigned int i = 0; i < sizeof(f1[id][0])/sizeof(f1[id][0][0]); i++)
        printf("                f1: [%d, %d]\n", f1[id][0][i], f1[id][1][i]);
    }
  }
}

void JPS2DNeib::Neib(int dx, int dy, int norm1, int dev, int& tx, int& ty)
{
  switch(norm1)
  {
    case 0:
      switch(dev)
      {
        case 0: tx=1; ty=0; return;
        case 1: tx=-1; ty=0; return;
        case 2: tx=0; ty=1; return;
        case 3: tx=1; ty=1; return;
        case 4: tx=-1; ty=1; return;
        case 5: tx=0; ty=-1; return;
        case 6: tx=1; ty=-1; return;
        case 7: tx=-1; ty=-1; return;
     }
    case 1:
      tx = dx; ty = dy; return;
    case 2:
      switch(dev)
      {
        case 0: tx = dx; ty = 0; return;
        case 1: tx = 0; ty = dy; return;
        case 2: tx = dx; ty = dy; return;
      }
  }
}

void JPS2DNeib::FNeib( int dx, int dy, int norm1, int dev,
                       int& fx, int& fy, int& nx, int& ny)
{
  switch(norm1)
  {
    case 1:
      switch(dev)
      {
        case 0: fx= 0; fy= 1; break;
        case 1: fx= 0; fy= -1;  break;
      }

      // switch order if different direction
      if(dx == 0)
        fx = fy, fy = 0;

      nx = dx + fx; ny = dy + fy;
      return;
    case 2:
      switch(dev)
      {
        case 0:
          fx = -dx; fy = 0;
          nx = -dx; ny = dy;
          return;
        case 1:
          fx = 0; fy = -dy;
          nx = dx; ny = -dy;
          return;
      }
  }
}

constexpr int JPS3DNeib::nsz[4][2];

JPS3DNeib::JPS3DNeib() {
  int id = 0;
  for(int dz = -1; dz <= 1; ++dz) {
    for(int dy = -1; dy <= 1; ++dy) {
      for(int dx = -1; dx <= 1; ++dx) {
        int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
        for(int dev = 0; dev < nsz[norm1][0]; ++dev)
          Neib(dx,dy,dz,norm1,dev,
              ns[id][0][dev], ns[id][1][dev], ns[id][2][dev]);
        for(int dev = 0; dev < nsz[norm1][1]; ++dev)
        {
          FNeib(dx,dy,dz,norm1,dev,
              f1[id][0][dev],f1[id][1][dev], f1[id][2][dev],
              f2[id][0][dev],f2[id][1][dev], f2[id][2][dev]);
        }
        id ++;
      }
    }
  }
}


void JPS3DNeib::Neib(int dx, int dy, int dz, int norm1, int dev,
    int& tx, int& ty, int& tz)
{
  switch(norm1)
  {
    case 0:
      switch(dev)
      {
        case 0: tx=1; ty=0; tz=0; return;
        case 1: tx=-1; ty=0; tz=0; return;
        case 2: tx=0; ty=1; tz=0; return;
        case 3: tx=1; ty=1; tz=0; return;
        case 4: tx=-1; ty=1; tz=0; return;
        case 5: tx=0; ty=-1; tz=0; return;
        case 6: tx=1; ty=-1; tz=0; return;
        case 7: tx=-1; ty=-1; tz=0; return;
        case 8: tx=0; ty=0; tz=1; return;
        case 9: tx=1; ty=0; tz=1; return;
        case 10: tx=-1; ty=0; tz=1; return;
        case 11: tx=0; ty=1; tz=1; return;
        case 12: tx=1; ty=1; tz=1; return;
        case 13: tx=-1; ty=1; tz=1; return;
        case 14: tx=0; ty=-1; tz=1; return;
        case 15: tx=1; ty=-1; tz=1; return;
        case 16: tx=-1; ty=-1; tz=1; return;
        case 17: tx=0; ty=0; tz=-1; return;
        case 18: tx=1; ty=0; tz=-1; return;
        case 19: tx=-1; ty=0; tz=-1; return;
        case 20: tx=0; ty=1; tz=-1; return;
        case 21: tx=1; ty=1; tz=-1; return;
        case 22: tx=-1; ty=1; tz=-1; return;
        case 23: tx=0; ty=-1; tz=-1; return;
        case 24: tx=1; ty=-1; tz=-1; return;
        case 25: tx=-1; ty=-1; tz=-1; return;
      }
    case 1:
      tx = dx; ty = dy; tz = dz; return;
    case 2:
      switch(dev)
      {
        case 0:
          if(dz == 0){
            tx = 0; ty = dy; tz = 0; return;
          }else{
            tx = 0; ty = 0; tz = dz; return;
          }
        case 1:
          if(dx == 0){
            tx = 0; ty = dy; tz = 0; return;
          }else{
            tx = dx; ty = 0; tz = 0; return;
          }
        case 2:
          tx = dx; ty = dy; tz = dz; return;
      }
    case 3:
      switch(dev)
      {
        case 0: tx = dx; ty =  0; tz =  0; return;
        case 1: tx =  0; ty = dy; tz =  0; return;
        case 2: tx =  0; ty =  0; tz = dz; return;
        case 3: tx = dx; ty = dy; tz =  0; return;
        case 4: tx = dx; ty =  0; tz = dz; return;
        case 5: tx =  0; ty = dy; tz = dz; return;
        case 6: tx = dx; ty = dy; tz = dz; return;
      }
  }
}

void JPS3DNeib::FNeib( int dx, int dy, int dz, int norm1, int dev,
                          int& fx, int& fy, int& fz,
                          int& nx, int& ny, int& nz)
{
  switch(norm1)
  {
    case 1:
      switch(dev)
      {
        case 0: fx= 0; fy= 1; fz = 0; break;
        case 1: fx= 0; fy=-1; fz = 0; break;
        case 2: fx= 1; fy= 0; fz = 0; break;
        case 3: fx= 1; fy= 1; fz = 0; break;
        case 4: fx= 1; fy=-1; fz = 0; break;
        case 5: fx=-1; fy= 0; fz = 0; break;
        case 6: fx=-1; fy= 1; fz = 0; break;
        case 7: fx=-1; fy=-1; fz = 0; break;
      }
      nx = fx; ny = fy; nz = dz;
      // switch order if different direction
      if(dx != 0){
        fz = fx; fx = 0;
        nz = fz; nx = dx;
      }if(dy != 0){
        fz = fy; fy = 0;
        nz = fz; ny = dy;
      }
      return;
    case 2:
      if(dx == 0){
        switch(dev)
        {
          case 0:
            fx = 0; fy = 0; fz = -dz;
            nx = 0; ny = dy; nz = -dz;
            return;
          case 1:
            fx = 0; fy = -dy; fz = 0;
            nx = 0; ny = -dy; nz = dz;
            return;
          case 2:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = dy; nz = dz;
            return;
          case 3:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = dy; nz = dz;
            return;
          case 4:
            fx = 1; fy = 0; fz = -dz;
            nx = 1; ny = dy; nz = -dz;
            return;
          case 5:
            fx = 1; fy = -dy; fz = 0;
            nx = 1; ny = -dy; nz = dz;
            return;
          case 6:
            fx = -1; fy = 0; fz = -dz;
            nx = -1; ny = dy; nz = -dz;
            return;
          case 7:
            fx = -1; fy = -dy; fz = 0;
            nx = -1; ny = -dy; nz = dz;
            return;
          // Extras
          case 8:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = dy; nz = 0;
            return;
          case 9:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = 0; nz = dz;
            return;
          case 10:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = dy; nz = 0;
            return;
          case 11:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = 0; nz = dz;
            return;
        }
      }else if(dy == 0){
        switch(dev)
        {
          case 0:
            fx = 0; fy = 0; fz = -dz;
            nx = dx; ny = 0; nz = -dz;
            return;
          case 1:
            fx = -dx; fy = 0; fz = 0;
            nx = -dx; ny = 0; nz = dz;
            return;
          case 2:
            fx = 0; fy = 1; fz = 0;
            nx = dx; ny = 1; nz = dz;
            return;
          case 3:
            fx = 0; fy = -1; fz = 0;
            nx = dx; ny = -1;nz = dz;
            return;
          case 4:
            fx = 0; fy = 1; fz = -dz;
            nx = dx; ny = 1; nz = -dz;
            return;
          case 5:
            fx = -dx; fy = 1; fz = 0;
            nx = -dx; ny = 1; nz = dz;
            return;
          case 6:
            fx = 0; fy = -1; fz = -dz;
            nx = dx; ny = -1; nz = -dz;
            return;
          case 7:
            fx = -dx; fy = -1; fz = 0;
            nx = -dx; ny = -1; nz = dz;
            return;
          // Extras
          case 8:
            fx = 0; fy = 1; fz = 0;
            nx = dx; ny = 1; nz = 0;
            return;
          case 9:
            fx = 0; fy = 1; fz = 0;
            nx = 0; ny = 1; nz = dz;
            return;
          case 10:
            fx = 0; fy = -1; fz = 0;
            nx = dx; ny = -1; nz = 0;
            return;
          case 11:
            fx = 0; fy = -1; fz = 0;
            nx = 0; ny = -1; nz = dz;
            return;
        }
      }else{// dz==0
        switch(dev)
        {
          case 0:
            fx = 0; fy = -dy; fz = 0;
            nx = dx; ny = -dy; nz = 0;
            return;
          case 1:
            fx = -dx; fy = 0; fz = 0;
            nx = -dx; ny = dy; nz = 0;
            return;
          case 2:
            fx =  0; fy = 0; fz = 1;
            nx = dx; ny = dy; nz = 1;
            return;
          case 3:
            fx =  0; fy = 0; fz = -1;
            nx = dx; ny = dy; nz = -1;
            return;
          case 4:
            fx = 0; fy = -dy; fz = 1;
            nx = dx; ny = -dy; nz = 1;
            return;
          case 5:
            fx = -dx; fy = 0; fz = 1;
            nx = -dx; ny = dy; nz = 1;
            return;
          case 6:
            fx = 0; fy = -dy; fz = -1;
            nx = dx; ny = -dy; nz = -1;
            return;
          case 7:
            fx = -dx; fy = 0; fz = -1;
            nx = -dx; ny = dy; nz = -1;
            return;
          // Extras
          case 8:
            fx =  0; fy = 0; fz = 1;
            nx = dx; ny = 0; nz = 1;
            return;
          case 9:
            fx = 0; fy = 0; fz = 1;
            nx = 0; ny = dy; nz = 1;
            return;
          case 10:
            fx =  0; fy = 0; fz = -1;
            nx = dx; ny = 0; nz = -1;
            return;
          case 11:
            fx = 0; fy = 0; fz = -1;
            nx = 0; ny = dy; nz = -1;
            return;
        }
      }
    case 3:
      switch(dev)
      {
        case 0:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = dy; nz = dz;
          return;
        case 1:
          fx = 0; fy = -dy; fz = 0;
          nx = dx; ny = -dy; nz = dz;
          return;
        case 2:
          fx = 0; fy = 0; fz = -dz;
          nx = dx; ny = dy; nz = -dz;
          return;
        // Need to check up to here for forced!
        case 3:
          fx = 0; fy = -dy; fz = -dz;
          nx = dx; ny = -dy; nz = -dz;
          return;
        case 4:
          fx = -dx; fy = 0; fz = -dz;
          nx = -dx; ny = dy; nz = -dz;
          return;
        case 5:
          fx = -dx; fy = -dy; fz = 0;
          nx = -dx; ny = -dy; nz = dz;
          return;
        // Extras
        case 6:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = 0; nz = dz;
          return;
        case 7:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = dy; nz = 0;
          return;
        case 8:
          fx = 0; fy = -dy; fz = 0;
          nx = 0; ny = -dy; nz = dz;
          return;
        case 9:
          fx = 0; fy = -dy; fz = 0;
          nx = dx; ny = -dy; nz = 0;
          return;
        case 10:
          fx = 0; fy = 0; fz = -dz;
          nx = 0; ny = dy; nz = -dz;
          return;
        case 11:
          fx = 0; fy = 0; fz = -dz;
          nx = dx; ny = 0; nz = -dz;
          return;
      }
  }
}





