#include "faster.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/StdVector>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <assert.h>
#include <stdlib.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>

using namespace JPS;
using namespace termcolor;

// Uncomment only one to choose the type of timer you want:
typedef ROSTimer MyTimer;
// typedef ROSWallTimer MyTimer;
// typedef Timer MyTimer;

Faster::Faster(parameters par) : par_(par)
{
  // flight_mode_ = flight_mode_.NOT_FLYING;
  flight_mode_.mode = GO;  // TODO (changed for the jackal)

  // mtx_G.lock();
  G_.pos << 0, 0, 0;
  // mtx_G.unlock();
  G_term_.pos << 0, 0, 0;

  mtx_initial_cond.lock();
  stateA_.setZero();
  mtx_initial_cond.unlock();

  // log_.total_dist = 0;

  // Setup of jps_manager
  std::cout << "par_.wdx / par_.res =" << par_.wdx / par_.res << std::endl;
  jps_manager_.setNumCells((int)par_.wdx / par_.res, (int)par_.wdy / par_.res, (int)par_.wdz / par_.res);
  jps_manager_.setFactorJPS(par_.factor_jps);
  jps_manager_.setResolution(par_.res);
  jps_manager_.setInflationJPS(par_.inflation_jps);
  jps_manager_.setZGroundAndZMax(par_.z_ground, par_.z_max);
  // jps_manager_.setVisual(par_.visual);
  jps_manager_.setDroneRadius(par_.drone_radius);

  double max_values[3] = { par_.v_max, par_.a_max, par_.j_max };

  // Setup of sg_whole_
  sg_whole_.setN(par_.N_whole);
  sg_whole_.createVars();
  sg_whole_.setDC(par_.dc);
  sg_whole_.set_max(max_values);
  sg_whole_.setMode(WHOLE_TRAJ);
  sg_whole_.setForceFinalConstraint(true);
  sg_whole_.setFactorInitialAndFinalAndIncrement(1, 10, par_.increment_whole);
  sg_whole_.setVerbose(par_.gurobi_verbose);
  sg_whole_.setThreads(par_.gurobi_threads);
  sg_whole_.setWMax(par_.w_max);

  // Setup of sg_safe_
  sg_safe_.setN(par_.N_safe);
  sg_safe_.createVars();
  sg_safe_.setDC(par_.dc);
  sg_safe_.set_max(max_values);
  sg_safe_.setMode(WHOLE_TRAJ);  // SAFE_PATH
  sg_safe_.setForceFinalConstraint(false);
  sg_safe_.setFactorInitialAndFinalAndIncrement(1, 10, par_.increment_safe);
  sg_safe_.setVerbose(par_.gurobi_verbose);
  sg_safe_.setThreads(par_.gurobi_threads);
  sg_safe_.setWMax(par_.w_max);

  pclptr_unk_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pclptr_map_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

/*void Faster::yaw(double diff, acl_msgs::QuadGoal& quad_goal)
{
  saturate(diff, -par_.dc * par_.w_max, par_.dc * par_.w_max);
  double dyaw_not_filtered;

  dyaw_not_filtered = copysign(1, diff) * par_.w_max;

  dyaw_filtered_ = (1 - par_.alpha_filter_dyaw) * dyaw_not_filtered + par_.alpha_filter_dyaw * dyaw_filtered_;
  quad_goal.dyaw = dyaw_filtered_;

  quad_goal.yaw += dyaw_filtered_ * par_.dc;
}*/

void Faster::createMoreVertexes(vec_Vecf<3>& path, double d)
{
  for (int j = 0; j < path.size() - 1; j++)
  {
    double dist = (path[j + 1] - path[j]).norm();
    int vertexes_to_add = floor(dist / d);
    Eigen::Vector3d v = (path[j + 1] - path[j]).normalized();
    // std::cout << "Vertexes to add=" << vertexes_to_add << std::endl;
    if (dist > d)
    {
      for (int i = 0; i < vertexes_to_add; i++)
      {
        path.insert(path.begin() + j + 1, path[j] + v * d);
        j = j + 1;
      }
    }
  }
}

void Faster::updateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map, pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_unk)
{
  mtx_map.lock();
  mtx_unk.lock();

  // TODO
  pclptr_map_ = pclptr_map;
  pclptr_unk_ = pclptr_unk;

  std::cout << "state_.pos=" << state_.pos << std::endl;

  jps_manager_.updateJPSMap(pclptr_map_, state_.pos);  // Update even where there are no points

  if (pclptr_map_->width != 0 && pclptr_map_->height != 0)  // Point Cloud is not empty
  {
    kdtree_map_.setInputCloud(pclptr_map_);
    kdtree_map_initialized_ = 1;
    jps_manager_.vec_o_ = pclptr_to_vec(pclptr_map_);
  }
  else
  {
    std::cout << "Occupancy Grid received is empty, maybe map is too small?" << std::endl;
  }

  if (pclptr_unk_->points.size() == 0)
  {
    std::cout << "Unkown cloud has 0 points" << std::endl;
    return;
  }
  else
  {
    kdtree_unk_.setInputCloud(pclptr_unk_);
    kdtree_unk_initialized_ = 1;
    jps_manager_.vec_uo_ = pclptr_to_vec(pclptr_unk_);  // insert unknown space
    jps_manager_.vec_uo_.insert(jps_manager_.vec_uo_.end(), jps_manager_.vec_o_.begin(),
                                jps_manager_.vec_o_.end());  // append known space
  }

  mtx_map.unlock();
  mtx_unk.unlock();
}

void Faster::setTerminalGoal(state& term_goal)
{
  mtx_G_term.lock();
  mtx_G.lock();
  mtx_state.lock();
  mtx_planner_status_.lock();

  G_term_.pos = term_goal.pos;
  Eigen::Vector3d temp = state_.pos;
  G_.pos = projectPointToBox(temp, G_term_.pos, par_.wdx, par_.wdy, par_.wdz);
  changeDroneStatus(DroneStatus::TRAVELING);  // TODO: This should be YAWING

  terminal_goal_initialized_ = true;

  mtx_state.unlock();
  mtx_G.unlock();
  mtx_G_term.unlock();
  mtx_planner_status_.unlock();
}

void Faster::getG(state& G)
{
  G = G_;
}

void Faster::getState(state& data)
{
  mtx_state.lock();
  data = state_;
  mtx_state.unlock();
}

int Faster::findIndexR(int indexH)
{
  // Ignore z to obtain this heuristics (if not it can become VERY conservative)
  // mtx_X_U_temp.lock();
  Eigen::Vector2d posHk;
  posHk << sg_whole_.X_temp_[indexH].pos(0), sg_whole_.X_temp_[indexH].pos(1);
  int indexR = indexH;

  std::cout << "Here5" << std::endl;

  for (int i = 0; i <= indexH; i = i + 1)  // Loop from A to H
  {
    Eigen::Vector2d vel;
    vel << sg_whole_.X_temp_[i].vel(0), sg_whole_.X_temp_[i].vel(1);  //(i, 3), sg_whole_.X_temp_(i, 4);

    Eigen::Vector2d pos;
    pos << sg_whole_.X_temp_[i].pos(0), sg_whole_.X_temp_[i].pos(1);

    Eigen::Vector2d braking_distance =
        (vel.array() * (posHk - pos).array()).sign() * vel.array().square() / (2 * par_.delta_a * par_.a_max);

    // std::cout << "braking_distance=" << braking_distance.transpose() << std::endl;
    // std::cout << "(posHk - pos).cwiseAbs().array())=" << (posHk - pos).cwiseAbs().array().transpose() << std::endl;

    bool thereWillBeCollision =
        (braking_distance.array() > (posHk - pos).cwiseAbs().array()).any();  // Any of the braking distances (in x, y,
                                                                              // z) is bigger than the distance to the
                                                                              // obstacle in that direction
    if (thereWillBeCollision)
    {
      indexR = i;

      if (indexR == 0)
      {
        std::cout << bold << red << "R was taken in A" << reset << std::endl;
      }

      break;
    }
  }
  std::cout << red << bold << "indexR=" << indexR << " /" << sg_whole_.X_temp_.size() - 1 << reset << std::endl;
  // std::cout << red << bold << "indexH=" << indexH << " /" << sg_whole_.X_temp_.rows() - 1 << reset << std::endl;
  // mtx_X_U_temp.unlock();

  return indexR;
}

int Faster::findIndexH(bool& needToComputeSafePath)
{
  int n = 1;  // find one neighbour
  std::vector<int> pointIdxNKNSearch(n);
  std::vector<float> pointNKNSquaredDistance(n);

  needToComputeSafePath = false;

  mtx_unk.lock();
  mtx_X_U_temp.lock();
  int indexH = sg_whole_.X_temp_.size() - 1;

  for (int i = 0; i < sg_whole_.X_temp_.size(); i = i + 10)
  {  // Sample points along the trajectory

    Eigen::Vector3d tmp = sg_whole_.X_temp_[i].pos;
    pcl::PointXYZ searchPoint(tmp(0), tmp(1), tmp(2));

    if (kdtree_unk_.nearestKSearch(searchPoint, n, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
      if (sqrt(pointNKNSquaredDistance[0]) < par_.drone_radius)
      {
        needToComputeSafePath = true;  // There is intersection, so there is need to compute rescue path
        indexH = (int)(par_.delta_H * i);
        break;
      }
    }
  }
  std::cout << red << bold << "indexH=" << indexH << " /" << sg_whole_.X_temp_.size() - 1 << reset << std::endl;
  mtx_unk.unlock();
  mtx_X_U_temp.unlock();

  return indexH;
}

bool Faster::ARisInFreeSpace(int index)
{  // We have to check only against the unkown space (A-R won't intersect the obstacles for sure)

  // std::cout << "In ARisInFreeSpace, radius_drone= " << par_.drone_radius << std::endl;
  int n = 1;  // find one neighbour

  std::vector<int> pointIdxNKNSearch(n);
  std::vector<float> pointNKNSquaredDistance(n);

  bool isFree = true;

  // std::cout << "Before mtx_unk" << std::endl;
  mtx_unk.lock();
  mtx_X_U_temp.lock();
  // std::cout << "After mtx_unk. index=" << index << std::endl;
  for (int i = 0; i < index; i = i + 10)
  {  // Sample points along the trajectory
     // std::cout << "i=" << i << std::endl;
    Eigen::Vector3d tmp = sg_whole_.X_temp_[i].pos;
    pcl::PointXYZ searchPoint(tmp(0), tmp(1), tmp(2));

    if (kdtree_unk_.nearestKSearch(searchPoint, n, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
      if (sqrt(pointNKNSquaredDistance[0]) < 0.2)
      {  // TODO: 0.2 is the radius of the drone.
        std::cout << "A->R collides, with d=" << sqrt(pointNKNSquaredDistance[0])
                  << ", radius_drone=" << par_.drone_radius << std::endl;
        isFree = false;
        break;
      }
    }
  }

  mtx_unk.unlock();
  mtx_X_U_temp.unlock();

  return isFree;
}

// Returns the first collision of JPS with the map (i.e. with the known obstacles). Note that JPS will collide with a
// map B if JPS was computed using an older map A
// If type_return==Intersection, it returns the last point in the JPS path that is at least par_.inflation_jps from map
Eigen::Vector3d Faster::getFirstCollisionJPS(vec_Vecf<3>& path, bool* thereIsIntersection, int map, int type_return)
{
  vec_Vecf<3> original = path;

  Eigen::Vector3d first_element = path[0];
  Eigen::Vector3d last_search_point = path[0];
  Eigen::Vector3d inters = path[0];
  pcl::PointXYZ pcl_search_point = eigenPoint2pclPoint(path[0]);

  Eigen::Vector3d result;

  // occupied (map)
  int n = 1;
  std::vector<int> id_map(n);
  std::vector<float> dist2_map(n);  // squared distance
  double r = 1000000;
  // printElementsOfJPS(path);
  // printf("In 2\n");

  mtx_map.lock();
  mtx_unk.lock();

  // Find the next eig_search_point
  int last_id = -1;  // this is the last index inside the sphere
  int iteration = 0;
  while (path.size() > 0)
  {
    // std::cout<<red<<"New Iteration, iteration="<<iteration<<reset<<std::endl;
    // std::cout << red << "Searching from point=" << path[0].transpose() << reset << std::endl;
    pcl_search_point = eigenPoint2pclPoint(path[0]);

    int number_of_neigh;

    if (map == MAP)
    {
      number_of_neigh = kdtree_map_.nearestKSearch(pcl_search_point, n, id_map, dist2_map);
    }
    else  // map == UNKNOWN_MAP
    {
      number_of_neigh = kdtree_unk_.nearestKSearch(pcl_search_point, n, id_map, dist2_map);
      // std::cout << "In unknown_map, number of neig=" << number_of_neigh << std::endl;
    }
    // printf("************NearestSearch: TotalTime= %0.2f ms\n", 1000 * (ros::Time::now().toSec() - before));

    if (number_of_neigh > 0)
    {
      r = sqrt(dist2_map[0]);

      // std::cout << "r=" << r << std::endl;
      // std::cout << "Point=" << r << std::endl;

      if (r < par_.drone_radius)  // collision of the JPS path and an inflated obstacle --> take last search point
      {
        // std::cout << "Collision detected" << std::endl;  // We will return the search_point
        // pubJPSIntersection(inters);
        // inters = path[0];  // path[0] is the search_point I'm using.
        if (iteration == 0)
        {
          std::cout << red << bold << "The first point is in collision --> Hacking" << reset << std::endl;
        }
        switch (type_return)
        {
          case RETURN_LAST_VERTEX:
            result = last_search_point;
            break;
          case RETURN_INTERSECTION:
            if (iteration == 0)
            {  // Hacking (TODO)
              Eigen::Vector3d tmp;
              tmp << original[0](0) + 0.01, original[0](1), original[0](2);
              path.clear();
              path.push_back(original[0]);
              path.push_back(tmp);
              result = path[path.size() - 1];
              // result=original[original.size() - 1];
            }
            else
            {
              // std::cout << "In Return Intersection, last_id=" << last_id<<el_eliminated<< std::endl;
              int vertexes_eliminated_tmp = original.size() - path.size() + 1;
              // std::cout << "In Return Intersection, vertexes_eliminated_tmp=" << vertexes_eliminated_tmp <<
              // std::endl;
              original.erase(original.begin() + vertexes_eliminated_tmp,
                             original.end());  // Now original contains all the elements eliminated
              original.push_back(path[0]);

              /*              std::cout << "Result before reduceJPSbyDistance" << original[original.size() -
                 1].transpose()
                                      << std::endl;*/

              // This is to force the intersection point to be at least par_.drone_radius away from the obstacles
              reduceJPSbyDistance(original, par_.drone_radius);

              result = original[original.size() - 1];

              // std::cout<<"Result here is"<<result.transpose()<<std::endl;

              path = original;
            }
            // Copy the resulting path to the reference
            /*     std::reverse(original.begin(), original.end());  // flip all the vector
               result = getFirstIntersectionWithSphere(original, par_.inflation_jps, original[0]);*/
            break;
        }

        *thereIsIntersection = true;

        break;  // Leave the while loop
      }

      bool no_points_outside_sphere = false;

      inters = getFirstIntersectionWithSphere(path, r, path[0], &last_id, &no_points_outside_sphere);
      // printf("**********Found it*****************\n");
      if (no_points_outside_sphere == true)
      {  // JPS doesn't intersect with any obstacle
        *thereIsIntersection = false;
        /*        std::cout << "JPS provided doesn't intersect any obstacles, returning the first element of the path
           you gave " "me\n"
                          << std::endl;*/
        result = first_element;

        if (type_return == RETURN_INTERSECTION)
        {
          result = original[original.size() - 1];
          path = original;
        }

        break;  // Leave the while loop
      }
      // printf("In 4\n");

      last_search_point = path[0];
      // Remove all the points of the path whose id is <= to last_id:
      path.erase(path.begin(), path.begin() + last_id + 1);

      // and add the intersection as the first point of the path
      path.insert(path.begin(), inters);
    }
    else
    {  // There is no neighbours
      *thereIsIntersection = false;
      ROS_INFO("JPS provided doesn't intersect any obstacles, returning the first element of the path you gave me\n");
      result = first_element;

      if (type_return == RETURN_INTERSECTION)
      {
        result = original[original.size() - 1];
        path = original;
      }

      break;
    }
    iteration = iteration + 1;
  }
  mtx_map.unlock();
  mtx_unk.unlock();

  return result;
}

void Faster::changeMode(int new_mode)
{
  flight_mode_.mode = new_mode;

  /*  if (flight_mode_.mode == LAND)  //&& flight_mode_ != flight_mode_.LAND
    {
      printf("LANDING\n");

      mtx_goals.lock();
      mtx_state.lock();

      state x0 = state_;
      state xf = state_;
      xf.pos[2] = par_.z_land;

      mtx_state.unlock();
      mtx_goals.unlock();

      sg_whole_.setXf(xf);
      sg_whole_.setX0(x0);
      std::vector<LinearConstraint3D> l_constraints_empty;
      sg_whole_.setPolytopes(l_constraints_empty);
      bool solved_landing = false;
      solved_landing = sg_whole_.genNewTraj();

      if (solved_landing == false)
      {
        std::cout << bold << red << "No solution for landing" << reset << std ::endl;
      }
      else
      {
        std::cout << "solution found" << std::endl;
        sg_whole_.fillXandU();
        to_land_ = true;
        mtx_X_U_temp.lock();
        X_temp_ = sg_whole_.X_temp_;
        mtx_X_U_temp.unlock();
        mtx_planner_status_.lock();
        planner_status_ = REPLANNED;
        mtx_planner_status_.unlock();
      }
    }*/
}

void Faster::updateState(state data)
{
  state_ = data;

  if (state_initialized_ == false)
  {
    plan_.push_back(state_);
  }

  state_initialized_ = true;
}

bool Faster::initialized()
{
  if (!state_initialized_ || !kdtree_map_initialized_ || !kdtree_unk_initialized_ || !terminal_goal_initialized_)
  {
    std::cout << red << bold << ("Waiting to initialize kdTree_map and/or kdTree_unk and/or goal_click and/or state_")
              << reset << std::endl;

    std::cout << "state_initialized_= " << state_initialized_ << std::endl;
    std::cout << "kdtree_map_initialized_= " << kdtree_map_initialized_ << std::endl;
    std::cout << "kdtree_unk_initialized_= " << kdtree_unk_initialized_ << std::endl;
    std::cout << "terminal_goal_initialized_= " << terminal_goal_initialized_ << std::endl;
    return false;
  }
  return true;
}

void Faster::replan(vec_Vecf<3>& JPS_safe_out, vec_Vecf<3>& JPS_whole_out, vec_E<Polyhedron<3>>& poly_safe_out,
                    vec_E<Polyhedron<3>>& poly_whole_out, std::vector<state>& X_safe_out,
                    std::vector<state>& X_whole_out)
{
  MyTimer replanCB_t(true);

  if (initialized() == false)
  {
    return;
  }

  sg_whole_.ResetToNormalState();
  sg_safe_.ResetToNormalState();

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// G <-- Project GTerm ////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  mtx_state.lock();
  mtx_G.lock();
  mtx_G_term.lock();

  state state_local = state_;
  state G;
  G.pos = projectPointToBox(state_local.pos, G_term_.pos, par_.wdx, par_.wdy, par_.wdz);
  state G_term = G_term_;  // Local copy of the terminal terminal goal

  mtx_G.unlock();
  mtx_G_term.unlock();
  mtx_state.unlock();

  // Check if we have reached the goal
  double dist_to_goal = (G_term.pos - state_local.pos).norm();
  if (dist_to_goal < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_REACHED);
  }
  // Don't plan if drone is not traveling
  if (drone_status_ == DroneStatus::GOAL_REACHED || (drone_status_ == DroneStatus::YAWING))
  {
    std::cout << "No replanning needed because" << std::endl;
    print_status();
    return;
  }

  std::cout << bold << on_red << "************IN REPLAN CB*********" << reset << std::endl;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Select state A /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  state A;
  int k_safe, k_end_whole;

  // If k_end_whole=0, then A = plan_.back() = plan_[plan_.size() - 1]
  k_end_whole = std::max((int)plan_.size() - deltaT_, 0);
  A = plan_[plan_.size() - 1 - k_end_whole];

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Solve JPS //////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  bool solvedjps = false;
  MyTimer timer_jps(true);

  vec_Vecf<3> JPSk = jps_manager_.solveJPS3D(A.pos, G.pos, &solvedjps, 1);

  if (solvedjps == false)
  {
    std::cout << bold << red << "JPS didn't find a solution" << std::endl;
    return;
  }

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Find JPS_in ////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  double ra = std::min((dist_to_goal - 0.001), par_.Ra_max);  // radius of the sphere S
  bool noPointsOutsideS;
  int li1;  // last index inside the sphere of JPSk
  state E;
  E.pos = getFirstIntersectionWithSphere(JPSk, ra, JPSk[0], &li1, &noPointsOutsideS);
  vec_Vecf<3> JPS_in(JPSk.begin(), JPSk.begin() + li1 + 1);
  if (noPointsOutsideS == false)
  {
    JPS_in.push_back(E.pos);
  }
  // createMoreVertexes in case dist between vertexes is too big
  createMoreVertexes(JPS_in, par_.dist_max_vertexes);

  //////////////////////////////////////////////////////////////////////////
  ///////////////// Solve with GUROBI Whole trajectory /////////////////////
  //////////////////////////////////////////////////////////////////////////

  if (par_.use_faster == true)
  {
    vec_Vecf<3> JPS_whole = JPS_in;
    deleteVertexes(JPS_whole, par_.max_poly_whole);
    E.pos = JPS_whole[JPS_whole.size() - 1];

    // Convex Decomp around JPS_whole
    MyTimer cvx_ellip_decomp_t(true);
    jps_manager_.cvxEllipsoidDecomp(JPS_whole, OCCUPIED_SPACE, l_constraints_whole_, poly_whole_out);
    std::cout << "poly_whole_out= " << poly_whole_out.size() << std::endl;

    // Check if G is inside poly_whole
    bool isGinside_whole = l_constraints_whole_[l_constraints_whole_.size() - 1].inside(G.pos);
    E.pos = (isGinside_whole == true) ? G.pos : E.pos;

    // Set Initial cond, Final cond, and polytopes for the whole traj
    sg_whole_.setX0(A);
    sg_whole_.setXf(E);
    sg_whole_.setPolytopes(l_constraints_whole_);

    std::cout << "Initial Position is inside= " << l_constraints_whole_[l_constraints_whole_.size() - 1].inside(A.pos)
              << std::endl;
    std::cout << "Final Position is inside= " << l_constraints_whole_[l_constraints_whole_.size() - 1].inside(E.pos)
              << std::endl;

    // Solve with Gurobi
    MyTimer whole_gurobi_t(true);
    bool solved_whole = sg_whole_.genNewTraj();

    if (solved_whole == false)
    {
      std::cout << bold << red << "No solution found for the whole trajectory" << reset << std::endl;
      return;
    }

    // Get Results
    sg_whole_.fillXandU();

    // Copy for visualization
    X_whole_out = sg_whole_.X_temp_;
    JPS_whole_out = JPS_whole;
  }
  else
  {  // Dummy whole trajectory
    state dummy;
    std::vector<state> dummy_vector;
    dummy_vector.push_back(dummy);
    sg_whole_.X_temp_ = dummy_vector;
  }

  std::cout << "This is the WHOLE TRAJECTORY" << std::endl;
  printStateVector(sg_whole_.X_temp_);
  std::cout << "===========================" << std::endl;

  //////////////////////////////////////////////////////////////////////////
  ///////////////// Solve with GUROBI Safe trajectory /////////////////////
  //////////////////////////////////////////////////////////////////////////

  vec_Vecf<3> JPSk_inside_sphere_tmp = JPS_in;
  bool thereIsIntersection2;
  state M;
  M.pos = getFirstCollisionJPS(JPSk_inside_sphere_tmp, &thereIsIntersection2, UNKNOWN_MAP,
                               RETURN_INTERSECTION);  // results saved in JPSk_inside_sphere_tmp

  bool needToComputeSafePath;
  int indexH = findIndexH(needToComputeSafePath);

  std::cout << "NeedToComputeSafePath=" << needToComputeSafePath << std::endl;

  if (par_.use_faster == false)
  {
    needToComputeSafePath = true;
  }

  if (needToComputeSafePath == false)
  {
    k_safe = indexH;
    sg_safe_.X_temp_ = std::vector<state>();  // 0 elements
  }
  else
  {
    std::cout << "Before the lock" << std::endl;
    mtx_X_U_temp.lock();

    std::cout << "Here4" << std::endl;
    k_safe = findIndexR(indexH);
    state R = sg_whole_.X_temp_[k_safe];

    mtx_X_U_temp.unlock();

    /*    if (ARisInFreeSpace(indexR_) == false and takeoff_done_ == true)
        {
          std::cout << red << bold << "The piece A-->R is not in Free Space" << std::endl;
          return;
        }*/

    JPSk_inside_sphere_tmp[0] = R.pos;

    if (par_.use_faster == false)
    {
      JPSk_inside_sphere_tmp[0] = A.pos;
    }
    std::cout << "Here5" << std::endl;
    vec_Vecf<3> JPS_safe = JPSk_inside_sphere_tmp;

    // delete extra vertexes
    deleteVertexes(JPS_safe, par_.max_poly_safe);
    M.pos = JPS_safe[JPS_safe.size() - 1];

    // compute convex decomposition of JPS_safe
    jps_manager_.cvxEllipsoidDecomp(JPS_safe, UNKOWN_AND_OCCUPIED_SPACE, l_constraints_safe_, poly_safe_out);

    JPS_safe_out = JPS_safe;

    bool isGinside = l_constraints_safe_[l_constraints_safe_.size() - 1].inside(G.pos);
    M.pos = (isGinside == true) ? G.pos : M.pos;

    state x0_safe;
    x0_safe = R;

    if (par_.use_faster == false)
    {
      x0_safe = stateA_;
    }

    bool shouldForceFinalConstraint_for_Safe = (par_.use_faster == false) ? true : false;

    if (l_constraints_safe_[0].inside(x0_safe.pos) == false)
    {
      std::cout << red << "First point of safe traj is outside" << reset << std::endl;
    }

    sg_safe_.setX0(x0_safe);
    sg_safe_.setXf(M);  // only used to compute dt
    sg_safe_.setPolytopes(l_constraints_safe_);
    sg_safe_.setForceFinalConstraint(shouldForceFinalConstraint_for_Safe);
    MyTimer safe_gurobi_t(true);
    std::cout << "Calling to Gurobi" << std::endl;
    bool solved_safe = sg_safe_.genNewTraj();

    if (solved_safe == false)
    {
      std::cout << red << "No solution found for the safe path" << reset << std::endl;
      return;
    }

    std::cout << "Going to fill the solutions" << std::endl;
    // Get the solution
    sg_safe_.fillXandU();
    X_safe_out = sg_safe_.X_temp_;
    std::cout << "filled the solutions" << std::endl;
  }

  std::cout << "This is the SAFE TRAJECTORY" << std::endl;
  printStateVector(sg_safe_.X_temp_);
  std::cout << "===========================" << std::endl;

  ///////////////////////////////////////////////////////////
  ///////////////       Append RESULTS    ////////////////////
  ///////////////////////////////////////////////////////////
  std::cout << "Going to append" << std::endl;

  if (appendToPlan(k_end_whole, sg_whole_.X_temp_, k_safe, sg_safe_.X_temp_) != true)
  {
    return;
  }

  mtx_plan_.lock();
  std::cout << "This is the COMMITED TRAJECTORY" << std::endl;
  printStateDeque(plan_);
  std::cout << "===========================" << std::endl;
  mtx_plan_.unlock();

  ///////////////////////////////////////////////////////////
  ///////////////       OTHER STUFF    //////////////////////
  //////////////////////////////////////////////////////////

  /*  mtx_planner_status_.lock();
    planner_status_ = PlannerStatus::REPLANNED;

    mtx_planner_status_.unlock();*/

  // Check if we have planned until G_term
  state F = plan_.back();  // Final point of the safe path (\equiv final point of the comitted path)
  std::cout << "F is " << std::endl;
  F.print();
  double dist = (G_term_.pos - F.pos).norm();
  std::cout << "Computed norm" << std::endl;
  if (dist < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_SEEN);
  }

  mtx_offsets.lock();

  int states_last_replan = ceil(replanCB_t.ElapsedMs() / (par_.dc * 1000));  // Number of states that
                                                                             // would have been needed for
                                                                             // the last replan

  /*  if (planner_status_ != PlannerStatus::REPLANNED)  // If already have a solution, keep using the same deltaT_
    {
      deltaT_ = std::max(par_.factor_deltaT * states_last_replan,
                         (double)par_.min_states_deltaT);  // Delta_t

      deltaT_min_ = par_.factor_min_deltaT * states_last_replan;
    }*/

  mtx_offsets.unlock();

  // Time allocation
  double new_init_whole = std::max(sg_whole_.factor_that_worked_ - par_.gamma_whole, 1.0);
  double new_final_whole = sg_whole_.factor_that_worked_ + par_.gammap_whole;
  sg_whole_.setFactorInitialAndFinalAndIncrement(new_init_whole, new_final_whole, par_.increment_whole);

  double new_init_safe = std::max(sg_safe_.factor_that_worked_ - par_.gamma_safe, 1.0);
  double new_final_safe = sg_safe_.factor_that_worked_ + par_.gammap_safe;
  sg_safe_.setFactorInitialAndFinalAndIncrement(new_init_safe, new_final_safe, par_.increment_safe);

  return;
}

bool Faster::appendToPlan(int k_end_whole, const std::vector<state>& whole, int k_safe, const std::vector<state>& safe)
{
  mtx_plan_.lock();

  std::cout << "Erasing" << std::endl;
  bool output;
  int plan_size = plan_.size();
  std::cout << "plan_.size()= " << plan_.size() << std::endl;
  std::cout << "plan_size - k_end_whole = " << plan_size - k_end_whole << std::endl;
  if ((plan_size - 1 - k_end_whole) < 0)
  {
    std::cout << bold << red << "Already publised the point A" << reset << std::endl;
    output = false;
  }
  else
  {
    std::cout << "(plan_.size() - k_end_whole)= " << (plan_.size() - k_end_whole) << std::endl;
    std::cout << "plan_.size()= " << plan_.size() << std::endl;
    std::cout << "k_end_whole)= " << k_end_whole << std::endl;

    plan_.erase(plan_.end() - k_end_whole - 1, plan_.end());

    std::cout << "Erased" << std::endl;

    std::cout << "k_safe = " << k_safe << std::endl;
    std::cout << "whole.size() = " << whole.size() << std::endl;
    for (int i = 0; i < k_safe; i++)
    {
      plan_.push_back(whole[i]);
    }

    std::cout << "k_safe = " << k_safe << std::endl;
    std::cout << "whole.size() = " << whole.size() << std::endl;
    for (int i = 0; i < safe.size(); i++)
    {
      plan_.push_back(safe[i]);
    }
    std::cout << "Pushed everything back" << std::endl;

    output = true;
  }

  mtx_plan_.unlock();
  return output;
}

// void Faster::pubCB(const ros::TimerEvent& e)
void Faster::getNextGoal(state& next_goal)
{
  if (initialized() == false)
  {
    return;
  }

  mtx_goals.lock();
  mtx_plan_.lock();

  next_goal.setZero();
  next_goal = plan_.front();
  if (plan_.size() > 1)
  {
    plan_.pop_front();
  }
  next_goal.yaw = 0;
  next_goal.dyaw = 0;

  mtx_goals.unlock();
  mtx_plan_.unlock();

  /*  if (flight_mode_.mode == LAND)
    {
      double d = sqrt(pow(quadGoal_.pos.z - par_.z_land, 2));
      if (d < 0.1)
      {
        ros::Duration(1.0).sleep();
        flight_mode_.mode = NOT_FLYING;
      }
    }*/

  /*  if (quadGoal_.cut_power && (flight_mode_ == flight_mode_.TAKEOFF || flight_mode_ == flight_mode_.GO))
    {
      double then = ros::Time::now().toSec();
      double diff = 0;
      while (diff < 0.5)  // spinup_time_
      {
        quadGoal_.header.stamp = ros::Time::now();
        diff = ros::Time::now().toSec() - then;
        quadGoal_.cut_power = 0;
        ros::Duration(0.01).sleep();
        pub_goal_.publish(quadGoal_);
      }
    }*/

  /////////////////// Commenting below, 16:41

  /*  if (optimized_ && flight_mode_.mode != NOT_FLYING && flight_mode_.mode != KILL)
    {
      // quadGoal_.cut_power = false;

      mtx_k.lock();

      k_ = std::min(k_, (int)(X_.rows() - 1));

      if (k_ > k_initial_cond_ && status_ == TRAVELING)
      {  // The initial condition of the optimization was already sent to the drone!
         // ROS_WARN("Optimization took too long. Increase deltaT");
      }

      if (((planner_status_ == PlannerStatus::REPLANNED && (k_ == k_initial_cond_ || to_land_ == true)) ||  // Should be
                                                                                                            // k_==
           (force_reset_to_0_ && planner_status_ == PlannerStatus::REPLANNED)))  //&& takeoff_done_ == false)  //
    hacktodo
      {
        to_land_ == false;
        printf("************Reseteando a 0!\n");
        // reset the current optimizations (not needed because I already have a solution)
        sg_whole_.StopExecution();
        sg_safe_.StopExecution();

        force_reset_to_0_ = false;
        mtx_X_U_temp.lock();
        mtx_X_U.lock();
        X_ = X_temp_;
        U_ = U_temp_;
        mtx_X_U.unlock();
        mtx_X_U_temp.unlock();
        X_initialized_ = true;
        k_ = 0;  // Start again publishing the waypoints in X_ from the first row
        mtx_planner_status_.lock();
        planner_status_ = PlannerStatus::START_REPLANNING;
        mtx_planner_status_.unlock();
        // printf("pucCB2: planner_status_=START_REPLANNING\n");
      }

      if ((planner_status_ == PlannerStatus::REPLANNED && (k_ > k_initial_cond_)))
      {  // I've published what I planned --> plan again
        std::cout << bold << magenta << "Rejecting current plan, planning again. Suggestion: Increase delta_t" << reset
                  << std::endl;
        sg_whole_.StopExecution();
        sg_safe_.StopExecution();
        mtx_planner_status_.lock();
        planner_status_ = PlannerStatus::START_REPLANNING;
        status_ = TRAVELING;
        mtx_planner_status_.unlock();
      }

      k_ = std::min(k_, (int)(X_.rows() - 1));
      mtx_k.unlock();

      next_goal.pos = getPos(k_);
      next_goal.vel = getVel(k_);
      next_goal.accel = (par_.use_ff) * getAccel(k_);
      next_goal.jerk = (par_.use_ff) * getJerk(k_);
      next_goal.dyaw = 0;

      if (status_ == YAWING)
      {
        double desired_yaw = atan2(G_.pos[1] - next_goal.pos[1], G_.pos[0] - next_goal.pos[0]);
        double diff = desired_yaw - next_goal.yaw;
        angle_wrap(diff);

        if (fabs(diff) < 0.04)
        {
          status_ = TRAVELING;
        }
        else
        {
          // printf("Yawing\n");
        }
      }

      if ((status_ == TRAVELING || status_ == GOAL_SEEN))
      {
        // double desired_yaw = atan2(quadGoal_.vel.y, quadGoal_.vel.x);
        desired_yaw_B_ = atan2(B_[1] - next_goal.pos[1], B_[0] - next_goal.pos[0]);
        double diff = desired_yaw_B_ - next_goal.yaw;
        angle_wrap(diff);
        // std::cout << red << bold << std::setprecision(6) << "diff after wrappping=" << diff << reset << std::endl;
        if (JPSk_solved_ == true and takeoff_done_ == true and fabs(diff) > 0.04)  // only yaw if diff is big enough
        {
          // yaw(diff, quadGoal_);
        }

        if (JPSk_solved_ == false)
        {
          next_goal.dyaw = 0;
        }
      }
      if (status_ == GOAL_REACHED || takeoff_done_ == false)
      {
        next_goal.dyaw = 0;
        next_goal.yaw = next_goal.yaw;
      }

      mtx_k.lock();
      k_++;

      mtx_k.unlock();
    }*/
  /////////////////// Commented above, 16:41
  /*  else
    {
      quadGoal_.cut_power = true;
    }*/

  /*  ////////////////////////////////
    // NOW generate all the things needed for the jackal
    geometry_msgs::Twist cmd_jackal;

    if (status_ == YAWING)
    {
      cmd_jackal.angular.z = quadGoal_.dyaw;
    }

    else if (status_ == GOAL_REACHED)
    {
      // don't send commands
    }
    else if (k_ >= (int)(X_.rows() - 1) && status_ != GOAL_SEEN)  // stopped at the end of a trajectory, but GOAL not
                                                                  // REACHED --> yaw
    {
      double angle = current_yaw_ - desired_yaw_B_;  // quadGoal_.yaw;
      angle_wrap(angle);
      int direction = (angle > 0) ? -1 : 1;
      cmd_jackal.angular.z = direction * par_.w_max;
    }
    else
    {
      double x = quadGoal_.pos.x;
      double y = quadGoal_.pos.y;
      double xd = quadGoal_.vel.x;
      double yd = quadGoal_.vel.y;
      double xd2 = quadGoal_.accel.x;
      double yd2 = quadGoal_.accel.y;

      double v_desired = sqrt(pow(xd, 2) + pow(yd, 2));
      double alpha = current_yaw_ - atan2(y - state_.pos.y(), x - state_.pos.x());
      angle_wrap(alpha);                                                    // wrap between -pi and pi
      int forward = (alpha <= 3.14 / 2.0 && alpha > -3.14 / 2.0) ? 1 : -1;  // 1 if forward, -1 if backwards
      double dist_error = forward * sqrt(pow(x - state_.pos.x(), 2) + pow(y - state_.pos.y(), 2));
      alpha = (fabs(dist_error) < 0.1) ? 0 : alpha;

      // See http://mathworld.wolfram.com/Curvature.html (diff(phi)/diff(t))
      double numerator = xd * yd2 - yd * xd2;
      double denominator = xd * xd + yd * yd;
      double w_desired = (denominator > 0.01) ? numerator / denominator : 0;
      double desired_yaw = (fabs(xd) < 0.001 || fabs(dist_error) < 0.03) ? desired_yaw_old_ : atan2(yd, xd);

      desired_yaw_old_ = desired_yaw;
      double yaw_error = current_yaw_ - desired_yaw;
      angle_wrap(yaw_error);  // wrap between -pi and pi

      double alpha_dot = (alpha - alpha_before_) / par_.dc;
      alpha_before_ = alpha;

      if (fabs(dist_error) > 0.15)
      {
        cmd_jackal.linear.x = par_.kdist * dist_error;
        cmd_jackal.angular.z = -par_.kalpha * alpha;
      }
      else
      {
        cmd_jackal.linear.x = par_.kv * v_desired;
        cmd_jackal.angular.z = par_.kw * w_desired - par_.kyaw * yaw_error;
      }
    }
    // std::cout << "Publishing Jackal Goal" << std::endl;
    pub_goal_jackal_.publish(cmd_jackal);
    ///////////////////////////////////////////////*/

  // std::cout << "Publishing QUAD Goal" << std::endl;

  // std::cout << "QUAD Goal published" << std::endl;

  // printf("End pubCB\n");
  // printf("#########Time in pubCB %0.2f ms\n", 1000 * (ros::Time::now().toSec() - t0pubCB));
  // mtx_goals.unlock();
}

void Faster::changeDroneStatus(int new_status)
{
  if (new_status == drone_status_)
  {
    return;
  }

  std::cout << "Changing DroneStatus from ";
  switch (drone_status_)
  {
    case DroneStatus::YAWING:
      std::cout << bold << "status_=YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "status_=TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "status_=GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "status_=GOAL_REACHED" << reset;
      break;
  }
  std::cout << " to";

  switch (new_status)
  {
    case DroneStatus::YAWING:
      std::cout << bold << "status_=YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "status_=TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "status_=GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "status_=GOAL_REACHED" << reset;
      break;
  }

  std::cout << std::endl;

  drone_status_ = new_status;
}

void Faster::print_status()
{
  switch (drone_status_)
  {
    case DroneStatus::YAWING:
      std::cout << bold << "status_=YAWING" << reset << std::endl;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "status_=TRAVELING" << reset << std::endl;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "status_=GOAL_SEEN" << reset << std::endl;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "status_=GOAL_REACHED" << reset << std::endl;
      break;
  }

  /*  switch (planner_status_)
    {
      case PlannerStatus::FIRST_PLAN:
        std::cout << bold << "planner_status_=FIRST_PLAN" << reset << std::endl;
        break;
      case PlannerStatus::START_REPLANNING:
        std::cout << bold << "planner_status_=START_REPLANNING" << reset << std::endl;
        break;
      case PlannerStatus::REPLANNED:
        std::cout << bold << "planner_status_=REPLANNED" << reset << std::endl;
        break;
    }*/

  switch (flight_mode_.mode)
  {
    case NOT_FLYING:
      std::cout << bold << "flight_mode_=NOT_FLYING" << reset << std::endl;
      break;
    case TAKEOFF:
      std::cout << bold << "flight_mode_=TAKEOFF" << reset << std::endl;
      break;
    case LAND:
      std::cout << bold << "flight_mode_=LAND" << reset << std::endl;
      break;
    case INIT:
      std::cout << bold << "flight_mode_=INIT" << reset << std::endl;
      break;
    case GO:
      std::cout << bold << "flight_mode_=GO" << reset << std::endl;
      break;
    case ESTOP:
      std::cout << bold << "flight_mode_=ESTOP" << reset << std::endl;
      break;
    case KILL:
      std::cout << bold << "flight_mode_=KILL" << reset << std::endl;
      break;
  }
}