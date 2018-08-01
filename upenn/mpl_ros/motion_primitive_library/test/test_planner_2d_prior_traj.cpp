#include "timer.hpp"
#include "read_map.hpp"
#include <motion_primitive_library/planner/mp_map_util.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using namespace MPL;

int main(int argc, char ** argv){
  if(argc != 2) {
    printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
    return -1;
  }

  // Load the map 
  MapReader<Vec2i, Vec2f> reader(argv[1]); 
  if(!reader.exist()) {
    printf(ANSI_COLOR_RED "Cannot find input file [%s]!\n" ANSI_COLOR_RESET, argv[1]);
    return -1;
  }

  // Pass the data into a VoxelMapUtil class for collision checking
  std::shared_ptr<OccMapUtil> map_util;
  map_util.reset(new OccMapUtil);
  map_util->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());
  map_util->freeUnknown();

  // Initialize start and goal, using vel control
  Waypoint2 start, goal;
  start.pos = Vec2f(reader.start(0), reader.start(1)); 
  start.vel = Vec2f::Zero(); 
  start.acc = Vec2f::Zero(); 
  start.jrk = Vec2f::Zero(); 
  start.use_pos = true;
  start.use_vel = false;
  start.use_acc = false; 
  start.use_jrk = false; 

  goal.pos = Vec2f(reader.goal(0), reader.goal(1));
  goal.vel = Vec2f::Zero(); 
  goal.acc = Vec2f::Zero(); 
  goal.jrk = Vec2f::Zero(); 
 
  goal.use_pos = start.use_pos;
  goal.use_vel = start.use_vel;
  goal.use_acc = start.use_acc;
  goal.use_jrk = start.use_jrk;

  // Initialize control input
  decimal_t u_max = 1.0;
  decimal_t du = u_max;
  vec_Vec2f U;
  for(decimal_t dx = -u_max; dx <= u_max; dx += du )
    for(decimal_t dy = -u_max; dy <= u_max; dy += du )
      U.push_back(Vec2f(dx, dy));


  // Initialize planner
  std::unique_ptr<MPMap2DUtil> planner(new MPMap2DUtil(true)); // Declare a mp planner using voxel map
  planner->setMapUtil(map_util); // Set collision checking function
  planner->setEpsilon(1.0); // Set greedy param (default equal to 1)
  planner->setVmax(1.0); // Set max velocity
  planner->setAmax(1.0); // Set max acceleration 
  planner->setUmax(u_max); // Set max control input
  planner->setDt(1.0); // Set dt for each primitive
  planner->setW(10); // Set weight for time
  planner->setMaxNum(-1); // Set maximum allowed states
  planner->setU(U); // Set control input 
  planner->setTol(0.5); // Tolerance for goal region


  // Planning
  Timer time1(true);
  planner->plan(start, goal); // Plan from start to goal
  double dt = time1.Elapsed().count();
  printf("MP Planner prior takes: %f ms\n", dt);
  printf("MP Planner prior expanded states: %zu\n", planner->getCloseSet().size());

  Trajectory2 prior_traj = planner->getTraj();
  printf("Total time T: %f\n", prior_traj.getTotalTime());
  printf("Total J:  J(1) = %f, J(2) = %f, J(3) = %f, J(4) = %f\n", 
      prior_traj.J(1), prior_traj.J(2), prior_traj.J(3), prior_traj.J(4));
 
  // Using acc control
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = true; 
  start.use_jrk = false; 

  // Reset control input
  u_max = 0.5;
  du = u_max;
  U.clear();
  for(decimal_t dx = -u_max; dx <= u_max; dx += du )
    for(decimal_t dy = -u_max; dy <= u_max; dy += du )
      U.push_back(Vec2f(dx, dy));

  // Reset planner
  int alpha = 0;
  planner.reset(new MPMap2DUtil(true)); // Declare a mp planner using voxel map
  planner->setMapUtil(map_util); // Set collision checking function
  planner->setEpsilon(1.0); // Set greedy param (default equal to 1)
  planner->setVmax(1.0); // Set max velocity
  planner->setAmax(1.0); // Set max acceleration 
  planner->setUmax(u_max); // Set max control input
  planner->setDt(1.0); // Set dt for each primitive
  planner->setW(10); // Set weight for time
  planner->setMaxNum(-1); // Set maximum allowed states
  planner->setU(U);// Set control input
  planner->setTol(0.5); // Tolerance for goal region
  planner->setAlpha(alpha);
  planner->setPriorTrajectory(prior_traj);


  // Planning
  Timer time2(true);
  bool valid = planner->plan(start, goal); // Plan from start to goal
  dt = time2.Elapsed().count();
  printf("MP Planner takes: %f ms\n", dt);
  printf("MP Planner expanded states: %zu\n", planner->getCloseSet().size());


  // Plot the result in image
  const Vec2i dim = reader.dim();
  typedef boost::geometry::model::d2::point_xy<double> point_2d;
  // Declare a stream and an SVG mapper
  std::ofstream svg("output.svg");
  boost::geometry::svg_mapper<point_2d> mapper(svg, 1000, 1000);

  // Draw the canvas
  boost::geometry::model::polygon<point_2d> bound;
  const double origin_x = reader.origin()(0);
  const double origin_y = reader.origin()(1);
  const double range_x = reader.dim()(0) * reader.resolution();
  const double range_y = reader.dim()(1) * reader.resolution();
  std::vector<point_2d> points;
  points.push_back(point_2d(origin_x, origin_y));
  points.push_back(point_2d(origin_x, origin_y+range_y));
  points.push_back(point_2d(origin_x+range_x, origin_y+range_y));
  points.push_back(point_2d(origin_x+range_x, origin_y));
  points.push_back(point_2d(origin_x, origin_y));
  boost::geometry::assign_points(bound, points);
  boost::geometry::correct(bound);

  mapper.add(bound);
  mapper.map(bound, "fill-opacity:1.0;fill:rgb(255,255,255);stroke:rgb(0,0,0);stroke-width:2"); // White

  // Draw start and goal
  point_2d start_pt, goal_pt;
  boost::geometry::assign_values(start_pt, start.pos(0), start.pos(1));
  mapper.add(start_pt);
  mapper.map(start_pt, "fill-opacity:1.0;fill:rgb(255,0,0);", 10); // Red
  boost::geometry::assign_values(goal_pt, goal.pos(0), goal.pos(1));
  mapper.add(goal_pt);
  mapper.map(goal_pt, "fill-opacity:1.0;fill:rgb(255,0,0);", 10); // Red

  // Draw the obstacles
  for(int x = 0; x < dim(0); x ++) {
    for(int y = 0; y < dim(1); y ++) {
      if(!map_util->isFree(Vec2i(x, y))) {
        Vec2f pt = map_util->intToFloat(Vec2i(x, y));
        point_2d a;
        boost::geometry::assign_values(a, pt(0), pt(1));
        mapper.add(a);
        mapper.map(a, "fill-opacity:1.0;fill:rgb(0,0,0);", 1);
      }
    }
  }

  // Draw expended states
  for(const auto& pt: planner->getCloseSet()) {
    point_2d a;
    boost::geometry::assign_values(a, pt(0), pt(1));
    mapper.add(a);
    mapper.map(a, "fill-opacity:1.0;fill:rgb(100,100,200);", 2); // Blue
  }



  if(valid) {
    Trajectory2 traj = planner->getTraj();
    decimal_t total_t = traj.getTotalTime();
    printf("Refined total time T: %f\n", total_t);
    printf("Refined total J:  J(1) = %f, J(2) = %f, J(3) = %f, J(4) = %f\n", 
        traj.J(1), traj.J(2), traj.J(3), traj.J(4));

    //printf("alpha: %d, ratio: %f\n", alpha, (10 * total_t + traj.J(1))/(10*prior_traj.getTotalTime() + prior_traj.J(1)));

    // Draw trajectory (Red thick line)
    int num = 500; // number of points on trajectory to draw
    double dt = total_t / num; 
    boost::geometry::model::linestring<point_2d> line;
    Vec2f prev_pt;
    for(double t = 0; t <= total_t; t += dt) {
      Waypoint2 w;
      traj.evaluate(t, w);
      if((w.pos - prev_pt).norm() > 0.2) {
        line.push_back(point_2d(w.pos(0), w.pos(1)));
        prev_pt = w.pos;
      }
    }
    mapper.add(line);
    mapper.map(line, "opacity:0.4;fill:none;stroke:rgb(212,0,0);stroke-width:5"); // Red

    // Draw states long trajectory
    for(const auto& pt: planner->getWs()) {
      point_2d a;
      boost::geometry::assign_values(a, pt.pos(0), pt.pos(1));
      mapper.add(a);
      mapper.map(a, "fill-opacity:1.0;fill:rgb(10,10,250);", 2); // Blue
    }

    // Draw prior trajectory (Black thin line)
    total_t = prior_traj.getTotalTime();
    dt = total_t / num; 
    boost::geometry::model::linestring<point_2d> prior_line;
    for(double t = 0; t <= total_t; t += dt) {
      Waypoint2 w;
      prior_traj.evaluate(t, w);
      if((w.pos - prev_pt).norm() > 0.2) {
        prior_line.push_back(point_2d(w.pos(0), w.pos(1)));
        prev_pt = w.pos;
      }
    }
    mapper.add(prior_line);
    mapper.map(prior_line, "opacity:1.0;fill:none;stroke:rgb(0,10,0);stroke-width:2"); // Black
 
  }


  return 0;
}
