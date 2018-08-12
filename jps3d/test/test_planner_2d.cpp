#include "timer.hpp"
#include "read_map.hpp"
#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using namespace JPS;

int main(int argc, char ** argv){
  if(argc != 2) {
    printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
    return -1;
  }

  // Read the map from yaml
  MapReader<Vec2i, Vec2f> reader(argv[1], true); // Map read from a given file
  if(!reader.exist()) {
    printf(ANSI_COLOR_RED "Cannot read input file [%s]!\n" ANSI_COLOR_RESET, argv[1]);
    return -1;
  }

  // store map in map_util
  std::shared_ptr<OccMapUtil> map_util = std::make_shared<OccMapUtil>();
  map_util->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());

  const Vec2f start(reader.start(0), reader.start(1));
  const Vec2f goal(reader.goal(0), reader.goal(1));

  std::unique_ptr<JPSPlanner2D> planner_ptr(new JPSPlanner2D(false)); // Declare a planner
  planner_ptr->setMapUtil(map_util); // Set collision checking function
  planner_ptr->updateMap();

  Timer time_jps(true);
  bool valid_jps = planner_ptr->plan(start, goal, 1, true); // Plan from start to goal using JPS
  double dt_jps = time_jps.Elapsed().count();
  const auto path_jps = planner_ptr->getRawPath(); // Get the planned raw path from JPS
  printf("JPS Planner takes: %f ms\n", dt_jps);
  printf("JPS Path Distance: %f\n", total_distance2f(path_jps));

  Timer time_astar(true);
  bool valid_astar = planner_ptr->plan(start, goal, 1, false); // Plan from start to goal using A*
  double dt_astar = time_astar.Elapsed().count();
  const auto path_astar = planner_ptr->getRawPath(); // Get the planned raw path from A*
  printf("JPS Planner takes: %f ms\n", dt_jps);
  printf("AStar Planner takes: %f ms\n", dt_astar);
  printf("AStar Path Distance: %f\n", total_distance2f(path_astar));

  // Plot the result in svg image
  typedef boost::geometry::model::d2::point_xy<double> point_2d;
  // Declare a stream and an SVG mapper
  std::ofstream svg("output.svg");
  boost::geometry::svg_mapper<point_2d> mapper(svg, 1000, 1000);

  // Draw the canvas
  boost::geometry::model::polygon<point_2d> bound;
  const Vec2i dim = map_util->getDim();
  const Vec2f ori = map_util->getOrigin();
  const double res = map_util->getRes();

  const double origin_x = ori(0);
  const double origin_y = ori(1);
  const double range_x = dim(0) * res;
  const double range_y = dim(1) * res;
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
  boost::geometry::assign_values(start_pt, start(0), start(1));
  mapper.add(start_pt);
  mapper.map(start_pt, "fill-opacity:1.0;fill:rgb(255,0,0);", 10); // Red
  boost::geometry::assign_values(goal_pt, goal(0), goal(1));
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

  // Draw the path from JPS
  if(valid_jps) {
    vec_Vec2f path = path_jps;
    boost::geometry::model::linestring<point_2d> line;
    for(auto pt: path)
      line.push_back(point_2d(pt(0), pt(1)));
    mapper.add(line);
    mapper.map(line, "opacity:0.4;fill:none;stroke:rgb(212,0,0);stroke-width:5"); // Red
  }

  // Draw the path from A*
  if(valid_astar) {
    vec_Vec2f path = path_astar;
    boost::geometry::model::linestring<point_2d> line;
    for(auto pt: path)
      line.push_back(point_2d(pt(0), pt(1)));
    mapper.add(line);
    mapper.map(line, "opacity:0.4;fill:none;stroke:rgb(1,212,0);stroke-width:5"); // Green
  }

  // Write title at the lower right corner on canvas
  mapper.text(point_2d(origin_x + range_x - 6, origin_y+1.8), "test_planner_2d",
              "fill-opacity:1.0;fill:rgb(10,10,250);");

  mapper.text(point_2d(origin_x + range_x - 8, origin_y+1.2), "Green: ",
              "fill-opacity:1.0;fill:rgb(1,212,0);");
  mapper.text(point_2d(origin_x + range_x - 5.5, origin_y+1.2), "astar path",
              "fill-opacity:1.0;fill:rgb(0,0,0);");

  mapper.text(point_2d(origin_x + range_x - 8, origin_y+0.6), "Red: ",
              "fill-opacity:1.0;fill:rgb(212,0,0);");
  mapper.text(point_2d(origin_x + range_x - 5.5, origin_y+0.6), "jps path",
              "fill-opacity:1.0;fill:rgb(0,0,0);");

  return 0;
}
