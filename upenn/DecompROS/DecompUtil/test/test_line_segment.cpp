#include "txt_reader.hpp"
#include <decomp_util/line_segment.h>
#include <decomp_geometry/geometric_utils.h>

#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>


int main(int argc, char **argv) {
  if (argc != 2) {
    printf(ANSI_COLOR_RED "Input txt file required!\n" ANSI_COLOR_RESET);
    return -1;
  }

  // Read obstacles
  vec_Vec2f obs;
  if(!read_obs<2>(argv[1], obs)) {
    printf(ANSI_COLOR_RED "Cannot find input file [%s]!\n" ANSI_COLOR_RESET,
           argv[1]);
    return -1;
  }


  // Seed
  const Vec2f pos1(-1.5, 0.0);
  const Vec2f pos2(1.5, 0.3);

  // Initialize SeedDecomp2D
  LineSegment2D decomp(pos1, pos2);
  decomp.set_obs(obs);
  decomp.set_local_bbox(Vec2f(2, 2));
  decomp.dilate(0);

  // Plot the result in svg image
  typedef boost::geometry::model::d2::point_xy<double> point_2d;
  std::ofstream svg("output.svg");
  // Declare a stream and an SVG mapper
  boost::geometry::svg_mapper<point_2d> mapper(svg, 1000, 1000);


  // Draw the canvas 4 x 4m
  boost::geometry::model::polygon<point_2d> bound;
  const double origin_x = -2;
  const double origin_y = -2;
  const double range_x = 4;
  const double range_y = 4;
  std::vector<point_2d> points;
  points.push_back(point_2d(origin_x, origin_y));
  points.push_back(point_2d(origin_x, origin_y + range_y));
  points.push_back(point_2d(origin_x + range_x, origin_y + range_y));
  points.push_back(point_2d(origin_x + range_x, origin_y));
  points.push_back(point_2d(origin_x, origin_y));
  boost::geometry::assign_points(bound, points);
  boost::geometry::correct(bound);

  mapper.add(bound);
  mapper.map(bound, "fill-opacity:1.0;fill:rgb(255,255,255);stroke:rgb(0,0,0);"
                    "stroke-width:2"); // White
  // Draw obstacles
  for(const auto& it: obs) {
    point_2d pt;
    boost::geometry::assign_values(pt, it(0), it(1));
    mapper.add(pt);
    mapper.map(pt, "fill-opacity:1.0;fill:rgb(255,0,0);", 10); // Red
  }

  // Draw ellispoid
  {
    const auto E = decomp.get_ellipsoid();
    int num = 40; // number of points on trajectory to draw
    boost::geometry::model::linestring<point_2d> line;
    for (const auto& it: E.sample(num))
      line.push_back(point_2d(it(0), it(1)));
    line.push_back(line.front());
    mapper.add(line);
    mapper.map(line,
               "opacity:0.4;fill:none;stroke:rgb(118,215,234);stroke-width:5");
  }

  // Draw polygon
  {
    const auto poly = decomp.get_polyhedron();
    const auto vertices = cal_vertices(poly);
    std::string ss("POLYGON((");
    for (size_t i = 0; i < vertices.size(); i++) {
      ss += std::to_string(vertices[i](0)) + " " +
        std::to_string(vertices[i](1));
      if(i == vertices.size() - 1)
        ss += "))";
      else
        ss += ",";
    }

    boost::geometry::model::polygon<point_2d> p;
    boost::geometry::read_wkt(ss, p);
    mapper.add(p);
    mapper.map(p, "fill-opacity:0.2;fill:rgb(51,51,153);stroke:rgb(51,51,153);stroke-width:2");
  }

  // Draw line segment
  {
    const auto line_segment = decomp.get_line_segment();
    boost::geometry::model::linestring<point_2d> line;
    for(const auto& it: line_segment)
      line.push_back(point_2d(it(0), it(1)));
    mapper.add(line);
    mapper.map(line,
               "opacity:0.8;fill:none;stroke:rgb(255,0,0);stroke-width:5");
  }


  // Write title at the lower right corner on canvas
  mapper.text(point_2d(1.0, -1.8), "test_line_segment",
              "fill-opacity:1.0;fill:rgb(10,10,250);");

  return 0;
}
