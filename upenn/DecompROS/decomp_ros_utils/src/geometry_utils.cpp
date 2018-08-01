#include "geometry_utils.h"

//**** Sort poits in an order
vec_Vec3f sort_pts(const vec_Vec3f &pts) {
  //**** sort in body frame
  Vec3f avg = Vec3f::Zero();
  for (auto p : pts)
    avg += p;
  avg /= pts.size();

  vec_E<std::pair<decimal_t, Vec3f>> ps_valued;
  ps_valued.resize(pts.size());
  for (unsigned int i = 0; i < pts.size(); i++) {
    decimal_t theta = atan2(pts[i](1) - avg(1), pts[i](0) - avg(0));
    ps_valued[i] = std::make_pair(theta, pts[i]);
  }

  std::sort(ps_valued.begin(), ps_valued.end(), compP<decimal_t, Vec3f>);
  vec_Vec3f b;
  for (const auto& it : ps_valued)
    b.push_back(it.second);
  return b;
}


//**** Determine if a point p is inside polytope
bool inside_polytope(const Vec3f &p,
                     const Polyhedron &Vs,
                     decimal_t epsilon) {
  bool inside = true;
  for (const auto& v : Vs) {
    Vec3f a = v.n;
    decimal_t b = v.p.dot(a);
    if (a.dot(p) - b > epsilon) {
      inside = false;
      break;
    }
  }
  return inside;
}


//**** Find intersection between two Line
//*** return false if they are not intersected
bool intersect(const std::pair<Vec3f, Vec3f>&v1, 
    const std::pair<Vec3f, Vec3f> &v2,
    Vec3f &pi) {
  if (v1.second == v2.second)
    return false;

  decimal_t a1 = -v1.first(1);
  decimal_t b1 = v1.first(0);
  decimal_t c1 = a1 * v1.second(0) + b1 * v1.second(1);

  decimal_t a2 = -v2.first(1);
  decimal_t b2 = v2.first(0);
  decimal_t c2 = a2 * v2.second(0) + b2 * v2.second(1);

  decimal_t x = (c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1);
  decimal_t y = (c1 * a2 - c2 * a1) / (a2 * b1 - a1 * b2);

  if (std::isnan(x) || std::isnan(y) || std::isinf(x) || std::isinf(y))
    return false;
  else {
    pi << x, y, 0;
    return true;
  }
}

//**** Find intersection between multiple lines
vec_Vec3f line_intersects(const vec_E<std::pair<Vec3f, Vec3f>> &lines) {
  vec_Vec3f pts;
  for (unsigned int i = 0; i < lines.size(); i++) {
    for (unsigned int j = i + 1; j < lines.size(); j++) {
      Vec3f pi;
      if (intersect(lines[i], lines[j], pi)) {
        pts.push_back(pi);
      }
    }
  }
  return pts;
}


//**** Find extreme points of polytope
std::pair<BoundVec3f, std::vector<bool>> cal_extreme_points(const Polyhedron &vts) {
  decimal_t epsilon = 1e-4;
  BoundVec3f bds;
  std::vector<bool> passes;
  //**** for each plane, find lines on it
  for (unsigned int i = 0; i < vts.size(); i++) {
    const Vec3f t = vts[i].p;
    const Vec3f n = vts[i].n;
    const Quatf q = Quatf::FromTwoVectors(Vec3f(0, 0, 1), n);
    const Mat3f R(q); // body to world
    vec_E<std::pair<Vec3f, Vec3f>> lines;
    for (unsigned int j = 0; j < vts.size(); j++) {
      if (j == i)
        continue;
      Vec3f nw = vts[j].n;
      Vec3f nb = R.transpose() * nw;
      decimal_t bb = vts[j].p.dot(nw) - nw.dot(t);
      Vec3f v = Vec3f(0, 0, 1).cross(nb); // line direction
      Vec3f p; // point on the line
      if (nb(1) != 0)
        p << 0, bb / nb(1), 0;
      else if (nb(0) != 0)
        p << bb / nb(0), 0, 0;
      else
        continue;
      lines.push_back(std::make_pair(v, p));
    }

    //**** find all intersect points
    vec_Vec3f pts = line_intersects(lines);
    //**** filter out points inside polytope
    vec_Vec3f pts_inside;
    for (const auto& it : pts) {
      Vec3f p = R * it + t; // convert to world frame
      if (inside_polytope(p, vts, epsilon)) {
        pts_inside.push_back(it);
      }
    }

    //**** sort in plane frame
    pts_inside = sort_pts(pts_inside);

    //**** transform to world frame
    for (auto &it : pts_inside)
      it = R * it + t;

    if(pts_inside.size() > 2){
      vec_Vec3f pts_valid;
      pts_valid.push_back(pts_inside[0]);
      Vec3f prev_pt = pts_inside[0];
      const unsigned int size = pts_inside.size();
      for(unsigned int k = 1; k < size - 1; k++){
	if((pts_inside[k] - prev_pt).norm() > epsilon)
	{
	  prev_pt = pts_inside[k];
	  pts_valid.push_back(prev_pt);
	}
      }

      if((pts_inside[size-1] - pts_inside[0]).norm() > epsilon &&
	  (pts_inside[size-1] - prev_pt).norm() > epsilon)
	pts_valid.push_back(pts_inside[size-1]);

      //**** insert resulting polygon
      if(pts_valid.size() > 2){
	bds.push_back(pts_valid);
        passes.push_back(vts[i].pass);
	/*
	   std::cout << "add ==========" << std::endl;
	   for(auto it: pts_valid)
	   std::cout << it.transpose() << std::endl;
	   */
      }
    }
  }
  return std::make_pair(bds, passes);
}

Vec3f intersect(const pair_Vec3f &v1, const pair_Vec3f &v2) {
  if(std::fabs(v1.second(0)) > 1e-3) {
    double r = v1.second(1) / v1.second(0);
    double d2 = (r*v1.first(0)-v1.first(1) - r*v2.first(0) + v2.first(1)) / (r * v2.second(0) - v2.second(1));
    return v2.first + d2 * v2.second;
  }
  else {
    double r = v2.second(1) / v2.second(0);
    double d1 = (r*v2.first(0)-v2.first(1) - r*v1.first(0) + v1.first(1)) / (r * v1.second(0) - v1.second(1));
    return v1.first + d1 * v1.second;
  }
}



vec_Vec3f inflate(const vec_Vec3f& cs, double r) {
  if(cs.size() < 3)
    return cs;

  const Vec3f normal = (cs[1] - cs[0]).cross(cs[2]-cs[1]).normalized();
  const Vec3f normalZ(0, 0, 1);
  const Quatf q = Quatf::FromTwoVectors(normalZ, normal); 
  const Mat3f R(q); // body to world    

  vec_Vec3f new_cs; // in body frame
  for(const auto& pt: cs)
    new_cs.push_back(R.transpose() * (pt - cs.front()));  

  vec_E<pair_Vec3f> lines;
  for(int i = 0; i < (int) new_cs.size(); i ++) {
    int j = i + 1 >= (int) new_cs.size() ? 0 : i+1;
    const Vec3f n = (new_cs[j] - new_cs[i]).normalized();
    const Vec3f n2 = n.cross(normalZ).normalized();
    const Vec3f p = new_cs[i] + r*n2;
    lines.push_back(std::make_pair(p, n));
  }

  vec_E<pair_Vec3f> new_lines;
  for(int i = 0; i < (int) lines.size(); i ++) {
    int j = i + 1 >= (int) lines.size() ? 0 : i+1;
    Vec3f p = intersect(lines[i], lines[j]);
    new_lines.push_back(lines[i]);
    const Vec3f n = (p - new_cs[j]).normalized();
    const Vec3f n2 = normalZ.cross(n).normalized();
    Vec3f new_p  = new_cs[j] + r*n;
    new_lines.push_back(std::make_pair(new_p, n2));
  }

  vec_Vec3f c2;
  //printf("inflated: \n");
  for(int i = 0; i < (int) new_lines.size(); i ++) {
    int j = i + 1 >= (int) new_lines.size() ? 0 : i+1;
    Vec3f p = intersect(new_lines[i], new_lines[j]);
    c2.push_back(R * p + cs.front());
    //std::cout << c2.back().transpose() << std::endl;
  }


  return c2;
}

