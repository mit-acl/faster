#include <decomp_util/geometry_utils.h>

Quatf vec_to_quaternion(const Vec3f &v) {
  // zero roll
  const Vec3f rpy(0, atan2(-v(2), v.topRows(2).norm()), atan2(v(1), v(0)));
  Quatf qx(cos(rpy(0) / 2), sin(rpy(0) / 2), 0, 0);
  Quatf qy(cos(rpy(1) / 2), 0, sin(rpy(1) / 2), 0);
  Quatf qz(cos(rpy(2) / 2), 0, 0, sin(rpy(2) / 2));
  return qz * qy * qx;
}

//**** Calculate eigen values
Vec3f eigen_value(const Mat3f& A) {
  Eigen::SelfAdjointEigenSolver<Mat3f> es(A);
  return es.eigenvalues();
}

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

Face closest_obstacle(const Ellipsoid &E, const vec_Vec3f &O) {
  decimal_t dist = std::numeric_limits<decimal_t>::max();
  Vec3f vt, best_v;
  best_v = E.second;
  for (const auto &it : O) {
    vt = E.first.inverse() * (it - E.second);
    if (vt.norm() < dist) {
      dist = vt.norm();
      best_v = it;
    }
  }

  Vec3f a = E.first.inverse() * E.first.inverse().transpose() *
    (best_v - E.second);
  a = a.normalized();
  return Face(best_v, a);
}



//Calculate points inside polyhedron
vec_Vec3f ps_in_polytope(const Polyhedron &Vs, const vec_Vec3f &O) {
  vec_Vec3f new_O;
  for (const auto &it : O) {
    if (inside_polytope(it, Vs))
      new_O.push_back(it);
  }
  return new_O;
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

//**** Determine if a point p is inside polytope
bool inside_polytope(const Vec3f &p,
                     const LinearConstraint3f& C){
  VecDf d = C.first * p - C.second;
  for (unsigned int i = 0; i < d.rows(); i++) {
    if (d(i) > 0)
      return false;
  }
  return true;
}

bool inside_ellipsoid(const Ellipsoid& E,
    const vec_Vec3f& O) {
  for (const auto &it : O) {
    decimal_t d = (E.first.inverse() * (it - E.second)).norm();
    if (d < 1 - 0.001) return true;
  }
  return false;
}

bool closest_pt(const Ellipsoid &E,
    const vec_Vec3f &O,
    Vec3f &best_v,
    int& id) {
  decimal_t dist = std::numeric_limits<decimal_t>::max();
  int cnt = 0;
  for (const auto &it : O) {
    decimal_t d = (E.first.inverse() * (it - E.second)).norm();
    if (d < dist) {
      dist = d;
      best_v = it;
      id = cnt;
    }
    cnt ++;
  }
  return dist < 1; // must have this epsilon here!
  //return dist >= 1 - epsilon_; // must have this epsilon here!
}

//**** Find normals
//*** used for visualization
vec_E<pair_Vec3f> cal_normals(const Polyhedron &vts) {
  vec_E<pair_Vec3f> ns;
  for (const auto& it : vts)
    ns.push_back(std::make_pair(it.n, it.p));
  return ns;
}

//**** Construct Ax <= b
//*** p0 must be inside polytope
LinearConstraint3f cal_Axb(const Vec3f& p0,
                           const Polyhedron &Vs) {
  const unsigned int size = Vs.size();
  MatD3f A(size, 3);
  VecDf b(size);

  for (unsigned int i = 0; i < size; i++) {
    Vec3f n = Vs[i].n;
    decimal_t c = Vs[i].p.dot(n);
    if (n.dot(p0) - c > 0) {
      n = -n;
      c = -c;
    }
    A.row(i) = n;
    b(i) = c;
  }
  return std::make_pair(A, b);
}

//**** Find intersection between two Line
//*** return false if they are not intersected
bool intersect(const pair_Vec3f &v1, const pair_Vec3f &v2,
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
vec_Vec3f line_intersects(const vec_E<pair_Vec3f> &lines) {
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
vec_E<vec_Vec3f> cal_extreme_points(const Polyhedron &vts) {
  vec_E<vec_Vec3f> bds;
  //**** for each plane, find lines on it
  for (unsigned int i = 0; i < vts.size(); i++) {
    const Vec3f t = vts[i].p;
    const Vec3f n = vts[i].n;
    const Quatf q = Quatf::FromTwoVectors(Vec3f(0, 0, 1), n);
    const Mat3f R(q); // body to world
    vec_E<pair_Vec3f> lines;
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
      if (inside_polytope(p, vts)) {
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
        if((pts_inside[k] - prev_pt).norm() > epsilon_)
        {
          prev_pt = pts_inside[k];
          pts_valid.push_back(prev_pt);
        }
      }

      if((pts_inside[size-1] - pts_inside[0]).norm() > epsilon_ &&
          (pts_inside[size-1] - prev_pt).norm() > epsilon_)
        pts_valid.push_back(pts_inside[size-1]);

      //**** insert resulting polygon
      if(pts_valid.size() > 2){
        bds.push_back(pts_valid);
        /*
           std::cout << "add ==========" << std::endl;
           for(auto it: pts_valid)
           std::cout << it.transpose() << std::endl;
           */
      }
    }
  }
  return bds;
}


//**** Find intersect polygon between a plane and polytope
vec_Vec3f plane_polytope_intersection(const Face &plane,
                                      const Polyhedron &vts) {
  if(!inside_polytope(plane.p, vts, epsilon_))
  {
    printf(ANSI_COLOR_RED "pt is outside polyhedron! \n" ANSI_COLOR_RESET);
    return vec_Vec3f();
  }

  const Vec3f t = plane.p;
  const Vec3f n = plane.n;

  const Quatf q = Quatf::FromTwoVectors(Vec3f(0, 0, 1), n);
  const Mat3f R(q); // body to world
  vec_E<pair_Vec3f> lines;
  for (unsigned int j = 0; j < vts.size(); j++) {
    Vec3f nw = vts[j].n;
    Vec3f nb = R.transpose() * nw;
    decimal_t bb = vts[j].p.dot(nw) - nw.dot(t);
    Vec3f v = Vec3f(0, 0, 1).cross(nb);
    Vec3f p;
    if (nb(1) != 0)
      p << 0, bb / nb(1), 0;
    else if (nb(0) != 0)
      p << bb / nb(0), 0, 0;
    else
      continue;
    lines.push_back(std::make_pair(v, p));
  }

  vec_Vec3f pts = line_intersects(lines);
  vec_Vec3f valid_pts;
  for (const auto &it : pts) {
    Vec3f p = R * it + t;
    if (inside_polytope(p, vts, 1e-2))
      valid_pts.push_back(it);
  }

  //**** sort in plane frame
  valid_pts = sort_pts(valid_pts);

  //**** transform to world frame
  for (auto &it : valid_pts)
    it = R * it + t;

  return valid_pts;
}

//**** uniformly sample path into many segments
vec_Vec3f path_downsample(const vec_Vec3f& ps, decimal_t d){
  // subdivide according to length
  if(ps.empty())
    return ps;
  vec_Vec3f path;
  for(unsigned int i = 1; i < ps.size(); i++){
    decimal_t dist = (ps[i] - ps[i-1]).norm();
    int cnt = std::ceil(dist / d);
    for(int j = 0; j < cnt; j++)
      path.push_back(ps[i-1] + j * (ps[i] - ps[i-1]) / cnt);
  }
  path.push_back(ps.back());

  return path;
}

vec_Vec3f path_downsample_i(const vec_Vec3f& ps, int cnt){
  // subdivide int to cnt segments
  if(ps.empty() || cnt < 2)
    return ps;
  vec_Vec3f path;
  for(unsigned int i = 1; i < ps.size(); i++){
    for(int j = 0; j < cnt; j++)
      path.push_back(ps[i-1] + j * (ps[i] - ps[i-1]) / cnt);
  }
  path.push_back(ps.back());

  return path;
}

vec_Vec3f path_crop(const vec_Vec3f& ps, decimal_t d){
  if(ps.size() < 2 || d < 0)
    return ps;

  vec_Vec3f path;
  Vec3f end = ps.back();
  decimal_t dist = 0;
  for(unsigned int i = 1; i < ps.size(); i++){
    if(dist + (ps[i] - ps[i-1]).norm() > d){
      end = ps[i-1] + (d - dist) * (ps[i] - ps[i-1]).normalized();
      path.push_back(ps[i-1]);
      break;
    }
    else
    {
      dist += (ps[i] - ps[i-1]).norm();
      path.push_back(ps[i-1]);
    }
  }


  if((path.back() - end).norm() > 5e-1)
    path.push_back(end);
  return path;
}

//**** find the intersection of two polytopes
Polyhedron polytope_intersection(const Polyhedron& vs1,
    const Polyhedron& vs2){
  Polyhedron candidates;
  for(const auto& it2: vs2)
  {
    bool add = true;
    for(const auto& it1: vs1)
    {
      if((it2.n - it1.n).norm() < epsilon_ &&
          fabs((it2.p - it1.p).dot(it1.n)) < epsilon_)
      {
        add = false;
        break;
      }
    }
    if(add)
      candidates.push_back(it2);
  }
  Polyhedron intersect_vs = vs1;
  intersect_vs.insert(intersect_vs.end(), candidates.begin(), candidates.end());
  return intersect_vs;
}

//**** Create triangles from a face
vec_E<vec_Vec3f> chop_triangle(const vec_Vec3f& pts){
  vec_E<vec_Vec3f> trias;
  if(pts.size() < 3){
    printf(ANSI_COLOR_RED  "In chop triangles, the number of points is %zu < 3\n" ANSI_COLOR_RESET, pts.size());
    return trias;
  }

  const Vec3f ref_pt = pts[0];
  for(unsigned int i = 1; i < pts.size() - 1; i++){
    vec_Vec3f tria;
    tria.push_back(ref_pt);
    tria.push_back(pts[i]);
    tria.push_back(pts[i+1]);
    trias.push_back(tria);
  }
  return trias;
}

//**** Calculate the volume of a polytope
decimal_t cal_volume(const vec_E<vec_Vec3f>& fs, const Vec3f& pt_inside){
	vec_E<vec_Vec3f> triangles;
	for(const auto& f: fs){
		vec_E<vec_Vec3f> trias = chop_triangle(f);
		triangles.insert(triangles.end(), trias.begin(), trias.end());
	}

	decimal_t V = 0;
	for(const auto& tria: triangles){
		Vec3f n = (tria[0] - tria[1]).cross(tria[0] - tria[2]);
		V += fabs((tria[0] - pt_inside).dot(n));
	}

	V /= 6;
	return V;
}

//**** Find the centroid of a polygon
Vec3f cal_centroid_2d(const vec_Vec3f &pts, const Face &p) {
  if (pts.size() < 3) {
    printf(ANSI_COLOR_RED "In getting 2d centroid, the number of vertices is %zu < 3\n" ANSI_COLOR_RESET,
           pts.size());
    return p.p;
  }

  const Vec3f t = p.p;
  const Vec3f n = p.n;
  const Quatf q =
      Quatf::FromTwoVectors(Vec3f(0, 0, 1), n);
  const Mat3f R(q); // body to world

  vec_Vec3f pts_b;
  for (auto it : pts)
    pts_b.push_back(R.transpose() * (it - t));

  decimal_t A = 0;
  decimal_t Cx = 0, Cy = 0;
  for (unsigned int i = 0; i < pts_b.size(); i++) {
    unsigned int j = i + 1;
    if (j >= pts_b.size())
      j = 0;
    decimal_t item = pts_b[i](0) * pts_b[j](1) - pts_b[j](0) * pts_b[i](1);

    A += item;
    Cx += (pts_b[i](0) + pts_b[j](0)) * item;
    Cy += (pts_b[i](1) + pts_b[j](1)) * item;
  }
  A /= 2;
  Cx /= 6 * A;
  Cy /= 6 * A;

  Vec3f c(Cx, Cy, 0);

  return R * c + t;
}

//**** Calculate centroid of a polytope
Vec3f cal_centroid_3d(const vec_E<vec_Vec3f>& fs, const Vec3f& pt_inside){
  vec_E<vec_Vec3f> triangles;
  for(const auto& f: fs){
    vec_E<vec_Vec3f> trias = chop_triangle(f);
    triangles.insert(triangles.end(), trias.begin(), trias.end());
  }

  decimal_t V = 0;
  for(const auto& tria: triangles){
    Vec3f n = (tria[0] - tria[1]).cross(tria[0] - tria[2]);
    decimal_t v = fabs((tria[0] - pt_inside).dot(n));
    V += v;
  }

  V /= 6;

  Vec3f C(0, 0, 0);
  for(const auto& tria: triangles){
    Vec3f n = (tria[1] - tria[0]).cross(tria[2] - tria[0]);
    Vec3f a = tria[0] - pt_inside;
    Vec3f b = tria[1] - pt_inside;
    Vec3f c = tria[2] - pt_inside;
    if(a.dot(n) < 0)
      n = -n;
    for(int i = 0; i< 3; i++)
      C(i) += n(i) * (std::pow(a(i) + b(i), 2) + std::pow(b(i) + c(i), 2) + std::pow(c(i) + a(i), 2));
  }

  C /= (2*V*24);

  return C + pt_inside;
}


//**** Get closest distance
decimal_t cal_closest_dist(const Vec3f& pt, const Polyhedron& vs){
  float dist = 10;
  for(const auto& it: vs){
    decimal_t d = fabs(it.n.dot(pt - it.p));
    if(d < dist)
      dist = d;
  }
  return dist;
}

vec_Vec3f ps_in_ellipsoid(const Ellipsoid &E,
    const vec_Vec3f &O) {
  vec_Vec3f new_O;
  for (const auto &it : O) {
    decimal_t d = (E.first.inverse() * (it - E.second)).norm();
    if (d < 1)
      new_O.push_back(it);
  }
  return new_O;
}
