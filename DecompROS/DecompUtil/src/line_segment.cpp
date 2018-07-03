#include <decomp_util/ellipse_decomp.h>

LineSegment::LineSegment(const Vec3f &p1, const Vec3f &p2):
  p1_(p1), p2_(p2) {}


void LineSegment::set_virtual_dim(decimal_t x, decimal_t y, decimal_t z){
  virtual_x_ = x;
  virtual_y_ = y;
  virtual_z_ = z;
}

void LineSegment::set_obstacles(const vec_Vec3f& obs) {
  Polyhedron vs;
  add_virtual_wall(vs);
  obs_ = ps_in_polytope(vs, obs);
}

Ellipsoid LineSegment::find_ellipsoid(const Vec3f& p1, const Vec3f& p2, double offset_x){
  const decimal_t f = (p1 - p2).norm() / 2;
  Mat3f C = f * Mat3f::Identity();
  C(0, 0) += offset_x;
  Vec3f axes(C(0, 0), C(1, 1), C(2, 2));
  if(axes(0) > 0) {
    double ratio = axes(1) / axes(0);
    axes *= ratio;
    C *= ratio;
  }

  const Quatf qi = vec_to_quaternion(p2 - p1);
  C = qi * C * qi.conjugate();

  Ellipsoid E = std::make_pair(C, (p1 + p2) / 2);
  Vec3f pw = Vec3f::Zero();
  Quatf qf = qi;

  vec_Vec3f Os = ps_in_ellipsoid(E, obs_);
  Vec3f prev_pw;
  bool removed = false;
  //**** decide short axes
  while (inside_ellipsoid(E, Os)) {
    int id = -1;
    closest_pt(E, Os, pw, id);
    Vec3f p = qi.inverse() * (pw - E.second); // to ellipse frame
    const decimal_t roll = atan2(p(2), p(1));
    qf = qi * Quatf(cos(roll / 2), sin(roll / 2), 0, 0);
    p = qf.inverse() * (pw - E.second);
    axes(1) = fabs(p(1)) / sqrt(1 - pow(p(0) / axes(0), 2));
    E.first = Mat3f::Identity();
    E.first(0, 0) = axes(0);
    E.first(1, 1) = axes(1);
    E.first(2, 2) = axes(1);
    E.first = qf * E.first * qf.conjugate();

    Os.erase(Os.begin() + id); // remove pw
    if(removed)
      Os.push_back(prev_pw);
    removed = true;
    prev_pw = pw;

  }

  //**** reset ellipsoid with old axes(2)
  E.first = f * Mat3f::Identity();
  E.first(0, 0) = axes(0);
  E.first(1, 1) = axes(1);
  E.first(2, 2) = axes(2);
  E.first = qf * E.first * qf.conjugate();
  Os = ps_in_ellipsoid(E, Os);

  removed = false;
  while (inside_ellipsoid(E, Os)) {
    int id = -1;
    closest_pt(E, Os, pw, id);

    Vec3f p = qf.inverse() * (pw - E.second);
    axes(2) =
      fabs(p(2)) / sqrt(1 - pow(p(0) / axes(0), 2) - pow(p(1) / axes(1), 2));
    E.first = Mat3f::Identity();
    E.first(0, 0) = axes(0);
    E.first(1, 1) = axes(1);
    E.first(2, 2) = axes(2);
    E.first = qf * E.first * qf.conjugate();

    Os.erase(Os.begin() + id); // remove pw
    if(removed)
      Os.push_back(prev_pw);
    removed = true;
    prev_pw = pw;
  }

  return E;
}

Polyhedron LineSegment::find_polyhedron(const Ellipsoid& E){
  //**** find half-space
  Polyhedron Vs;
  vec_Vec3f O_remain = obs_;
  while (!O_remain.empty()) {
    Face v = closest_obstacle(E, O_remain);
    //adjust(v);
    Vs.push_back(v);
    Vec3f a = v.n;
    decimal_t b = v.p.dot(a);
    vec_Vec3f O_tmp;
    for (const auto &it : O_remain) {
      if (a.dot(it) - b < 0)
        O_tmp.push_back(it);
    }
    O_remain = O_tmp;
    /*
    std::cout << "a: " << a.transpose() << std::endl;
    std::cout << "b: " << b << std::endl;
    */
  }

  return Vs;
}

void LineSegment::add_virtual_wall(Polyhedron &Vs) {
  //**** virtual walls parallel to path p1->p2
  Vec3f dir = p2_ - p1_;
  dir /= dir.norm();
  Vec3f dir_h(dir(1), -dir(0), 0);
  if (dir_h == Vec3f::Zero())
    dir_h << -1, 0, 0;
  Vec3f pp1 = p1_ + dir_h * virtual_y_;
  Vec3f pp2 = p1_ - dir_h * virtual_y_;
  Vs.push_back(Face(pp1, dir_h));
  Vs.push_back(Face(pp2, -dir_h));

  Vec3f dir_v = dir.cross(dir_h);
  Vec3f pp3 = p1_ + dir_v * virtual_z_;
  Vec3f pp4 = p1_ - dir_v * virtual_z_;
  Vs.push_back(Face(pp3, dir_v));
  Vs.push_back(Face(pp4, -dir_v));

  Vec3f pp5 = p2_ + dir * virtual_x_;
  Vec3f pp6 = p1_ - dir * virtual_x_;
  Vs.push_back(Face(pp5, dir));
  Vs.push_back(Face(pp6, -dir));
}

void LineSegment::dilate(decimal_t radius) {
  ellipsoid_ = find_ellipsoid(p1_, p2_, radius);
  polyhedron_ = find_polyhedron(ellipsoid_);
  add_virtual_wall(polyhedron_);
}


decimal_t LineSegment::ellipsoid_volume() {
  return ellipsoid_.first.determinant();
}

decimal_t LineSegment::polyhedron_volume() {
  vec_E<vec_Vec3f> bs = cal_extreme_points(polyhedron_);
  return cal_volume(bs, (p1_ + p2_)/2);
}


void LineSegment::shrink(const Vec3f& p1, const Vec3f& p2, double shrink_distance) {
  for (auto &it : polyhedron_) {
    decimal_t b = it.p.dot(it.n);
    decimal_t d1 = it.n.dot(p1) - b;
    decimal_t d2 = it.n.dot(p2) - b;
    decimal_t d = -std::max(d1, d2) - 0.1;
    d = d < shrink_distance ? d : shrink_distance;
    if (d > 0.01)
      it.p -= d * it.n;
  }
}
