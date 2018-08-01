#include <decomp_util/seed_decomp.h>

SeedDecomp::SeedDecomp(const Vec3f& p): p_(p) {}

void SeedDecomp::set_virtual_dim(decimal_t x, decimal_t y, decimal_t z){
  virtual_x_ = x;
  virtual_y_ = y;
  virtual_z_ = z;
}

void SeedDecomp::set_obstacles(const vec_Vec3f& obs) {
  Polyhedron vs;
  add_virtual_wall(vs);
  obs_ = ps_in_polytope(vs, obs);
}

void SeedDecomp::dilate(decimal_t radius){
  dilate(Vec3f(radius, radius, radius), Mat3f::Identity());
}

void SeedDecomp::dilate(const Vec3f& axes, const Mat3f& R){
  Mat3f C = Mat3f::Identity();
  C(0, 0) = axes(0);
  C(1, 1) = axes(1);
  C(2, 2) = axes(2);
  C = R * C * R.transpose();
  Ellipsoid E = std::make_pair(C, p_);
  dilate(E);
}

void SeedDecomp::dilate(const Ellipsoid& E) {
  double radius = 1.0; // default radius is 1
  if(!obs_.empty()) {
    Face v = closest_obstacle(E, obs_);
    decimal_t b = v.p.dot(v.n);
    radius = std::abs(b - v.n.dot(p_)) / std::sqrt((v.n.dot(E.first * E.first.transpose()*v.n)));
    //printf("radius = %f\n", radius);
  }

  ellipsoid_.first = radius * E.first;
  ellipsoid_.second = E.second;

  //**** find half-space
  Polyhedron Vs;
  vec_Vec3f O_remain = obs_;
  while (!O_remain.empty()) {
    Face v = closest_obstacle(ellipsoid_, O_remain);
    Vs.push_back(v);
    Vec3f a = v.n;
    decimal_t b = v.p.dot(a);
    vec_Vec3f O_tmp;
    for (const auto &it : O_remain) {
      if (a.dot(it) - b < 0)
        O_tmp.push_back(it);
    }
    O_remain = O_tmp;
  }

  add_virtual_wall(Vs);
  polyhedron_ = Vs;
}


void SeedDecomp::shrink(decimal_t thr) {
  for (auto &it : polyhedron_) {
    decimal_t b = it.p.dot(it.n);
    decimal_t d = it.n.dot(p_) - b;
    d = -d;
    d = d < thr ? d : thr;
    if (d > 0.0)
      it.p -= d * it.n;
  }
}


void SeedDecomp::add_virtual_wall(Polyhedron &Vs) {
  //**** virtual walls x-y-z
  Vec3f dir = Vec3f(1, 0, 0);
  dir /= dir.norm();
  Vec3f dir_h(dir(1), -dir(0), 0);
  Vec3f pp1 = p_ + dir_h * virtual_y_;
  Vec3f pp2 = p_ - dir_h * virtual_y_;
  Vs.push_back(Face(pp1, dir_h));
  Vs.push_back(Face(pp2, -dir_h));

  Vec3f dir_v = dir.cross(dir_h);
  Vec3f pp3 = p_ + dir_v * virtual_z_;
  Vec3f pp4 = p_ - dir_v * virtual_z_;
  Vs.push_back(Face(pp3, dir_v));
  Vs.push_back(Face(pp4, -dir_v));

  Vec3f pp5 = p_ + dir * virtual_x_;
  Vec3f pp6 = p_ - dir * virtual_x_;
  Vs.push_back(Face(pp5, dir));
  Vs.push_back(Face(pp6, -dir));
}
