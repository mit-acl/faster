#include <decomp_util/ellipsoid_utils.h>

Ellipsoid find_sphere(const Vec3f& pt, const vec_Vec3f& obs, vec_Vec3f& ps, decimal_t f) {
  ps.clear();
  Ellipsoid E = std::make_pair(f * Mat3f::Identity(), pt);

  vec_Vec3f Os = ps_in_ellipsoid(E, obs);
  if(Os.empty())
    return E;

  Vec3f closest_o = pt;
  decimal_t min_dist = f;
  for(const auto& it: Os)
  {
    if((it - pt).norm() < min_dist)
    {
      min_dist = (it - pt).norm();
      closest_o = it;
    }
  }

  E.first = std::min(min_dist, f) * Mat3f::Identity();

  return E;
}

Ellipsoid find_ellipsoid(const Vec3f& pt, const vec_Vec3f& obs, vec_Vec3f& ps, const Vec3f& fs) {
  ps.clear();
  Ellipsoid E = std::make_pair(Mat3f::Identity(), pt);
  E.first(0, 0) = fs(0);
  E.first(1, 1) = fs(1);
  E.first(2, 2) = fs(2);

  vec_Vec3f Os = ps_in_ellipsoid(E, obs);
  if(Os.empty())
    return E;

  Vec3f closest_o = pt;
  decimal_t min_dist = std::numeric_limits<decimal_t>::max();
  for(const auto& it: Os)
  {
    if((it - pt).norm() < min_dist)
    {
      min_dist = (it - pt).norm();
      closest_o = it;
    }
  }

  const Quatf qi = vec_to_quaternion(closest_o - pt);
  E.first(0,0) = min_dist;
  E.first = qi * E.first * qi.conjugate();
  Vec3f axes(min_dist, fs(1), fs(2));

  ps.push_back(closest_o);

  Os = ps_in_ellipsoid(E, Os);

  Vec3f pw = Vec3f::Zero();
  Quatf qf = qi;

  while (inside_ellipsoid(E, Os)) {
    int id = -1;
    if(!closest_pt(E, Os, pw, id))
      printf(ANSI_COLOR_RED "error!!\n" ANSI_COLOR_RESET);


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
  }
  ps.push_back(pw);

  //**** reset ellipsoid with old axes(2)
  E.first = Mat3f::Identity();
  E.first(0, 0) = axes(0);
  E.first(1, 1) = axes(1);
  E.first(2, 2) = axes(2);
  E.first = qf * E.first * qf.conjugate();
  Os = ps_in_ellipsoid(E, Os);
  while (inside_ellipsoid(E, Os)) {
    int id = -1;
    if(!closest_pt(E, Os, pw, id))
      printf(ANSI_COLOR_RED "error2!!\n" ANSI_COLOR_RESET);


    Vec3f p = qf.inverse() * (pw - E.second);
    axes(2) =
      fabs(p(2)) / sqrt(1 - pow(p(0) / axes(0), 2) - pow(p(1) / axes(1), 2));
    E.first = Mat3f::Identity();
    E.first(0, 0) = axes(0);
    E.first(1, 1) = axes(1);
    E.first(2, 2) = axes(2);
    E.first = qf * E.first * qf.conjugate();
  }
  ps.push_back(pw);

  return E;
}

bool max_ellipsoid(const Vec3f& pt, const Polyhedron& poly,
                   Ellipsoid& E, decimal_t max_radius) {
  if(!inside_polytope(pt, poly))
    return false;


  Polyhedron vs = poly;
  for(auto& it: vs)
    it.p = pt + dist(pt, it) * it.n;

  Face vb(Vec3f::Zero(), Vec3f::Zero());
  Vec3f axes(max_radius, max_radius, max_radius);
  Quatf q1 =  Quatf::Identity();
  if(estimate_ellipsoid(pt, vs, q1, 0, axes, vb))
    q1 = vec_to_quaternion(vb.n);
  else
    return false;

  Quatf q2 = q1;
  axes(1) = max_radius, axes(2) = max_radius;
  if(estimate_ellipsoid(pt, vs, q1, 1, axes, vb)){
    const decimal_t roll = atan2(vb.p(2), vb.p(1));
    q2 = q1 * Quatf(cos(roll / 2), sin(roll / 2), 0, 0);
  }
  else
    return false;

  axes(2) = max_radius;
  if(estimate_ellipsoid(pt, vs, q2, 2, axes, vb)){
    E.first = Mat3f::Identity();
    E.first(0, 0) = axes(0);
    E.first(1, 1) = axes(1);
    E.first(2, 2) = axes(2);
    E.first = q2 * E.first * q2.conjugate();
    E.second = pt;
    return true;
  }
  else
    return false;
}

bool estimate_ellipsoid(const Vec3f& pt, const Polyhedron& poly, const Quatf& q, int axes_id, Vec3f& axes, Face &vb) {
  Polyhedron inner_poly;
  for (const auto &it : poly) {
    Vec3f p = q.inverse() * (it.p - pt);
    Vec3f n = q.inverse() * it.n;
    decimal_t k = 1 /  sqrt(n(0)*n(0)*axes(0)*axes(0) +
                            n(1)*n(1)*axes(1)*axes(1) +
                            n(2)*n(2)*axes(2)*axes(2));
    Vec3f t(k*n(0)*axes(0)*axes(0),
            k*n(1)*axes(1)*axes(1),
            k*n(2)*axes(2)*axes(2));
    if((t - p).dot(n) > 0)
      inner_poly.push_back(Face(p, n));
  }

  decimal_t axes_length = 0;
  for(unsigned int i = 0; i < inner_poly.size(); i++) {
    Face v = inner_poly[i];
    decimal_t k = v.p.dot(v.n);
    Vec3f d = v.n;
    decimal_t axes_length_tmp = 0;
    if(axes_id == 0)
      axes_length_tmp = std::fabs(k);
    else if(axes_id == 1)
      axes_length_tmp = sqrt((k*k-axes(0)*axes(0)*d(0)*d(0)) / (d(1)*d(1) + d(2)*d(2)));
    else if(axes_id == 2)
      axes_length_tmp = sqrt((k*k-axes(0)*axes(0)*d(0)*d(0)-axes(1)*axes(1)*d(1)*d(1)) / (d(2)*d(2)));

    if(std::isinf(axes_length_tmp) || std::isnan(axes_length_tmp))
      continue;

    Vec3f axes_tmp = axes;
    if(axes_id == 0)
      axes_tmp(0) = axes_length_tmp, axes_tmp(1) = axes_length_tmp, axes_tmp(2) = axes_length_tmp;
    else if(axes_id == 1)
      axes_tmp(1) = axes_length_tmp, axes_tmp(2) = axes_length_tmp;
    else if(axes_id == 2)
      axes_tmp(2) = axes_length_tmp;

    if(axes_length_tmp > axes_length &&
       !has_inlier(pt, poly, q, axes_tmp)){
      axes_length = axes_length_tmp;
      axes = axes_tmp;
      vb = v;
    }
  }

  if(axes_length == 0 && !inner_poly.empty())
    return false;
  else
    return true;
}

bool has_inlier(const Vec3f& pt, const Polyhedron& vs,
                const Quatf& qf, const Vec3f axes) {
  for (const auto &it : vs) {
    Vec3f p = qf.inverse() * (it.p - pt);
    Vec3f n = qf.inverse() * it.n;
    decimal_t k = 1 /  sqrt(n(0)*n(0)*axes(0)*axes(0) + n(1)*n(1)*axes(1)*axes(1) + n(2)*n(2)*axes(2)*axes(2));
    Vec3f t(k*n(0)*axes(0)*axes(0),
            k*n(1)*axes(1)*axes(1),
            k*n(2)*axes(2)*axes(2));
    if((t - p).dot(n) - epsilon_ > 0)
    {
      /*
      std::cout << "p: " << it.first.transpose() << std::endl;
      std::cout << "n: " << it.second.transpose() << std::endl;
      std::cout << "t: " << t.transpose() << std::endl;
      std::cout << "axes: " << axes.transpose() << std::endl;
      printf("d: %f\n", (t - p).dot(n));
      */
     return true;
    }
  }
  return false;
}



