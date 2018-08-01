#include <decomp_util/ellipse_decomp.h>

EllipseDecomp::EllipseDecomp(bool verbose) {
  has_bounding_box_ = false;
  verbose_ = verbose;
  if(verbose_)
    printf(ANSI_COLOR_GREEN "DECOMP VERBOSE ON! \n" ANSI_COLOR_RESET);
}

EllipseDecomp::EllipseDecomp(const Vec3f &origin, const Vec3f &dim, bool verbose){
  has_bounding_box_ = true;
  min_ = origin;
  max_ = origin + dim;
  verbose_ = verbose;
  if(verbose_) {
    printf(ANSI_COLOR_GREEN "DECOMP VERBOSE ON! \n" ANSI_COLOR_RESET);
    printf("Min: [%f, %f, %f]\n", min_(0), min_(1), min_(2));
    printf("Max: [%f, %f, %f]\n", max_(0), max_(1), max_(2));
  }
}

void EllipseDecomp::clean() {
  clear();
  obs_.clear();
}

void EllipseDecomp::clear() {
  lines_.clear();
  ellipsoids_.clear();
  polyhedrons_.clear();
  intersect_polyhedrons_.clear();
  dilate_path_.clear();
  center_path_.clear();
}

vec_LinearConstraint3f EllipseDecomp::get_constraints(){
  vec_LinearConstraint3f constraints;
  constraints.resize(polyhedrons_.size());
  for (unsigned int i = 0; i < polyhedrons_.size(); i++){
    Vec3f pt = (center_path_[i] + center_path_[i+1])/2;
    constraints[i] = cal_Axb(pt, polyhedrons_[i]);
  }
  return constraints;
}

void EllipseDecomp::add_bounding(Polyhedron &Vs) {
  //**** add bound along X, Y, Z axis
  //*** Z
  Vs.push_back(Face(Vec3f(0, 0, max_(2)), Vec3f(0, 0, 1)));
  Vs.push_back(Face(Vec3f(0, 0, min_(2)), Vec3f(0, 0, -1)));

  //*** X
  Vs.push_back(Face(Vec3f(max_(0), 0, 0), Vec3f(1, 0, 0)));
  Vs.push_back(Face(Vec3f(min_(0), 0, 0), Vec3f(-1, 0, 0)));
  //*** Y
  Vs.push_back(Face(Vec3f(0, max_(1), 0), Vec3f(0, 1, 0)));
  Vs.push_back(Face(Vec3f(0, min_(1), 0), Vec3f(0, -1, 0)));
}

bool EllipseDecomp::decomp(const vec_Vec3f &poses, double offset_x) {
  clear();
  if (poses.size() < 2)
  {
    if(verbose_)
      printf(ANSI_COLOR_RED "Decomp failed, poses size: %zu\n" ANSI_COLOR_RESET, poses.size());
    return false;
  }

  unsigned int N = poses.size() - 1;
  lines_.resize(N);
  ellipsoids_.resize(N);
  polyhedrons_.resize(N);
  intersect_polyhedrons_.resize(N-1);

  for (unsigned int i = 0; i < N; i++) {
    lines_[i] = std::make_shared<LineSegment>(poses[i], poses[i+1]);
    if(virtual_.norm() > 0)
      lines_[i]->set_virtual_dim(virtual_(0), virtual_(1), virtual_(2));
    lines_[i]->set_obstacles(obs_);
    lines_[i]->dilate(offset_x);

    ellipsoids_[i] = lines_[i]->ellipsoid();
    polyhedrons_[i] = lines_[i]->polyhedron();
  }

  for (unsigned int i = 0; i < poses.size() - 2; i++){
    intersect_polyhedrons_[i] = polytope_intersection(polyhedrons_[i], polyhedrons_[i+1]);
    if(has_bounding_box_)
      add_bounding(intersect_polyhedrons_[i]);
  }

  dilate_path_ = poses;
  center_path_ = cal_centers(intersect_polyhedrons_);
  center_path_.push_back(poses.back());
  center_path_.insert(center_path_.begin(), poses.front());

  if(has_bounding_box_) {
    for(unsigned int i = 0; i < lines_.size(); i++) {
      polyhedrons_[i] = lines_[i]->polyhedron();
      add_bounding(polyhedrons_[i]);
    }
  }

  return true;
}

void EllipseDecomp::shrink(const vec_Vec3f& path, double shrink_distance) {
  if(shrink_distance <= 0)
    return;
  for(unsigned int i = 0; i < lines_.size(); i++) {
    lines_[i]->shrink(path[i], path[i+1], shrink_distance);
    polyhedrons_[i] = lines_[i]->polyhedron();
    if(has_bounding_box_)
      add_bounding(polyhedrons_[i]);
  }
}

vec_Vec3f EllipseDecomp::cal_centers(const Polyhedra &intersect_vs) {
  vec_Vec3f path;
  for (unsigned int i = 0; i < intersect_vs.size(); i++) {
    Polyhedron vs = intersect_vs[i];

    Vec3f pt = dilate_path_[i+1];
    if(!inside_polytope(pt, vs))
      continue;

    vec_E<vec_Vec3f> bs = cal_extreme_points(vs);
    Vec3f avg = Vec3f::Zero();
    int cnt = 0;
    for (unsigned int j = 0; j < bs.size(); j++) {
      for (const auto &it : bs[j]) {
        avg += it;
        cnt++;
      }
    }
    //*** use average point as initial guess
    avg /= cnt;
#if USE_CENTROID
    Vec3f C = cal_centroid_3d(bs, avg);
#else
    Mat3f init_E;
    init_E << 0.01, 0, 0,
           0, 0.01, 0,
           0, 0, 0.01;
    Vec3f init_d = avg;
    ellipse_Vec3f e = std::make_pair(init_E, init_d);

    Vec3f C = avg;
    LinearConstraint3f Cs = cal_Axb(e.second, vs);
    if(max_ellipsoid(Cs, e)){
      C = e.second;
      ellipses_.push_back(e);
    }
#endif

    if (!inside_polytope(C, vs, 0)) // TODO: SOME WEIRD THING Happen
    {
      if(verbose_)
        std::cout << " C: " << C.transpose() << std::endl;
      path.push_back(avg);
    } else
      path.push_back(C);
  }

  /*
  double z = path.front()(2);
  for(auto &it: path)
    it(2) = z;
    */

  return path;
}

decimal_t EllipseDecomp::get_corridor_volume() {
  decimal_t V = 0;
  for (const auto &it: lines_)
    V += it->polyhedron_volume();

  for (unsigned int i = 0; i < intersect_polyhedrons_.size(); i++) {
    vec_E<vec_Vec3f> bs = cal_extreme_points(intersect_polyhedrons_[i]);
    V -= cal_volume(bs, center_path_[i+1]);
  }

  return V;
}

decimal_t EllipseDecomp::get_ellipsoid_volume() {
  decimal_t V = 0;
  for (const auto &it: lines_)
    V += it->ellipsoid_volume();
  return V;
}

