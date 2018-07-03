#include <decomp_util/iterative_decomp.h>

IterativeDecomp::IterativeDecomp(bool verbose) {
  has_bounding_box_ = false;
  verbose_ = verbose;
  if(verbose_)
    printf(ANSI_COLOR_GREEN "ITERATIVE DECOMP VERBOSE ON! \n" ANSI_COLOR_RESET);
}

IterativeDecomp::IterativeDecomp(const Vec3f &origin, const Vec3f &dim, bool verbose){
  has_bounding_box_ = true;
  min_ = origin;
  max_ = origin + dim;
  verbose_ = verbose;
  if(verbose_){
    printf(ANSI_COLOR_GREEN "ITERATIVE DECOMP VERBOSE ON! \n" ANSI_COLOR_RESET);
    printf("Min: [%f, %f, %f]\n", min_(0), min_(1), min_(2));
    printf("Max: [%f, %f, %f]\n", max_(0), max_(1), max_(2));
  }
}

bool IterativeDecomp::decomp_iter(const vec_Vec3f& poses, int iter_num, double offset_x){
  vec_Vec3f path = poses;
  int cnt = 0;
  for(int i = 0; i < iter_num; i++){
    if(!decomp(path, offset_x))
      return false;
    cnt ++;
    vec_Vec3f new_path;
    if(1)
      new_path = simplify(center_path_);
    else
      new_path = center_path_;
    bool converge = false;
    if(new_path.size() == path.size()){
      converge = true;
      for(int j = 0; j < (int) path.size(); j++){
        if((new_path[j]-path[j]).norm() > 2e-1){
          converge = false;
          break;
        }
      }
    }
    if(converge)
      break;
    else
      path = new_path;
  }

  if(verbose_)
    printf(ANSI_COLOR_GREEN "takes %d iteration out of %d iteration num\n" ANSI_COLOR_RESET, cnt, iter_num);
  return true;

}

vec_Vec3f IterativeDecomp::simplify(const vec_Vec3f& path){
  if(path.size() <= 2)
    return path;

  Vec3f ref_pt = path.front();
  vec_Vec3f new_path;
  new_path.push_back(ref_pt);

  for(int i = 2; i < (int) path.size(); i ++){
    if(inside_polytope(ref_pt, polyhedrons_[i-1], 0) &&
        cal_closest_dist(ref_pt, polyhedrons_[i-1]) > 1e-1) {
      if(verbose_)
        printf(ANSI_COLOR_GREEN "remove intermediate waypoints\n" ANSI_COLOR_RESET);
    }
    else{
      ref_pt = path[i-1];
      new_path.push_back(ref_pt);
    }
  }
  new_path.push_back(path.back());
  return new_path;
}

