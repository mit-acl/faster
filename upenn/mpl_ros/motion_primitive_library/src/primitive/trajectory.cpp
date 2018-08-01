#include <motion_primitive_library/primitive/trajectory.h>

//********** Lambda Seg ****************
LambdaSeg::LambdaSeg(const VirtualPoint& v1, const VirtualPoint& v2) {
  Mat4f A;
  A << v1.t*v1.t*v1.t, v1.t*v1.t, v1.t, 1,
    3*v1.t*v1.t, 2*v1.t, 1, 0,
    v2.t*v2.t*v2.t, v2.t*v2.t, v2.t, 1,
    3*v2.t*v2.t, 2*v2.t, 1, 0;
  Vec4f b;
  b << v1.p, v1.v, v2.p, v2.v;

  a = A.inverse() * b;


  if(fabs(a(0)) < 1e-5)
    a(0)= 0;
  if(fabs(a(1)) < 1e-5)
    a(1)= 0;
  if(fabs(a(2)) < 1e-5)
    a(2)= 0;
  if(fabs(a(3)) < 1e-5)
    a(3)= 0;

  ti = v1.t;
  tf = v2.t;

  dT = getT(tf) - getT(ti);
}

decimal_t LambdaSeg::getT(decimal_t t) const {
  return a(0)/4*t*t*t*t+a(1)/3*t*t*t+
    a(2)/2*t*t+a(3)*t;
}

VirtualPoint LambdaSeg::evaluate(decimal_t tau) const {
  VirtualPoint vt;
  vt.t = tau;
  vt.p = a(0)*tau*tau*tau+a(1)*tau*tau+a(2)*tau+a(3);
  vt.v = 3*a(0)*tau*tau+2*a(1)*tau+a(2);
  return vt;
}

//********** Lambda ********************
Lambda::Lambda(const std::vector<VirtualPoint>& vs) {
  for(int i = 0; i < (int)vs.size() - 1; i++) {
    LambdaSeg seg(vs[i], vs[i+1]);
    segs.push_back(seg);
  }
}

VirtualPoint Lambda::evaluate(decimal_t tau) const {
  VirtualPoint vt;
  for(const auto& seg: segs) {
    if(tau >= seg.ti && tau < seg.tf) {
      vt = seg.evaluate(tau);
      break;
    }
  }
  return vt;
}

std::vector<VirtualPoint> Lambda::sample(int N) {
  // sample N points
  std::vector<VirtualPoint> vs;
  if(segs.empty())
    return vs;
  decimal_t ti = segs.front().ti;
  decimal_t tf = segs.back().tf;
  decimal_t dt = (tf - ti) / N;
  for(decimal_t t = ti; t <= tf; t+=dt) {
    for(const auto& seg: segs) {
      if(t >= seg.ti && t < seg.tf) {
        vs.push_back(seg.evaluate(t));
        break;
      }
    }
  }

  return vs;
}

vec_Vec3f Lambda::sampleT(int N) {
  vec_Vec3f ts;
  decimal_t ti = segs.front().ti;
  decimal_t tf = segs.back().tf;
  decimal_t dt = (tf-ti)/N;

  for(decimal_t t = ti; t <= tf; t+=dt)
    ts.push_back(Vec3f(t, getT(t), 0));

  return ts;
}

decimal_t Lambda::getTotalTime() const {
  decimal_t t = 0;
  for(const auto& seg: segs)
    t += seg.dT;

  return t;
}

decimal_t Lambda::getT(decimal_t tau) const {
  decimal_t T = 0;
  for(const auto& seg: segs) {
    if(tau >= seg.ti && tau <= seg.tf) {
      T += seg.getT(tau) - seg.getT(seg.ti);
      return T;
    }
    T += seg.dT;
  }


  return tau;
}


decimal_t Lambda::getTau(decimal_t t) const {
  if(!exist())
    return t;
  decimal_t T = 0;
  for(const auto& seg: segs) {
    if(t >= T && t <= T + seg.dT) {
      decimal_t a  = seg.a(0) / 4;
      decimal_t b  = seg.a(1) / 3;
      decimal_t c  = seg.a(2) / 2;
      decimal_t d  = seg.a(3);
      decimal_t e  = T-t-seg.getT(seg.ti);

      std::vector<decimal_t> ts = solve(a, b, c, d, e);
      for(const auto &it: ts) {
        if(it >= seg.ti && it <= seg.tf)
          return it;
      }
    }
    T += seg.dT;
  }

  printf("error: cannot find tau, t = %f\n", t);
  return -1;
}

bool Lambda::exist() const{
  return !segs.empty();
}


//*********** Trajectory *********************
template <int Dim>
Trajectory<Dim>::Trajectory(const vec_E<Primitive<Dim>>& prs) {
  // Constructor from multiple primitives
  segs = prs;

  taus.push_back(0);
  for(const auto& pr: prs) 
    taus.push_back(pr.t()+taus.back());
  Ts = taus;
  total_t_ = taus.back();
}

template <int Dim>
decimal_t Trajectory<Dim>::getTotalTime() const {
  return total_t_;
}

template <int Dim>
bool Trajectory<Dim>::scale_down(decimal_t mv, decimal_t ri, decimal_t rf) {
  std::vector<VirtualPoint> vs;
  VirtualPoint vi, vf;
  vi.p = ri;
  vi.v = 0;
  vi.t = 0;

  vf.p = rf;
  vf.v = 0;
  vf.t = taus.back();

  vs.push_back(vi);
  for(int id = 0; id < (int)segs.size(); id++) {
    for(int i = 0; i < 3; i++) {
      if(segs[id].max_vel(i) > mv) {
        std::vector<decimal_t> ts = segs[id].traj(i).extrema_vel(segs[id].t());
        if(id != 0)
          ts.push_back(0);
        ts.push_back(segs[id].t());
        for(const auto& tv: ts){
          Vec4f p = segs[id].traj(i).evaluate(tv);
          decimal_t v = p(1);
          decimal_t lambda_v = fabs(v) / mv;
          if(lambda_v <= 1)
            continue;

          VirtualPoint vt;
          vt.p = lambda_v;
          vt.v = 0;
          vt.t = tv+taus[id];
          vs.push_back(vt);
        }
      }
    }
  }

  vs.push_back(vf);

  std::sort(vs.begin(), vs.end(), [](const VirtualPoint& i, const VirtualPoint& j){return i.t < j.t;});
  decimal_t max_l = 1;
  for(const auto& v: vs) {
    if(v.p > max_l)
      max_l = v.p;
  }

  if(max_l <= 1)
    return false;

  //printf("max_l: %f\n", max_l);
  for(int i = 1; i < (int)vs.size()-1; i++)
    vs[i].p = max_l;
  std::vector<VirtualPoint> vs_s;
  vs_s.push_back(vs.front());
  for(const auto& v: vs)
    if(v.t > vs_s.back().t)
      vs_s.push_back(v);

  lambda_ = Lambda(vs_s);

  std::vector<decimal_t> ts;
  for(const auto& tau: taus)
    ts.push_back(lambda_.getT(tau));
  Ts = ts;
  total_t_ = Ts.back();
  return true;

}


template <int Dim>
bool Trajectory<Dim>::scale(decimal_t ri, decimal_t rf) {
  std::vector<VirtualPoint> vs;
  VirtualPoint vi, vf;
  vi.p = 1.0/ri;
  vi.v = 0;
  vi.t = 0;

  vf.p = 1.0/rf;
  vf.v = 0;
  vf.t = taus.back();

  vs.push_back(vi);
  vs.push_back(vf);
  Lambda ls(vs);
  lambda_ = ls;
  std::vector<decimal_t> ts;
  for(const auto& tau: taus)
    ts.push_back(lambda_.getT(tau));
  Ts = ts;
  total_t_ = Ts.back();
  return true;
}


template <int Dim>
bool Trajectory<Dim>::evaluate(decimal_t time, Waypoint<Dim>& p) const {
  decimal_t tau = lambda_.getTau(time);
  if(tau < 0)
    tau = 0;
  if(tau > total_t_)
    tau = total_t_;

  decimal_t lambda = 1;
  decimal_t lambda_dot = 0;

  if(lambda_.exist()) {
    VirtualPoint vt = lambda_.evaluate(tau);
    lambda = vt.p;
    lambda_dot = vt.v;
  }

  for(int id = 0; id < (int) segs.size(); id++) {
    if(tau >= taus[id] && tau <= taus[id+1]) {
      tau -= taus[id];
      for(int j = 0; j < Dim; j++) {
        Vec4f d = segs[id].traj(j).evaluate(tau);
        p.pos(j) = d(0);
        p.vel(j) = d(1)/lambda;
        p.acc(j) = d(2)/lambda/lambda-d(1)*lambda_dot/lambda/lambda/lambda;
        p.jrk(j) = d(3)/lambda/lambda-3/power(lambda, 3)*d(2)*d(2)*lambda_dot+3/power(lambda,4)*d(1)*lambda_dot*lambda_dot; // assume lambda_ddot = 0
      }
      return true;

    }
  }

  printf("cannot find tau according to time: %f\n", time);
  return false;
}

template <int Dim>
vec_E<Waypoint<Dim>> Trajectory<Dim>::sample(int N) const {
  vec_E<Waypoint<Dim>> ps;

  decimal_t dt = total_t_ / N;
  for(decimal_t t = 0; t <= total_t_; t+= dt) {
    Waypoint<Dim> pt;
    if(evaluate(t, pt))
      ps.push_back(pt);
  }

  return ps;
}


template <int Dim>
Lambda Trajectory<Dim>::lambda() const {
  return lambda_;
}

template <int Dim>
decimal_t Trajectory<Dim>::J(int i) const {
  decimal_t J = 0;
  for(const auto& seg: segs)
    J += seg.J(i);
  return J;
}

template <int Dim>
std::vector<decimal_t> Trajectory<Dim>::getSegsT() const {
  std::vector<decimal_t> dts;
  for(int i = 0; i < (int)Ts.size() - 1; i++)
    dts.push_back(Ts[i+1] - Ts[i]);

  return dts;
}

template class Trajectory<2>;

template class Trajectory<3>;
