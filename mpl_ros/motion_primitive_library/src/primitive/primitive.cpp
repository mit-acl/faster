#include <motion_primitive_library/primitive/primitive.h>

//******* Primitive 1D Base Class ***********
Primitive1D::Primitive1D() {}

Primitive1D::Primitive1D(const Vec6f& coeff) : c(coeff) {}

Primitive1D::Primitive1D(decimal_t p1, decimal_t v1, decimal_t a1,
                         decimal_t p2, decimal_t v2, decimal_t a2, decimal_t t) {
  Mat6f A;
  A << 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 1, 0, 0,
    t*t*t*t*t/120, t*t*t*t/24, t*t*t/6,t*t/2, t, 1,
    t*t*t*t/24, t*t*t/6, t*t/2, t, 1, 0,
    t*t*t/6, t*t/2, t, 1, 0, 0;
  Vec6f b;
  b << p1, v1, a1, p2, v2, a2;
  c = A.inverse() * b;
}


Primitive1D::Primitive1D(decimal_t p1, decimal_t v1,
                         decimal_t p2, decimal_t v2, decimal_t t) {
  Mat4f A;
  A << 0, 0, 0, 1,
    0, 0, 1, 0,
    t*t*t/6,t*t/2, t, 1,
    t*t/2, t, 1, 0;
  Vec4f b;
  b << p1, v1, p2, v2;
  Vec4f cc = A.inverse() * b;
  c << 0, 0, cc(0), cc(1), cc(2), cc(3);
}

Primitive1D::Primitive1D(decimal_t p1, decimal_t p2, decimal_t t) {
 c << 0, 0, 0, 0, (p2-p1)/t, p1;
}

Vec4f Primitive1D::evaluate(decimal_t t) const {
  Vec4f vec;
  vec << c(0)/120*t*t*t*t*t+c(1)/24*t*t*t*t+c(2)/6*t*t*t+c(3)/2*t*t+c(4)*t+c(5),
      c(0)/24*t*t*t*t+c(1)/6*t*t*t+c(2)/2*t*t+c(3)*t+c(4),
      c(0)/6*t*t*t+c(1)/2*t*t+c(2)*t+c(3),
      c(0)/2*t*t+c(1)*t+c(2);

  return vec;
}

decimal_t Primitive1D::J(decimal_t t, int i) const {
  // i = 1, return integration of square of vel
  if(i == 1)
    return c(0)*c(0)/5184*power(t,9)+c(0)*c(1)/576*power(t,8)+(c(1)*c(1)/252+c(0)*c(2)/168)*power(t,7)+
      (c(0)*c(3)/72+c(1)*c(2)/36)*power(t,6)+(c(2)*c(2)/20+c(0)*c(4)/60+c(1)*c(3)/15)*power(t,5)+
      (c(2)*c(3)/4+c(1)*c(4)/12)*power(t,4)+(c(3)*c(3)/3+c(2)*c(4)/3)*t*t*t+c(3)*c(4)*t*t+c(4)*c(4)*t;
  // i = 2, return integration of square of acc
  else if(i == 2)
    return c(0)*c(0)/252*t*t*t*t*t*t*t+c(0)*c(1)/36*t*t*t*t*t*t+(c(1)*c(1)/20+c(0)*c(2)/15)*t*t*t*t*t+
      (c(0)*c(3)/12+c(1)*c(2)/4)*t*t*t*t+(c(2)*c(2)/3+c(1)*c(3)/3)*t*t*t+
      c(2)*c(3)*t*t+c(3)*c(3)*t;
  // i = 3, return integration of square of jerk
  else if(i == 3)
    return c(2)*c(2)*t+c(1)*c(2)*t*t+(c(1)*c(1)+c(0)*c(2))/3*t*t*t+
      c(0)*c(1)/4*t*t*t*t+c(0)*c(0)/20*t*t*t*t*t;
  // i = 4, return integration of square of snap
  else if(i == 4)
    return c(0)*c(0)/3.*t*t*t+c(0)*c(1)*t*t+c(1)*c(1)*t;
  else
    return 0;
}

Vec6f Primitive1D::coeff() const {
  return c;
}

//********** Primitive 1D Vel Class ***********
Primitive1D::Primitive1D(decimal_t p, decimal_t u) {
  c << 0, 0, 0, 0, u, p;
}

//********** Primitive 1D Acc Class ***********
Primitive1D::Primitive1D(Vec2f state, decimal_t u) {
  c << 0, 0, 0, u, state(1), state(0);
}

//********** Primitive 1D Jrk Class ***********
Primitive1D::Primitive1D(Vec3f state, decimal_t u) {
  c << 0, 0, u, state(2), state(1), state(0);
}

//********** Primitive 1D Snap Class ***********
Primitive1D::Primitive1D(Vec4f state, decimal_t u) {
  c << 0, u, state(3), state(2), state(1), state(0);
}


std::vector<decimal_t> Primitive1D::extrema_vel(decimal_t t) const {
  std::vector<decimal_t> ts = solve(0, c(0)/6, c(1)/2, c(2), c(3));
  std::vector<decimal_t> ts_max;
  for(const auto &it: ts) {
    if(it > 0 && it < t)
      ts_max.push_back(it);
  }
  return ts_max;
}

std::vector<decimal_t> Primitive1D::extrema_acc(decimal_t t) const {
  std::vector<decimal_t> ts = solve(0, 0, c(0)/2, c(1), c(2));
  std::vector<decimal_t> ts_max;
  for(const auto &it: ts) {
    if(it > 0 && it < t)
      ts_max.push_back(it);
  }
  return ts_max;
}

std::vector<decimal_t> Primitive1D::extrema_jrk(decimal_t t) const {
  std::vector<decimal_t> ts_max;
  if(c(0) != 0) {
    decimal_t t = -c(1)*2/c(0);
    if(t > 0 && t < t)
      ts_max.push_back(t);
  }
  return ts_max;
}


//********** Primitive Main Class *************
template <int Dim>
Primitive<Dim>::Primitive() {}

template <int Dim>
Primitive<Dim>::Primitive(const vec_E<Vec6f>& cs, decimal_t t) : t_(t)
{
  // Constructor from coeffs
  for(int i = 0; i < Dim; i++)
    prs_[i] = Primitive1D(cs[i]);
}


template <int Dim>
Primitive<Dim>::Primitive(const Waypoint<Dim>& p, const Vecf<Dim>& u, decimal_t t) : t_(t)
{
  if(p.use_jrk) {
    for(int i = 0; i < Dim; i++) {
      Vec4f vec;
      vec << p.pos(i), p.vel(i), p.acc(i), p.jrk(i);
      prs_[i] = Primitive1D(vec, u(i));
    }
  }
  else if(p.use_acc) {
   for(int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(Vec3f(p.pos(i), p.vel(i), p.acc(i)), u(i));
  }
  else if(p.use_vel) {
    for(int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(Vec2f(p.pos(i), p.vel(i)), u(i));
  }
  else if(p.use_pos) {
    for(int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(p.pos(i), u(i));
  }
  else
    printf("Null Primitive using control!\n");
}

template <int Dim>
Primitive<Dim>::Primitive(const Waypoint<Dim>& p1, const Waypoint<Dim>& p2, decimal_t t) : t_(t)
{
  // Constructor from Two Waypoints
  // Fully contrained
  if(p1.use_pos && p1.use_vel && p1.use_acc && !p1.use_jrk &&
     p2.use_pos && p2.use_vel && p2.use_acc && !p2.use_jrk) {
    for(int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(p1.pos(i), p1.vel(i), p1.acc(i),
                            p2.pos(i), p2.vel(i), p2.acc(i), t_);
  }
  // Use vel only
  else if(p1.use_pos && p1.use_vel && !p1.use_acc && !p1.use_jrk &&
          p2.use_pos && p2.use_vel && !p2.use_acc && !p2.use_jrk) {
    for(int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(p1.pos(i), p1.vel(i),
                              p2.pos(i), p2.vel(i), t_);
  }
  // Use pos only
  else if(p1.use_pos && !p1.use_vel && !p1.use_acc && !p1.use_jrk &&
          p2.use_pos && !p2.use_vel && !p2.use_acc && !p2.use_jrk) {
    for(int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(p1.pos(i), p2.pos(i), t_);
  }
  // Null
  else {
    printf("Null Primitive using states!\n");
    p1.print();
    p2.print();
  }
}

template <int Dim>
decimal_t Primitive<Dim>::max_vel(int k) const {
  if(k >= Dim)
    return 0;

  std::vector<decimal_t> ts = prs_[k].extrema_vel(t_);
  Vec4f p1 = prs_[k].evaluate(0);
  Vec4f p2 = prs_[k].evaluate(t_);
  decimal_t max_v = std::max(fabs(p1(1)), fabs(p2(1)));

  for(const auto &it: ts) {
    if(it > 0 && it < t_){
      Vec4f p3 = prs_[k].evaluate(it);
      if(fabs(p3(1)) > max_v)
        max_v = fabs(p3(1));
    }
  }
  return max_v;
}

template <int Dim>
decimal_t Primitive<Dim>::max_acc(int k) const {
  if(k >= Dim)
    return 0;

  std::vector<decimal_t> ts = prs_[k].extrema_acc(t_);
  Vec4f p1 = prs_[k].evaluate(0);
  Vec4f p2 = prs_[k].evaluate(t_);
  decimal_t max_a = std::max(fabs(p1(2)), fabs(p2(2)));

  for(const auto &it: ts) {
    if(it > 0 && it < t_){
      Vec4f p3 = prs_[k].evaluate(it);
      if(fabs(p3(2)) > max_a)
        max_a = fabs(p3(2));
    }
  }

  return max_a;
}

template <int Dim>
decimal_t Primitive<Dim>::max_jrk(int k) const {
  if(k >= Dim)
    return 0;

  std::vector<decimal_t> ts = prs_[k].extrema_jrk(t_);
  Vec4f p1 = prs_[k].evaluate(0);
  Vec4f p2 = prs_[k].evaluate(t_);
  decimal_t max_j = std::max(fabs(p1(3)), fabs(p2(3)));

  for(const auto &it: ts) {
    if(it > 0 && it < t_){
      Vec4f p3 = prs_[k].evaluate(it);
      if(fabs(p3(3)) > max_j)
        max_j = fabs(p3(3));
    }
  }

  return max_j;
}


template <int Dim>
bool Primitive<Dim>::valid_vel(decimal_t mv) const {
  // ignore negative threshold
  if(mv < 0)
    return true;
  // check if max vel is violating the constraint
  for(int i = 0; i < Dim; i++) {
    if(max_vel(i) > mv)
      return false;
  }
  return true;
}

template <int Dim>
bool Primitive<Dim>::valid_acc(decimal_t ma) const {
  // ignore negative threshold
  if(ma < 0)
    return true;
  // check if max acc is violating the constraint
  for(int i = 0; i < Dim; i++) {
    if(max_acc(i) > ma)
      return false;
  }
  return true;
}

template <int Dim>
bool Primitive<Dim>::valid_jrk(decimal_t mj) const {
  // ignore negative threshold
  if(mj < 0)
    return true;
  // check if max jerk is violating the constraint
  for(int i = 0; i < Dim; i++) {
    if(max_jrk(i) > mj)
      return false;
  }
  return true;
}


template <int Dim>
Waypoint<Dim> Primitive<Dim>::evaluate(decimal_t t) const {
  Waypoint<Dim> p;
  for(int j = 0; j < Dim; j++) {
    Vec4f d = prs_[j].evaluate(t);
    p.pos(j) = d(0);
    p.vel(j) = d(1);
    p.acc(j) = d(2);
    p.jrk(j) = d(3);
  }
  return p;
}


template <int Dim>
vec_E<Waypoint<Dim>> Primitive<Dim>::sample(int N) const {
  vec_E<Waypoint<Dim>> ps;
  decimal_t dt = t_ / N;
  /*
  for(decimal_t t = 0; t <= t_; t+= dt)
      ps.push_back(evaluate(t));
      */
  for(int i = 0; i <= N; i ++)
      ps.push_back(evaluate(i*dt));
 
  return ps;
}

template <int Dim>
Primitive1D Primitive<Dim>::traj(int k) const { return prs_[k]; }

template <int Dim>
decimal_t Primitive<Dim>::t() const { return t_; }

template <int Dim>
decimal_t Primitive<Dim>::J(int i) const {
  decimal_t j = 0;
  for(const auto& pr: prs_)
    j += pr.J(t_, i);
  return j;
}

template <int Dim>
vec_E<Vec6f> Primitive<Dim>::coeffs() const {
  vec_E<Vec6f> cs;
  for(const auto& pr: prs_)
    cs.push_back(pr.coeff());
  return cs;
}

template class Primitive<2>;

template class Primitive<3>;
