#include <motion_primitive_library/primitive/poly_solver.h>

template <int Dim>
PolySolver<Dim>::PolySolver(unsigned int smooth_derivative_order,
                            unsigned int minimize_derivative)
    : N_(2 * (smooth_derivative_order + 1)), R_(minimize_derivative),
      debug_(false) {
  ptraj_ = std::make_shared<PolyTraj<Dim>>();
}

template <int Dim>
std::shared_ptr<PolyTraj<Dim>> PolySolver<Dim>::getTrajectory() { return ptraj_; }

template <int Dim>
bool PolySolver<Dim>::solve(const vec_E<Waypoint<Dim>>& waypoints,
                            const std::vector<decimal_t>& dts) {
  ptraj_->clear();
  ptraj_->addTime(dts);

  const unsigned int num_waypoints = waypoints.size();
  const unsigned int num_segments = num_waypoints - 1;
  if(num_waypoints < 2)
    return false;

  MatDf A = MatDf::Zero(num_segments * N_, num_segments * N_);
  MatDf Q = MatDf::Zero(num_segments * N_, num_segments * N_);
  for (unsigned int i = 0; i < num_segments; i++) {
    decimal_t seg_time = dts[i];
    for (unsigned int n = 0; n < N_; n++) {
      // A_0
      if (n < N_ / 2) {
        int val = 1;
        for (unsigned int m = 0; m < n; m++)
          val *= (n - m);
        A(i * N_ + n, i * N_ + n) = val;
      }
      // A_T
      for (unsigned int r = 0; r < N_ / 2; r++) {
        if (r <= n) {
          int val = 1;
          for (unsigned int m = 0; m < r; m++)
            val *= (n - m);
          A(i * N_ + N_ / 2 + r, i * N_ + n) = val * power(seg_time, n - r);
        }
      }
      // Q
      for (unsigned int r = 0; r < N_; r++) {
        if (r >= R_ && n >= R_) {
          int val = 1;
          for (unsigned int m = 0; m < R_; m++)
            val *= (r - m) * (n - m);
          Q(i * N_ + r, i * N_ + n) = 2 * val *
                                      power(seg_time, r + n - 2 * R_ + 1) /
                                      (r + n - 2 * R_ + 1);
        }
      }
    }
  }

  const unsigned int num_fixed_derivatives = num_waypoints - 2 + N_;
  const unsigned int num_free_derivatives =
      num_waypoints * N_ / 2 - num_fixed_derivatives;
  if (debug_)
    printf("num_fixed_derivatives: %d, num_free_derivatives: %d\n",
           num_fixed_derivatives, num_free_derivatives);
  // M
  MatDf M = MatDf::Zero(num_segments * N_, num_waypoints * N_ / 2);
  M.block(0, 0, N_ / 2, N_ / 2) = MatDf::Identity(N_ / 2, N_ / 2);
  M.block(num_segments * N_ - N_ / 2, N_ / 2 + num_waypoints - 2, N_ / 2,
          N_ / 2) = MatDf::Identity(N_ / 2, N_ / 2);
  for (unsigned int i = 0; i < num_waypoints - 2; i++) {
    M((2 * i + 1) * N_ / 2, N_ / 2 + i) = 1;
    M((2 * i + 2) * N_ / 2, N_ / 2 + i) = 1;
    for (unsigned int j = 1; j < N_ / 2; j++) {
      M((2 * i + 1) * N_ / 2 + j,
        num_fixed_derivatives - 1 + i * (N_ / 2 - 1) + j) = 1;
      M((2 * i + 2) * N_ / 2 + j,
        num_fixed_derivatives - 1 + i * (N_ / 2 - 1) + j) = 1;
    }
  }
  // Eigen::MatrixXf A_inv = A.inverse();
  MatDf A_inv_M = A.partialPivLu().solve(M);
  MatDf R = A_inv_M.transpose() * Q * A_inv_M;
  if (debug_) {
    std::cout << "A:\n" << A << std::endl;
    std::cout << "Q:\n" << Q << std::endl;
    std::cout << "M:\n" << M << std::endl;
    // std::cout << "A_inv_M:\n" << A_inv_M << std::endl;
    std::cout << "R:\n" << R << std::endl;
  }
  MatDf Rpp = R.block(num_fixed_derivatives, num_fixed_derivatives,
                                num_free_derivatives, num_free_derivatives);
  MatDf Rpf = R.block(num_fixed_derivatives, 0, num_free_derivatives,
                                num_fixed_derivatives);

  // Fixed derivatives
  MatDNf<Dim> Df = MatDNf<Dim>(num_fixed_derivatives, Dim);
  // First point
  Df.row(0) = waypoints[0].pos.transpose();
  Df.row(1) = waypoints[0].vel.transpose();
  if (N_/2 > 2)
    Df.row(2) = waypoints[0].acc.transpose();
  if (N_/2 > 3) 
    Df.row(3) = waypoints[0].jrk.transpose();
  for (unsigned int i = 4; i < N_ / 2; i++) 
    Df.row(i) = Vecf<Dim>::Zero().transpose();

  // Middle waypoints
  for (unsigned int i = 1; i < num_waypoints - 1; i++) 
    Df.row((N_ / 2) - 1 + i) = waypoints[i].pos.transpose();

  // End point
  int end = num_waypoints + 1;
  Df.row(end) = waypoints[num_waypoints - 1].pos.transpose();
  Df.row(end+1) = waypoints[num_waypoints - 1].vel.transpose();
  if (N_ / 2 > 2) 
    Df.row(end+2) = waypoints[num_waypoints - 1].acc.transpose();
  if (N_ / 2 > 3) 
    Df.row(end+3) = waypoints[num_waypoints - 1].jrk.transpose();
  for (unsigned int i = 4; i < N_ / 2; i++) {
    Df.row(end+i) = Vecf<Dim>::Zero().transpose();
    // Df.row((N_ / 2) + (num_waypoints - 2) + i) = end_vel_.transpose();
  }
  if (debug_)
    std::cout << "Df:\n" << Df << std::endl;
  MatDNf<Dim> D = MatDNf<Dim>(num_waypoints * N_ / 2, Dim);
  D.topRows(num_fixed_derivatives) = Df;
  if (num_waypoints > 2 && num_free_derivatives > 0) {
    MatDNf<Dim> Dp = -Rpp.partialPivLu().solve(Rpf * Df);
    // std::cout << "Dp:\n" << Dp << std::endl;
    D.bottomRows(num_free_derivatives) = Dp;
  }
  MatDNf<Dim> d = M * D;
  if (debug_)
    std::cout << "d:\n" << d << std::endl;
  for (unsigned int i = 0; i < num_segments; i++) {
    const MatDNf<Dim> p = A.block(i * N_, i * N_, N_, N_)
      .partialPivLu()
      .solve(d.block(i * N_, 0, N_, Dim));
    if (debug_)
      std::cout << "p:\n" << p << std::endl;
    ptraj_->addCoeff(p);
  }
  return true;
}

template class PolySolver<2>;

template class PolySolver<3>;
