#include "solvers.hpp"

SolverAccel::SolverAccel()
{
  v_max_ = 20;
  a_max_ = 2;
  N_ = 15;
  int size = (int)(N_)*dt_ / DC;
  X_temp_ = Eigen::MatrixXd::Zero(size, 6);
  U_temp_ = Eigen::MatrixXd::Zero(size, 6);
  for (int i = 0; i < 6; i++)
  {
    xf_[i] = 0;
    x0_[i] = 0;
  }
}

void SolverAccel::genNewTraj()
{
  // printf("Replanning with x0= %f, %f, %f\n", nextQuadGoal_.pos.x, nextQuadGoal_.pos.y, nextQuadGoal_.pos.z);

  // printf("In genNewTraj\n");
  /*  */

  // double then = ros::Time::now().toSec();
  // Call optimizer
  callOptimizer();
  // ROS_WARN("solve time: %0.2f ms", 1000 * (ros::Time::now().toSec() - then));
  double** x = Accel::get_state();
  double** u = Accel::get_control();
  // then = ros::Time::now().toSec();
  // int size = (int)(N_)*dt_ / DC;
  // U_temp_ = Eigen::MatrixXd::Zero(size, 6);
  //  X_temp_ = Eigen::MatrixXd::Zero(size, 6);

  resetXandU();
  // if (dt_ != 0)
  //{
  //  interpolate(POS, ACCEL, u, x);    // interpolate POS when the input is acceleration*/
  // interpolate(VEL, ACCEL, u, x);    // ...
  // interpolate(ACCEL, ACCEL, u, x);  // ...
  // obtainByDerivation(u, x);         // ...
  //}
  //  printf("******MY INTERPOLATION\n");
  // std::cout << "X=\n" << X_temp_ << std::endl;
  // std::cout << "U=\n" << U_temp_ << std::endl;

  printf("*****CVXGEN\n");
  for (int i = 1; i < 15; i++)
  {
    printf("%0.2f  %0.2f  %0.2f\n", x[i][3], x[i][4], x[i][5]);
  }

  // printf("******BRETT'S INTERPOLATION\n");
  // U_temp_ = Eigen::MatrixXd::Zero(size, 6);
  // X_temp_ = Eigen::MatrixXd::Zero(size, 6);
  interpBRETT(dt_, xf_, u0_, x0_, u, x, U_temp_, X_temp_);
  // std::cout << "X=\n" << X_temp_ << std::endl;
  // std::cout << "U=\n" << U_temp_ << std::endl;
  // ROS_WARN("interp time: %0.2f ms", 1000 * (ros::Time::now().toSec() - then));

  // ROS_INFO("%0.2f %0.2f %0.2f", x[N_-1][0], x[N_-1][1], x[N_-1][2]);
  // ROS_INFO("%0.2f %0.2f %0.2f", X(X.rows()-1,0), X(X.rows()-1,1), X(X.rows()-1,2));
  // pubTraj(x);
}

void SolverAccel::interpBRETT(double dt, double xf[], double u0[], double x0[], double** u, double** x,
                              Eigen::MatrixXd& U, Eigen::MatrixXd& X)
{
  // printf("In interpInput\n");
  // linearly interpolate between control input from optimizer
  double dc = DC;
  int size = (int)(N_)*dt / dc;
  U = Eigen::MatrixXd::Zero(size, 6);
  X = Eigen::MatrixXd::Zero(size, 6);

  int j = 1;
  double s[3] = { (u[1][0] - u0[0]) / dt, (u[1][1] - u0[1]) / dt, (u[1][2] - u0[2]) / dt };

  for (int i = 0; i < size; i++)
  {
    if (i > 0 && dc * (i - 1) >= dt * (j))
    {
      j++;
      s[0] = (u[j][0] - u[j - 1][0]) / dt;
      s[1] = (u[j][1] - u[j - 1][1]) / dt;
      s[2] = (u[j][2] - u[j - 1][2]) / dt;
    }

    if (j == 1)
    {
      U(i, 0) = s[0] * (dc * i) + u0[0];
      U(i, 1) = s[1] * (dc * i) + u0[1];
      U(i, 2) = s[2] * (dc * i) + u0[2];
    }
    else
    {
      U(i, 0) = s[0] * (dc * i - dt * (j - 1)) + u[j - 1][0];
      U(i, 1) = s[1] * (dc * i - dt * (j - 1)) + u[j - 1][1];
      U(i, 2) = s[2] * (dc * i - dt * (j - 1)) + u[j - 1][2];
    }
  }

  for (int i = 0; i < size - 1; i++)
  {
    U(i, 3) = (U(i + 1, 0) - U(i, 0)) / dc;
    U(i, 4) = (U(i + 1, 1) - U(i, 1)) / dc;
    U(i, 5) = (U(i + 1, 2) - U(i, 2)) / dc;
  }

  U.row(size - 1) << 0, 0, 0, 0, 0, 0;

  int k = 1;
  double p[3] = { (x[1][0] - x0[0]) / dt, (x[1][1] - x0[1]) / dt, (x[1][2] - x0[2]) / dt };
  double v[3] = { (x[1][3] - x0[3]) / dt, (x[1][4] - x0[4]) / dt, (x[1][5] - x0[5]) / dt };

  for (int i = 0; i < size; i++)
  {
    if (i > 0 && dc * (i - 1) >= dt * k)
    {
      k++;
      p[0] = (x[k][0] - x[k - 1][0]) / dt;
      p[1] = (x[k][1] - x[k - 1][1]) / dt;
      p[2] = (x[k][2] - x[k - 1][2]) / dt;
      v[0] = (x[k][3] - x[k - 1][3]) / dt;
      v[1] = (x[k][4] - x[k - 1][4]) / dt;
      v[2] = (x[k][5] - x[k - 1][5]) / dt;
    }

    if (k == 1)
    {
      X(i, 0) = p[0] * (dc * i) + x0[0];
      X(i, 1) = p[1] * (dc * i) + x0[1];
      X(i, 2) = p[2] * (dc * i) + x0[2];
      X(i, 3) = v[0] * (dc * i) + x0[3];
      X(i, 4) = v[1] * (dc * i) + x0[4];
      X(i, 5) = v[2] * (dc * i) + x0[5];
    }
    else
    {
      X(i, 0) = p[0] * (dc * (i)-dt * (k - 1)) + x[k - 1][0];
      X(i, 1) = p[1] * (dc * (i)-dt * (k - 1)) + x[k - 1][1];
      X(i, 2) = p[2] * (dc * (i)-dt * (k - 1)) + x[k - 1][2];
      X(i, 3) = v[0] * (dc * (i)-dt * (k - 1)) + x[k - 1][3];
      X(i, 4) = v[1] * (dc * (i)-dt * (k - 1)) + x[k - 1][4];
      X(i, 5) = v[2] * (dc * (i)-dt * (k - 1)) + x[k - 1][5];
    }
  }

  for (int i = 0; i < 6; i++)
  {
    X(size - 1, i) = xf[i];
  }
}

void SolverAccel::callOptimizer()
{
  // printf("In callOptimizer\n");
  bool converged = false;
  // TODO: Be careful because u_max can be accel, jerk,...Also, there may be constraints in vel_max if input is accel
  float accelx = copysign(1, xf_[0] - x0_[0]) * a_max_;
  float accely = copysign(1, xf_[1] - x0_[1]) * a_max_;
  float accelz = copysign(1, xf_[2] - x0_[2]) * a_max_;
  float v0x = x0_[3];
  float v0y = x0_[4];
  float v0z = x0_[5];
  // printf("x0_[0]=%f\n", x0_[0]);
  // printf("xf_[0]=%f\n", xf_[0]);
  float tx =
      solvePolyOrder2(Eigen::Vector3f(0.5 * accelx, v0x, x0_[0] - xf_[0]));  // Solve equation xf=x0+v0t+0.5*a*t^2
  float ty = solvePolyOrder2(Eigen::Vector3f(0.5 * accely, v0y, x0_[1] - xf_[1]));
  float tz = solvePolyOrder2(Eigen::Vector3f(0.5 * accelz, v0z, x0_[2] - xf_[2]));
  float dt_initial = std::max({ tx, ty, tz }) / N_;
  if (dt_initial > 100)  // happens when there is no solution to the previous eq.
  {
    dt_initial = 0;
  }
  // ROS_INFO("dt I found= %0.2f", dt_found);
  double dt = dt_initial;  // 0.025
                           // ROS_INFO("empezando con, dt = %0.2f", dt);
  double** x;
  int i = 0;

  while (!converged)
  {
    Accel::load_default_data(dt, v_max_, a_max_, x0_, xf_);
    int r = Accel::optimize();
    i = i + 1;
    if (r == 1)
    {
      x = Accel::get_state();
      int s = checkConvergence(x[N_]);
      if (s == 1)
        converged = true;
      else
        dt += 0.025;
    }
    else
      dt += 0.025;
  }

  // ROS_INFO("Iterations = %d\n", i);
  // ROS_INFO("converged, dt = %0.2f", dt);
  // printf("difference= %0.2f\n", dt - dt_initial);
  dt_ = dt;
  // return dt;
}

int SolverAccel::checkConvergence(double xf_opt[])
{
  // printf("In checkConvergence\n");
  float d2 = 0;
  float dv2 = 0;

  for (int i = 0; i < 3; i++)
  {
    d2 += pow(xf_[i] - xf_opt[i], 2);
    dv2 += pow(xf_[i + 3] - xf_opt[i + 3], 2);
  }

  int r = (sqrt(d2) < 0.2 && sqrt(dv2) < 0.2) ? 1 : 0;

  return r;
}

// template class Solver<int INPUT_ORDER>;

/*template <int INPUT_ORDER>
class Solver;*/