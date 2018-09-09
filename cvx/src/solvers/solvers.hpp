#ifndef SOLVERS_HPP
#define SOLVERS_HPP
#include <Eigen/Dense>
#include "../utils.hpp"
#include "mlinterp.hpp"

#include "cvxgen/interface_vel.h"
#include "cvxgen/interface_accel.h"
#include "cvxgen/interface_jerk.h"

#include <unsupported/Eigen/Polynomials>

template <int INPUT_ORDER>
class Solver
{
public:
  Solver();
  void interpolate(int var, double** u, double** x);
  void obtainByDerivation(double** u, double** x);
  double getCost();
  Eigen::MatrixXd getX();
  Eigen::MatrixXd getU();
  void set_x0(double x0[]);
  // void set_u0(double u0[]);
  void set_xf(double xf[]);
  void resetXandU();
  void set_max(double max_values[INPUT_ORDER]);
  bool checkConvergence(double xf_opt[3 * INPUT_ORDER]);
  void genNewTraj();
  void callOptimizer();
  double getDTInitial();
  int getOrder();
  int getN();
  void setDC(double dc);
  void setq(double q);

protected:
  Eigen::MatrixXd U_temp_;
  Eigen::MatrixXd X_temp_;
  double cost_;
  double dt_;  // time step found by the solver
  int N_;
  double xf_[3 * INPUT_ORDER];
  double x0_[3 * INPUT_ORDER];
  double v_max_;
  double a_max_;
  double j_max_;
  double DC;
  double q_;  // weight to the 2nd term in the cost function
  double** x_;
  double** u_;
};

// Definitions of the functions of the template of the class Solver. They are in this file because Template definitions
// need to go in headers, they can't go in separately compiled source files (see
// https://www.reddit.com/r/cpp_questions/comments/5yb6it/template_classfunction_and_error_undefined/)

template <int INPUT_ORDER>
Solver<INPUT_ORDER>::Solver()
{
  v_max_ = 20;
  a_max_ = 2;
  j_max_ = 20;
  switch (INPUT_ORDER)
  {
    case VEL:
      vel_initialize_optimizer();
      N_ = 15;
      break;
    case ACCEL:
      accel_initialize_optimizer();
      N_ = 15;
      break;
    case JERK:
      jerk_initialize_optimizer();
      N_ = 10;
      break;
  }
}

template <int INPUT_ORDER>
int Solver<INPUT_ORDER>::getOrder()
{
  return INPUT_ORDER;
}

template <int INPUT_ORDER>
int Solver<INPUT_ORDER>::getN()
{
  return N_;
}

template <int INPUT_ORDER>
void Solver<INPUT_ORDER>::setDC(double dc)
{
  DC = dc;
}

template <int INPUT_ORDER>
void Solver<INPUT_ORDER>::setq(double q)
{
  q_ = q;
}

// it obtains the variable with degree=degree_input+1
template <int INPUT_ORDER>
void Solver<INPUT_ORDER>::obtainByDerivation(double** u, double** x)
{
  int size = (int)(N_)*dt_ / DC;
  size = (size < 2) ? 2 : size;  // force size to be at least 2 (happens when dt_<DC)
  for (int i = 0; i < size - 1; i++)
  {
    U_temp_(i, 3) = (U_temp_(i + 1, 0) - U_temp_(i, 0)) / DC;
    U_temp_(i, 4) = (U_temp_(i + 1, 1) - U_temp_(i, 1)) / DC;
    U_temp_(i, 5) = (U_temp_(i + 1, 2) - U_temp_(i, 2)) / DC;
  }
}
template <int INPUT_ORDER>
Eigen::MatrixXd Solver<INPUT_ORDER>::getX()
{
  return X_temp_;
}

template <int INPUT_ORDER>
Eigen::MatrixXd Solver<INPUT_ORDER>::getU()
{
  return U_temp_;
}

template <int INPUT_ORDER>
void Solver<INPUT_ORDER>::set_x0(double x0[])
{
  for (int i = 0; i < 3 * INPUT_ORDER; i++)
  {
    x0_[i] = x0[i];
  }
}

template <int INPUT_ORDER>
void Solver<INPUT_ORDER>::set_xf(double xf[])
{
  for (int i = 0; i < 3 * INPUT_ORDER; i++)
  {
    xf_[i] = xf[i];
  }
}

/*template <int INPUT_ORDER>
void Solver<INPUT_ORDER>::set_u0(double u0[])
{
  for (int i = 0; i < 3; i++)
  {
    u0_[i] = u0[i];
  }
}*/

template <int INPUT_ORDER>
void Solver<INPUT_ORDER>::resetXandU()
{
  int size = (int)(N_)*dt_ / DC;
  size = (size < 2) ? 2 : size;  // force size to be at least 2
  U_temp_ = Eigen::MatrixXd::Zero(size, 6);
  X_temp_ = Eigen::MatrixXd::Zero(size, 3 * INPUT_ORDER);
}

template <int INPUT_ORDER>
double Solver<INPUT_ORDER>::getCost()
{
  double term_cost = 0;
  switch (INPUT_ORDER)
  {
    case VEL:

      for (int i = 0; i < 3 * INPUT_ORDER; i++)
      {
        term_cost = term_cost + q_ * pow(x_[N_][i] - xf_[i], 2);
      }

      cost_ = (vel_get_cost() - term_cost) * N_ * dt_ / (v_max_ * v_max_);
      break;
    case ACCEL:
      printf("not implemented yet\n");
      // cost_=accel_get_cost();
      break;
    case JERK:
      for (int i = 0; i < 3 * INPUT_ORDER; i++)
      {
        term_cost = term_cost + q_ * pow(x_[N_][i] - xf_[i], 2);
      }
      /*      printf("real total cost=%f\n", jerk_get_cost());
            printf("real jerk cost=%f\n", jerk_get_cost() - term_cost);*/
      cost_ = (jerk_get_cost() - term_cost) * N_ * dt_ / (j_max_ * j_max_);
      break;
  }
  return cost_;
}

template <int INPUT_ORDER>
void Solver<INPUT_ORDER>::set_max(double max_values[INPUT_ORDER])
{
  switch (INPUT_ORDER)
  {
    case VEL:
      v_max_ = max_values[0];
      break;
    case ACCEL:
      v_max_ = max_values[0];
      a_max_ = max_values[1];
      break;
    case JERK:
      v_max_ = max_values[0];
      a_max_ = max_values[1];
      j_max_ = max_values[2];
      break;
  }
}

template <int INPUT_ORDER>
bool Solver<INPUT_ORDER>::checkConvergence(double xf_opt[3 * INPUT_ORDER])
{
  bool converged = false;
  float d2 = 0;   // distance in position squared
  float dv2 = 0;  // distance in velocity squared
  float da2 = 0;  // distance in acceleration squared

  /*  printf("checking convergence\n");
    printf("xf_=\n");
    for (int i = 0; i < 9; i++)
    {
      printf("xf_opt[i]=%0.2f,   xf_[i]=%0.2f\n", xf_opt[i], xf_[i]);
    }*/
  switch (INPUT_ORDER)
  {
    case VEL:
      for (int i = 0; i < 3; i++)
      {
        d2 += pow(xf_[i] - xf_opt[i], 2);
      }
      converged = (sqrt(d2) < 0.2) ? true : false;
      break;
    case ACCEL:
      for (int i = 0; i < 3; i++)
      {
        d2 += pow(xf_[i] - xf_opt[i], 2);
        dv2 += pow(xf_[i + 3] - xf_opt[i + 3], 2);
      }
      converged = (sqrt(d2) < 0.2 && sqrt(dv2) < 0.2) ? true : false;
      break;
    case JERK:
      for (int i = 0; i < 3; i++)
      {
        d2 += pow(xf_[i] - xf_opt[i], 2);
        dv2 += pow(xf_[i + 3] - xf_opt[i + 3], 2);
        da2 += pow(xf_[i + 6] - xf_opt[i + 6], 2);
      }
      // printf("d2=%f, dv2=%f, da2=%f\n", d2, dv2, da2);
      // sleep(2);  // TODO: remove this
      converged = (sqrt(d2) < 0.2 && sqrt(dv2) < 0.2 && 1) ? true : false;
      break;
  }

  return converged;
}

template <int INPUT_ORDER>
// var is the type of variable to interpolate (POS=1, VEL=2, ACCEL=3,...)
void Solver<INPUT_ORDER>::interpolate(int var, double** u, double** x)
{
  int nxd = N_ + 1;
  int nd[] = { nxd };
  // printf("N_=%d\n", N_);
  // printf("dt_=%f\n", dt_);
  // printf("DC=%f\n", DC);
  int ni = (int)(N_)*dt_ / DC;  // total number of points
  // printf("ni=%d", ni);
  double xd[nxd];
  double yd[nxd];
  double xi[ni];
  double yi[ni];
  int type_of_var = (var < INPUT_ORDER) ? STATE : INPUT;
  if (ni <= 2)
  {
    // printf("NOT INTERPOLATING. THIS USUALLY HAPPENS WHEN INPUT=VEL, y VEL_MAX IS HIGH, REDUCE IT!\n");
    for (int axis = 0; axis < 3; axis++)  // var_x,var_y,var_z
    {
      int column_x = axis + 3 * var;
      if (type_of_var == INPUT)
      {
        U_temp_(0, axis) = 0;  // u[0][axis];
        U_temp_(1, axis) = 0;  // Force the last input to be 0
      }
      if (type_of_var == STATE)
      {
        X_temp_(0, column_x) = x0_[column_x];
        X_temp_(1, column_x) = x0_[column_x];
      }
    }
    return;
  }
  // printf("Tipo de variable=%d", type_of_var);
  for (int n = 0; n < ni; ++n)
  {
    xi[n] = n * DC;
  }

  for (int axis = 0; axis < 3; axis++)  // var_x,var_y,var_z
  {
    int column_x = axis + 3 * var;
    for (int i = 1; i < nxd; i++)
    {
      xd[i] = i * dt_;
      yd[i] = (type_of_var == INPUT) ? u[i][axis] : x[i][column_x];
    }

    xd[0] = 0;
    yd[0] = (type_of_var == INPUT) ? u[0][axis] : x0_[column_x];

    /*    printf("ROJOS\n");
        for (int i = 0; i < nxd; i++)
        {
          printf("%f\n", yd[i]);
        }
*/
    /*    printf("CVXGEN\n");
        for (int i = 1; i < N_; i++)
        {
          printf("%f  %f  %f\n", x[i][0], x[i][1], x[i][2]);
        }*/

    mlinterp::interp(nd, ni,  // Number of points
                     yd, yi,  // Output axis (y)
                     xd, xi   // Input axis (x)
    );
    // Force the last input to be 0, and the last state to be the final state:
    yi[ni - 1] = (type_of_var == INPUT) ? 0 : xf_[column_x];

    for (int n = 0; n < ni; ++n)
    {
      // printf("inside the loop\n");
      if (type_of_var == INPUT)
      {
        U_temp_(n, axis) = yi[n];
      }
      if (type_of_var == STATE)
      {
        X_temp_(n, column_x) = yi[n];
      }
      // printf("%f    %f\n", U(n, 0), yi[n]);
    }
    // printf("He asignado X_temp_=\n");
    // std::cout << X_temp_ << std::endl;
  }
}

template <int INPUT_ORDER>
void Solver<INPUT_ORDER>::genNewTraj()
{
  // printf("In genNewTraj\n");
  callOptimizer();
  // printf("In genNewTraj0.5\n");
  resetXandU();
  // printf("In genNewTraj1\n");
  switch (INPUT_ORDER)
  {
    case VEL:
      // printf("To grab states/control\n");
      x_ = vel_get_state();
      u_ = vel_get_control();
      // printf("To interpolate\n");
      interpolate(POS, u_, x_);  // interpolate POS
      interpolate(VEL, u_, x_);  // ...
      // printf("To obtain by derivation\n");
      obtainByDerivation(u_, x_);
      // printf("Obtained by derivation\n");
      break;
    case ACCEL:
      // printf("In genNewTraj2\n");
      x_ = accel_get_state();
      u_ = accel_get_control();
      // printf("In genNewTraj3\n");
      interpolate(POS, u_, x_);    // interpolate POS
      interpolate(VEL, u_, x_);    // ...
      interpolate(ACCEL, u_, x_);  // ...
      obtainByDerivation(u_, x_);
      break;
    case JERK:
      x_ = jerk_get_state();
      u_ = jerk_get_control();

      /*      double my_cost = 0;
            for (int i = 0; i <= N_ - 1; i++)
            {
              my_cost = my_cost + u_[i][0] * u_[i][0] + u_[i][1] * u_[i][1] + u_[i][2] * u_[i][2];
            }
            getCost();
            printf("*****My effort cost: %f\n", my_cost);
            double term_cost = 0;
            for (int i = 0; i <= 8; i++)
            {
              term_cost = term_cost + 100000 * pow(x_[N_][i] - xf_[i], 2);
            }
            printf("*****My terminal cost: %f\n", term_cost);*/
      // my_cost = my_cost + term_cost;

      interpolate(POS, u_, x_);    // interpolate POS
      interpolate(VEL, u_, x_);    // ...
      interpolate(ACCEL, u_, x_);  // ...
      interpolate(JERK, u_, x_);
      break;
  }
}

template <int INPUT_ORDER>
void Solver<INPUT_ORDER>::callOptimizer()
{
  // printf("In callOptimizer\n");
  bool converged = false;

  // ROS_INFO("dt I found= %0.2f", dt_found);
  double dt = getDTInitial();  // 0.025
                               // ROS_INFO("empezando con, dt = %0.2f", dt);

  double** x;
  int i = 0;
  int r = 0;
  while (1)
  {
    dt = dt + 4 * 0.025;  // To make sure that it will converge in very few iterations (hopefully only 1)
    i = i + 1;
    // printf("Loading default data!\n");
    switch (INPUT_ORDER)
    {
      case VEL:
      {
        vel_load_default_data(dt, v_max_, x0_, xf_, q_);
        r = vel_optimize();
        if (r == 1)
        {
          x = vel_get_state();
          converged = checkConvergence(x[N_]);
        }
        break;
      }
      case ACCEL:
      {
        accel_load_default_data(dt, v_max_, a_max_, x0_, xf_, q_);
        r = accel_optimize();
        if (r == 1)
        {
          x = accel_get_state();
          converged = checkConvergence(x[N_]);
        }
        break;
      }
      case JERK:
      {
        jerk_load_default_data(dt, v_max_, a_max_, j_max_, x0_, xf_, q_);
        r = jerk_optimize();
        if (r == 1)
        {
          x = jerk_get_state();
          converged = checkConvergence(x[N_]);
        }
        break;
      }
    }
    if (converged == 1)
    {
      break;
    }
    // dt += 0.025;
  }

  if (i > 1)
  {
    printf("Iterations = %d\n", i);
    printf("Iterations>1, if you increase dt at the beginning, it would be faster\n");
  }
  // ROS_INFO("Iterations = %d\n", i);
  // ROS_INFO("converged, dt = %f", dt);
  dt_ = dt;
}

template <int INPUT_ORDER>
double Solver<INPUT_ORDER>::getDTInitial()
{  // TODO: Not sure if this is right or not. Implement also for Jerk? See page 4, (up-right part) of Search-based
   // Motion Planning for Quadrotors using Linear Quadratic Minimum Time Control
  double dt_initial = 0;
  float t_vx = 0;
  float t_vy = 0;
  float t_vz = 0;
  float t_ax = 0;
  float t_ay = 0;
  float t_az = 0;
  float t_jx = 0;
  float t_jy = 0;
  float t_jz = 0;

  t_vx = (xf_[0] - x0_[0]) / v_max_;
  t_vy = (xf_[1] - x0_[1]) / v_max_;
  t_vz = (xf_[2] - x0_[2]) / v_max_;
  // printf("%f\n", t_vx);
  // printf("%f\n", t_vy);
  // printf("%f\n", t_vz);
  switch (INPUT_ORDER)
  {
    case JERK:
    {
      float jerkx = copysign(1, xf_[0] - x0_[0]) * j_max_;
      float jerky = copysign(1, xf_[1] - x0_[1]) * j_max_;
      float jerkz = copysign(1, xf_[2] - x0_[2]) * j_max_;
      float a0x = x0_[6];
      float a0y = x0_[7];
      float a0z = x0_[8];
      float v0x = x0_[3];
      float v0y = x0_[4];
      float v0z = x0_[5];

      // polynomial ax3+bx2+cx+d=0 --> coeff=[d c b a]
      Eigen::Vector4d coeffx(x0_[0] - xf_[0], v0x, a0x / 2, jerkx / 6);
      Eigen::Vector4d coeffy(x0_[1] - xf_[1], v0y, a0y / 2, jerky / 6);
      Eigen::Vector4d coeffz(x0_[2] - xf_[2], v0z, a0z / 2, jerkz / 6);

      Eigen::PolynomialSolver<double, Eigen::Dynamic> psolvex(coeffx);
      Eigen::PolynomialSolver<double, Eigen::Dynamic> psolvey(coeffy);
      Eigen::PolynomialSolver<double, Eigen::Dynamic> psolvez(coeffz);

      std::vector<double> realRootsx;
      std::vector<double> realRootsy;
      std::vector<double> realRootsz;
      psolvex.realRoots(realRootsx);
      psolvey.realRoots(realRootsy);
      psolvez.realRoots(realRootsz);

      t_jx = *std::min_element(realRootsx.begin(), realRootsx.end());
      t_jy = *std::min_element(realRootsy.begin(), realRootsy.end());
      t_jz = *std::min_element(realRootsz.begin(), realRootsz.end());

      // printf("t_jx, t_jy, t_jz:\n");
      // std::cout << t_jx << "  " << t_jy << "  " << t_jz << std::endl;

      // DON'T PUT HERE A BREAK!!
    }
    case ACCEL:  // case Accel
    {
      float accelx = copysign(1, xf_[0] - x0_[0]) * a_max_;
      float accely = copysign(1, xf_[1] - x0_[1]) * a_max_;
      float accelz = copysign(1, xf_[2] - x0_[2]) * a_max_;
      float v0x = x0_[3];
      float v0y = x0_[4];
      float v0z = x0_[5];
      // Solve equation xf=x0+v0t+0.5*a*t^2
      t_ax = solvePolyOrder2(Eigen::Vector3f(0.5 * accelx, v0x, x0_[0] - xf_[0]));
      t_ay = solvePolyOrder2(Eigen::Vector3f(0.5 * accely, v0y, x0_[1] - xf_[1]));
      t_az = solvePolyOrder2(Eigen::Vector3f(0.5 * accelz, v0z, x0_[2] - xf_[2]));
    }
    case VEL:
    {
      // I'm done
      break;
    }
  }
  dt_initial = std::max({ t_vx, t_vy, t_vz, t_ax, t_ay, t_az, t_jx, t_jy, t_jz }) / N_;
  if (dt_initial > 10000)  // happens when there is no solution to the previous eq.
  {
    printf("there is not a solution to the previous equations");
    dt_initial = 0;
  }
  // printf("dt_initial=%f", dt_initial);
  return dt_initial;
}

#endif