#ifndef SOLVERS_HPP
#define SOLVERS_HPP
#include <Eigen/Dense>
#include "../utils.hpp"
#include "mlinterp.hpp"
namespace Accel  // When the input is acceleration
{
#include "cvxgen/solver_accel.h"
}
//####Class Solver
template <int INPUT_ORDER>
class Solver
{
public:
  void interpolate(int var, double** u, double** x);
  void obtainByDerivation(double** u, double** x);
  Eigen::MatrixXd getX();
  Eigen::MatrixXd getU();
  void set_x0(double x0[]);
  void set_u0(double u0[]);
  void set_xf(double xf[]);
  void resetXandU();
  void set_max(double max_values[INPUT_ORDER]);
  bool checkConvergence(double xf_opt[3 * INPUT_ORDER]);
  void genNewTraj();
  void callOptimizer();
  double getDTInitial();

protected:
  Eigen::MatrixXd U_temp_;
  Eigen::MatrixXd X_temp_;
  double dt_;  // time step found by the solver
  int N_;
  double xf_[3 * INPUT_ORDER];
  double x0_[3 * INPUT_ORDER];
  double u0_[3];
  double v_max_;
  double a_max_;
  double j_max_;
};

//####Class SolverAccel
class SolverAccel : public Solver<ACCEL>
{
public:
  SolverAccel();

  void interpBRETT(double dt, double xf[], double u0[], double x0[], double** u, double** x, Eigen::MatrixXd& U,
                   Eigen::MatrixXd& X);

private:
};

// Definitions of the functions of the template of the class Solver. They are in this file because Template definitions
// need to go in headers, they can't go in separately compiled source files (see
// https://www.reddit.com/r/cpp_questions/comments/5yb6it/template_classfunction_and_error_undefined/)

// it obtains the variable with degree=degree_input+1
template <int INPUT_ORDER>
void Solver<INPUT_ORDER>::obtainByDerivation(double** u, double** x)
{
  int size = (int)(N_)*dt_ / DC;
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

template <int INPUT_ORDER>
void Solver<INPUT_ORDER>::set_u0(double u0[])
{
  for (int i = 0; i < 3; i++)
  {
    u0_[i] = u0[i];
  }
}

template <int INPUT_ORDER>
void Solver<INPUT_ORDER>::resetXandU()
{
  int size = (int)(N_)*dt_ / DC;
  U_temp_ = Eigen::MatrixXd::Zero(size, 6);
  X_temp_ = Eigen::MatrixXd::Zero(size, 3 * INPUT_ORDER);
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
      converged = (sqrt(d2) < 0.2 && sqrt(dv2) < 0.2 && sqrt(da2) < 0.2) ? true : false;
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
  int ni = (int)(N_)*dt_ / DC;  // total number of points
  double xd[nxd];
  double yd[nxd];
  double xi[ni];
  double yi[ni];
  int type_of_var = (var < INPUT_ORDER) ? STATE : INPUT;

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
    yd[0] = (type_of_var == INPUT) ? u0_[axis] : x0_[column_x];

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

    /*    printf("type_of_var=%d\n", type_of_var);
        printf("type_of_var=%d\n", type_of_var);*/
    for (int n = 0; n < ni; ++n)
    {
      if (type_of_var == INPUT)
      {
        // printf("assigning\n");
        U_temp_(n, axis) = yi[n];
      }
      if (type_of_var == STATE)
      {
        // printf("assigning\n");
        X_temp_(n, column_x) = yi[n];
      }
      // printf("%f    %f\n", U(n, 0), yi[n]);
    }
  }
}

template <int INPUT_ORDER>
void Solver<INPUT_ORDER>::genNewTraj()
{
  callOptimizer();
  resetXandU();
  double** x;
  double** u;
  switch (INPUT_ORDER)
  {
    case VEL:
      printf("Not implemented yet!");
      // double** x = Vel::get_state();
      // double** u = Vel::get_control();
      // interpolate(POS, u, x);  // interpolate POS when the input is acceleration*/
      // interpolate(VEL, u, x);  // ...
      // obtainByDerivation(u, x);
      break;
    case ACCEL:
      x = Accel::get_state();
      u = Accel::get_control();
      interpolate(POS, u, x);    // interpolate POS when the input is acceleration*/
      interpolate(VEL, u, x);    // ...
      interpolate(ACCEL, u, x);  // ...
      obtainByDerivation(u, x);
      break;
    case JERK:
      printf("Not implemented yet!");
      // double** x = Jerk::get_state();
      // double** u = Jerk::get_control();
      // interpolate(POS, u, x);    // interpolate POS when the input is acceleration*/
      // interpolate(VEL, u, x);    // ...
      // interpolate(ACCEL, u, x);  // ...
      // interpolate(Jerk, u, x);
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

  while (!converged)
  {
    Accel::load_default_data(dt, v_max_, a_max_, x0_, xf_);
    int r = Accel::optimize();
    i = i + 1;
    if (r == 1)
    {
      x = Accel::get_state();
      bool s = checkConvergence(x[N_]);
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
  dt_ = dt;
}

template <int INPUT_ORDER>
double Solver<INPUT_ORDER>::getDTInitial()
{
  double dt_initial;
  switch (INPUT_ORDER)
  {
    case VEL:
    {
      printf("Not implemented yet!");
      break;
    }
    case ACCEL:
    {
      // TODO: Be careful with constraints also in velocity?
      float accelx = copysign(1, xf_[0] - x0_[0]) * a_max_;
      float accely = copysign(1, xf_[1] - x0_[1]) * a_max_;
      float accelz = copysign(1, xf_[2] - x0_[2]) * a_max_;
      float v0x = x0_[3];
      float v0y = x0_[4];
      float v0z = x0_[5];
      float tx =
          solvePolyOrder2(Eigen::Vector3f(0.5 * accelx, v0x, x0_[0] - xf_[0]));  // Solve equation xf=x0+v0t+0.5*a*t^2
      float ty = solvePolyOrder2(Eigen::Vector3f(0.5 * accely, v0y, x0_[1] - xf_[1]));
      float tz = solvePolyOrder2(Eigen::Vector3f(0.5 * accelz, v0z, x0_[2] - xf_[2]));
      dt_initial = std::max({ tx, ty, tz }) / N_;
      if (dt_initial > 100)  // happens when there is no solution to the previous eq.
      {
        dt_initial = 0;
      }
      break;
    }
    case JERK:
    {
      printf("Not implemented yet!");
      break;
    }
  }
  return dt_initial;
}

#endif