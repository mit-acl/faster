#ifndef SOLVERS_HPP
#define SOLVERS_HPP
#include <Eigen/Dense>
#include "../utils.hpp"
#include "mlinterp.hpp"

#include "cvxgen/interface_jerk.h"

#include "cvxgen/interface_accel.h"

template <int INPUT_ORDER>
class Solver
{
public:
  Solver();
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
  int getOrder();
  int getN();

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
      printf("not implemented yet\n");
      // vel_initialize_optimizer();
      // N_=;
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
  // printf("In genNewTraj\n");
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
      // interpolate(POS, u, x);  // interpolate POS
      // interpolate(VEL, u, x);  // ...
      // obtainByDerivation(u, x);
      break;
    case ACCEL:
      x = accel_get_state();
      u = accel_get_control();
      interpolate(POS, u, x);    // interpolate POS
      interpolate(VEL, u, x);    // ...
      interpolate(ACCEL, u, x);  // ...
      obtainByDerivation(u, x);
      break;
    case JERK:
      x = jerk_get_state();
      u = jerk_get_control();
      interpolate(POS, u, x);    // interpolate POS
      interpolate(VEL, u, x);    // ...
      interpolate(ACCEL, u, x);  // ...
      interpolate(JERK, u, x);
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
    i = i + 1;
    // printf("Loading default data!\n");
    switch (INPUT_ORDER)
    {
      case VEL:
      {
        printf("Not implemented\n");
        break;
      }
      case ACCEL:
      {
        accel_load_default_data(dt, v_max_, a_max_, x0_, xf_);
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
        jerk_load_default_data(dt, v_max_, a_max_, j_max_, x0_, xf_);
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
    dt += 0.025;
  }
  ROS_INFO("Iterations = %d\n", i);
  // ROS_INFO("converged, dt = %0.2f", dt);
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

  t_vx = (xf_[0] - x0_[0]) / v_max_;
  t_vy = (xf_[1] - x0_[1]) / v_max_;
  t_vz = (xf_[2] - x0_[2]) / v_max_;
  switch (INPUT_ORDER)
  {
    case VEL:
    {
      // I'm done
      break;
    }
    case ACCEL:
    case JERK:  // case Accel or case Jerk
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
      break;
    }
  }
  dt_initial = std::max({ t_vx, t_vy, t_vz, t_ax, t_ay, t_az }) / N_;
  if (dt_initial > 10000)  // happens when there is no solution to the previous eq.
  {
    dt_initial = 0;
  }
  printf("dt_initial=%f", dt_initial);
  return dt_initial;
}

#endif