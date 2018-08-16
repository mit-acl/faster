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
  void interpolate(int var, int input, double** u, double** x);
  void obtainByDerivation(double** u, double** x);
  Eigen::MatrixXd getX();
  Eigen::MatrixXd getU();
  void set_x0(double x0[]);
  void set_u0(double u0[]);
  void set_xf(double xf[]);
  void resetXandU();
  void set_max(double max_values[INPUT_ORDER]);

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
  void genNewTraj();
  void callOptimizer();
  int checkConvergence(double xf_opt[]);
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
// var is the type of variable to interpolate (POS=1, VEL=2, ACCEL=3,...)
void Solver<INPUT_ORDER>::interpolate(int var, int input, double** u, double** x)
{
  int nxd = N_ + 1;
  int nd[] = { nxd };
  int ni = (int)(N_)*dt_ / DC;  // total number of points
  double xd[nxd];
  double yd[nxd];
  double xi[ni];
  double yi[ni];
  int type_of_var = (var < input) ? STATE : INPUT;

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

#endif