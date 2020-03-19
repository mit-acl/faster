/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

// Continuous version, and with polytope constraints.

#include "gurobi_c++.h"
#include <sstream>
#include <Eigen/Dense>
#include <type_traits>
using namespace std;

template <typename T>
GRBQuadExpr GetNorm2(const std::vector<T>& x)  // Return the squared norm of a vector
{
  GRBQuadExpr result = 0;
  for (int i = 0; i < x.size(); i++)
  {
    result = result + x[i] * x[i];
  }
  return result;
}

std::vector<GRBLinExpr> MatrixMultiply(const std::vector<std::vector<double>>& A, const std::vector<GRBVar>& x)
{
  std::vector<GRBLinExpr> result;

  for (int i = 0; i < A.size(); i++)
  {
    GRBLinExpr lin_exp = 0;
    for (int m = 0; m < x.size(); m++)
    {
      lin_exp = lin_exp + A[i][m] * x[m];
    }
    result.push_back(lin_exp);
  }
  return result;
}

std::vector<GRBLinExpr> MatrixMultiply(const std::vector<std::vector<double>>& A, const std::vector<GRBLinExpr>& x)
{
  std::vector<GRBLinExpr> result;

  for (int i = 0; i < A.size(); i++)
  {
    GRBLinExpr lin_exp = 0;
    for (int m = 0; m < x.size(); m++)
    {
      lin_exp = lin_exp + A[i][m] * x[m];
    }
    result.push_back(lin_exp);
  }
  return result;
}

template <typename T>  // Overload + to sum Elementwise std::vectors
std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b)
{
  assert(a.size() == b.size());

  std::vector<T> result;
  result.reserve(a.size());

  std::transform(a.begin(), a.end(), b.begin(), std::back_inserter(result), std::plus<T>());
  return result;
}

template <typename T>  // Overload - to substract Elementwise std::vectors
std::vector<T> operator-(const std::vector<T>& a, const std::vector<T>& b)
{
  assert(a.size() == b.size());

  std::vector<T> result;
  result.reserve(a.size());

  std::transform(a.begin(), a.end(), b.begin(), std::back_inserter(result), std::minus<T>());
  return result;
}

std::vector<GRBLinExpr> operator-(const std::vector<GRBVar>& x, const std::vector<double>& b)
{
  std::vector<GRBLinExpr> result;
  for (int i = 0; i < x.size(); i++)
  {
    GRBLinExpr tmp = x[i] - b[i];
    result.push_back(tmp);
  }
  return result;
}

template <typename T>
std::vector<T> eigenVector2std(const Eigen::Matrix<T, -1, 1>& x)
{
  std::vector<T> result = 0;
  for (int i = 0; i < x.rows(); i++)
  {
    result.push_back(x(i, 1));
  }
  return result;
}

template <typename T>
std::vector<std::vector<T>> eigenMatrix2std(const Eigen::Matrix<T, -1, -1>& x)
{
  std::vector<std::vector<T>> result;

  for (int i = 0; i < x.rows(); i++)
  {
    std::vector<T> row;
    for (int j = 0; j < x.cols(); j++)
    {
      row.push_back(x(i, j));
    }
    result.push_back(row);
  }
  return result;
}

template <typename T>
std::vector<T> GetColumn(std::vector<std::vector<T>> x, int column)
{
  std::vector<T> result;

  for (int i = 0; i < x.size(); i++)
  {
    result.push_back(x[i][column]);
  }
  return result;
}

GRBLinExpr getPos(int t, double tau, int ii, bool solved, std::vector<std::vector<GRBVar>> x)
{
  if (solved == true)
  {
    GRBLinExpr pos = (x[t][0 + ii].get(GRB_DoubleAttr_X)) * tau * tau * tau +
                     (x[t][3 + ii].get(GRB_DoubleAttr_X)) * tau * tau + (x[t][6 + ii].get(GRB_DoubleAttr_X)) * tau +
                     (x[t][9 + ii].get(GRB_DoubleAttr_X));
    return pos;
  }
  else
  {
    GRBLinExpr pos = x[t][0 + ii] * tau * tau * tau + x[t][3 + ii] * tau * tau + x[t][6 + ii] * tau + x[t][9 + ii];
    return pos;
  }
}

GRBLinExpr getVel(int t, double tau, int ii, bool solved, std::vector<std::vector<GRBVar>> x)
{  // t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)
  if (solved == true)
  {
    GRBLinExpr vel = (3 * x[t][0 + ii].get(GRB_DoubleAttr_X)) * tau * tau +
                     (2 * x[t][3 + ii].get(GRB_DoubleAttr_X)) * tau + (x[t][6 + ii].get(GRB_DoubleAttr_X));
    return vel;
  }
  else
  {
    GRBLinExpr vel = 3 * x[t][0 + ii] * tau * tau + 2 * x[t][3 + ii] * tau + x[t][6 + ii];
    return vel;
  }
}

GRBLinExpr getAccel(int t, double tau, int ii, bool solved, std::vector<std::vector<GRBVar>> x)
{  // t is the segment, tau is the time inside a specific segment(\in[0, dt], i is the axis)
  if (solved == true)
  {
    GRBLinExpr accel = (6 * x[t][0 + ii].get(GRB_DoubleAttr_X)) * tau + 2 * x[t][3 + ii].get(GRB_DoubleAttr_X);
    return accel;
  }
  else
  {
    GRBLinExpr accel = 6 * x[t][0 + ii] * tau + 2 * x[t][3 + ii];
    return accel;
  }
}

GRBLinExpr getJerk(int t, double tau, int ii, bool solved, std::vector<std::vector<GRBVar>> x)
{  // t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)
  if (solved == true)
  {
    GRBLinExpr jerk = 6 * x[t][0 + ii].get(GRB_DoubleAttr_X);  // Note that here tau doesn't appear (makes sense)
    return jerk;
  }
  else
  {
    GRBLinExpr jerk = 6 * x[t][0 + ii];  // Note that here tau doesn't appear (makes sense)
    return jerk;
  }
}

int main(int argc, char* argv[])
{
  GRBEnv* env = 0;
  GRBVar* open = 0;
  GRBVar** transport = 0;
  int transportCt = 0;
  try
  {
    // Model
    env = new GRBEnv();
    GRBModel m = GRBModel(*env);
    m.set(GRB_StringAttr_ModelName, "planning");

    int N = 10;
    double umax = 5;
    double amax = 3;
    double vmax = 5;
    /*    double q = 20000000000;*/
    double dt = 5.0 / N;
    /*    double dt2 = dt * dt / 2.0;
        double dt3 = dt * dt * dt / 6.0;
        std::vector<std::string> states = { "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az" };
        std::vector<std::string> inputs = { "jx", "jy", "jz" };*/

    std::vector<std::string> coeff = { "ax", "ay", "az", "bx", "by", "bz", "cx", "cy", "cz", "dx", "dy", "dz" };

    std::vector<double> x0 = { 5, 11.5, 0.5, 0, 0, 0, 0, 0, 0 };

    std::vector<double> xf = { 14, 5, 2.5, 0, 0, 0, 0, 0, 0 };

    // std::cout << "here1" << std::endl;

    std::vector<std::vector<GRBVar>> x;
    std::vector<std::vector<GRBVar>> u;

    for (int t = 0; t < N + 1; t++)
    {
      std::vector<GRBVar> row_t;
      for (int i = 0; i < 12; i++)
      {
        row_t.push_back(m.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, coeff[i] + std::to_string(t)));
      }
      x.push_back(row_t);
    }

    // std::cout << "here2" << std::endl;

    /*    for (int i = 0; i < 3; i++)
        {
          std::vector<GRBVar> row_i;
          for (int t = 0; t < N; t++)
          {
            row_i.push_back(m.addVar(-umax, umax, 0, GRB_CONTINUOUS, inputs[i] + std::to_string(t)));
          }
          u.push_back(row_i);
        }*/

    // Constraints x_t+1=Ax_t+Bu_t
    for (int t = 0; t < N - 1; t++)
    {
      for (int i = 0; i < 3; i++)
      {
        m.addConstr(getPos(t, dt, i, false, x) == getPos(t + 1, 0, i, false, x));      // Continuity in position
        m.addConstr(getVel(t, dt, i, false, x) == getVel(t + 1, 0, i, false, x));      // Continuity in velocity
        m.addConstr(getAccel(t, dt, i, false, x) == getAccel(t + 1, 0, i, false, x));  // Continuity in acceleration
      }
    }

    // std::cout << "here3" << std::endl;
    // Constraint x0==x_initial

    for (int i = 0; i < 3; i++)
    {
      m.addConstr(getPos(0, 0, i, false, x) == x0[i]);        // Initial position
      m.addConstr(getVel(0, 0, i, false, x) == x0[i + 3]);    // Initial velocity
      m.addConstr(getAccel(0, 0, i, false, x) == x0[i + 6]);  // Initial acceleration}
    }

    //  std::cout << "here4" << std::endl;

    // Constraint xT==x_final
    for (int i = 0; i < 3; i++)
    {
      m.addConstr(getPos(N - 1, dt, i, false, x) - xf[i] <= 0.2);   // Final position
      m.addConstr(getPos(N - 1, dt, i, false, x) - xf[i] >= -0.2);  // Final position

      m.addConstr(getVel(N - 1, dt, i, false, x) - xf[i + 3] <= 0.2);   // Final velocity
      m.addConstr(getVel(N - 1, dt, i, false, x) - xf[i + 3] >= -0.2);  // Final velocity

      m.addConstr(getAccel(N - 1, dt, i, false, x) - xf[i + 6] <= 0.2);   // Final acceleration
      m.addConstr(getAccel(N - 1, dt, i, false, x) - xf[i + 6] >= -0.2);  // Final acceleration
    }

    // std::cout << "here5" << std::endl;
    // Constraint v<=vmax, a<=amax, u<=umax
    for (int t = 0; t < N - 1; t++)
    {
      for (int i = 0; i < 3; i++)
      {
        m.addConstr(getVel(t, dt, i, false, x) <= vmax);
        m.addConstr(getVel(t, dt, i, false, x) >= -vmax);

        m.addConstr(getAccel(t, dt, i, false, x) <= amax);
        m.addConstr(getAccel(t, dt, i, false, x) >= -amax);

        m.addConstr(getJerk(t, dt, i, false, x) <= umax);
        m.addConstr(getJerk(t, dt, i, false, x) >= -umax);
      }
    }

    //  std::cout << "here6" << std::endl;

    /*    Eigen::Matrix<double, -1, 3> A1;
        Eigen::Matrix<double, -1, 3> A2;
        Eigen::Matrix<double, -1, 3> A3;*/

    Eigen::MatrixXd A1(12, 3);
    Eigen::MatrixXd A2(10, 3);
    Eigen::MatrixXd A3(10, 3);

    Eigen::VectorXd b1(12);
    Eigen::VectorXd b2(10);
    Eigen::VectorXd b3(10);

    A1 << -0.0990887, 0.994031, -0.0456529,  ////////////////////////////////////
        -0.11874, 0.992494, 0.0292636,       ////////////////////////////////////
        0.315838, 0.947835, -0.0430724,      ////////////////////////////////////
        0.279625, 0.956348, 0.0849006,       ////////////////////////////////////
        0.0379941, -0.999235, -0.00925698,   ////////////////////////////////////
        0.031154, -0.999406, 0.0147274,      ////////////////////////////////////
        0, -1, 0,                            ////////////////////////////////////
        -0, 1, -0,                           ////////////////////////////////////
        0.95448, 0, 0.298275,                ////////////////////////////////////
        -0.95448, -0, -0.298275,             ////////////////////////////////////
        0.298275, 0, -0.95448,               ////////////////////////////////////
        -0.298275, -0, 0.95448;              ////////////////////////////////////

    std::cout << "here6.5" << std::endl;

    b1 << 11.2113,
        11.1351,   ////////////////////////////////////
        15.1733,   ////////////////////////////////////
        15.1708,   ////////////////////////////////////
        -9.26336,  ////////////////////////////////////
        -9.27607,  ////////////////////////////////////
        -9.5,      ////////////////////////////////////
        13.5,      ////////////////////////////////////
        14.3031,   ////////////////////////////////////
        -3.92154,  ////////////////////////////////////
        2.01413,   ////////////////////////////////////
        -0.014135;

    A1 << -0.0990887, 0.994031, -0.0456529,  ////////////////////////////////////
        -0.11874, 0.992494, 0.0292636,       ////////////////////////////////////
        0.315838, 0.947835, -0.0430724,      ////////////////////////////////////
        0.279625, 0.956348, 0.0849006,       ////////////////////////////////////
        0.0379941, -0.999235, -0.00925698,   ////////////////////////////////////
        0.031154, -0.999406, 0.0147274,      ////////////////////////////////////
        0, -1, 0,                            ////////////////////////////////////
        -0, 1, -0,                           ////////////////////////////////////
        0.95448, 0, 0.298275,                ////////////////////////////////////
        -0.95448, -0, -0.298275,             ////////////////////////////////////
        0.298275, 0, -0.95448,               ////////////////////////////////////
        -0.298275, -0, 0.95448;              ////////////////////////////////////

    A2 << -0.199658, 0.976518, 0.0809297,  ////////////////////////////////////
        -0.166117, 0.983608, -0.0701482,   ////////////////////////////////////
        -0.358298, -0.933434, 0.0179951,   ////////////////////////////////////
        -0.365568, -0.928824, -0.0603848,  ////////////////////////////////////
        -0.707107, -0.707107, 0,           ////////////////////////////////////
        0.707107, 0.707107, -0,            ////////////////////////////////////
        0.485071, -0.485071, -0.727607,    ////////////////////////////////////
        -0.485071, 0.485071, 0.727607,     ////////////////////////////////////
        -0.514496, 0.514496, -0.685994,    ////////////////////////////////////
        0.514496, -0.514496, 0.685994;     ////////////////////////////////////

    b2 << 9.06362,  ////////////////////////////////////
        9.21024,    ////////////////////////////////////
        -13.5612,   ////////////////////////////////////
        -13.7295,   ////////////////////////////////////
        -15.3241,   ////////////////////////////////////
        19.3241,    ////////////////////////////////////
        1.60634,    ////////////////////////////////////
        2.45521,    ////////////////////////////////////
        -1.82973,   ////////////////////////////////////
        3.82973;    ////////////////////////////////////

    A3 << -0.999958, 0.00342454, 0.00852636,  ////////////////////////////////////
        -0.999832, 0.00363672, -0.0179664,    ////////////////////////////////////
        -0.999778, -0.0204566, 0.00504416,    ////////////////////////////////////
        -0.999564, -0.0227306, -0.0188383,    ////////////////////////////////////
        -1, -0, 0,                            ////////////////////////////////////
        1, 0, -0,                             ////////////////////////////////////
        0, -0.98387, 0.178885,                ////////////////////////////////////
        -0, 0.98387, -0.178885,               ////////////////////////////////////
        0, -0.178885, -0.98387,               ////////////////////////////////////
        -0, 0.178885, 0.98387;

    b3 << -12.7365,  ////////////////////////////////////
        -12.7834,    ////////////////////////////////////
        -12.9236,    ////////////////////////////////////
        -12.9824,    ////////////////////////////////////
        -12,         ////////////////////////////////////
        16,          ////////////////////////////////////
        -3.47214,    ////////////////////////////////////
        11.0623,     ////////////////////////////////////
        -2.3541,     ////////////////////////////////////
        4.3541;
    std::cout << "here6.7" << std::endl;

    std::vector<std::vector<double>> A1std = eigenMatrix2std(A1);
    std::vector<std::vector<double>> A2std = eigenMatrix2std(A2);
    std::vector<std::vector<double>> A3std = eigenMatrix2std(A3);

    /*    std::vector<std::vector<double>> b1std = eigenMatrix2std(b1);
        std::vector<std::vector<double>> b2std = eigenMatrix2std(b2);
        std::vector<std::vector<double>> b3std = eigenMatrix2std(b3);*/

    // m.update();

    std::cout << "here7" << std::endl;
    std::vector<std::vector<GRBVar>> b;
    for (int t = 0; t < N + 1; t++)
    {
      std::vector<GRBVar> row;
      for (int i = 0; i < 3; i++)  // For the three polytopes
      {
        GRBVar variable =
            m.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_BINARY, "s" + std::to_string(i) + "_" + std::to_string(t));
        row.push_back(variable);
      }
      b.push_back(row);
    }

    std::cout << "here8" << std::endl;

    // If is 1 --> in that polytope

    for (int t = 0; t < N; t++)
    {
      GRBLinExpr sum = 0;
      for (int col = 0; col < b[0].size(); col++)
      {
        sum = sum + b[t][col];
      }
      m.addConstr(sum == 1);

      std::vector<GRBLinExpr> pos = { getPos(t, 0, 0, false, x), getPos(t, 0, 1, false, x), getPos(t, 0, 2, false, x) };

      for (int i = 0; i < b1.rows(); i++)
      {
        m.addGenConstrIndicator(b[t][0], 1, MatrixMultiply(A1std, pos)[i], '<',
                                b1[i]);  // If b[t,0]==1, then...
      }
      for (int i = 0; i < b2.rows(); i++)
      {
        m.addGenConstrIndicator(b[t][1], 1, MatrixMultiply(A2std, pos)[i], '<',
                                b2[i]);  // If b[t,1]==1, then...
      }
      for (int i = 0; i < b3.rows(); i++)
      {
        m.addGenConstrIndicator(b[t][2], 1, MatrixMultiply(A3std, pos)[i], '<',
                                b3[i]);  // If b[t,2]==1, then...
      }
    }
    std::cout << "here9" << std::endl;

    GRBQuadExpr control_cost = 0;
    for (int t = 0; t < N; t++)
    {
      std::vector<GRBLinExpr> ut = { getJerk(t, 0, 0, false, x), getJerk(t, 0, 1, false, x),
                                     getJerk(t, 0, 2, false, x) };
      control_cost = control_cost + GetNorm2(ut);
    }

    std::cout << "here10" << std::endl;

    /*    GRBQuadExpr final_state_cost = 0;
        std::vector<GRBVar> xFinal = GetColumn(x, N);
        // std::vector<GRBLinExpr> prueba = xFinal - xf;
        final_state_cost = GetNorm2(xFinal - xf);
        final_state_cost = q * final_state_cost;*/

    m.setObjective(control_cost, GRB_MINIMIZE);

    // Solve*/
    m.update();
    std::cout << "here11" << std::endl;
    m.write("debug.lp");
    m.optimize();
    std::cout << "here12" << std::endl;

    std::cout << "\nOBJECTIVE: " << m.get(GRB_DoubleAttr_ObjVal) << std::endl;
    std::cout << "Positions X:" << std::endl;
    for (int t = 0; t < N + 1; t++)
    {
      std::cout << getPos(t, 0, 0, true, x) << std::endl;
    }
  }

  catch (GRBException e)
  {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  }
  /*catch (...)
  {
    cout << "Exception during optimization" << endl;
  }*/

  delete env;
  return 0;
}
