/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

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
std::vector<T> eigenVector2std(const Eigen::Matrix<T, -1, 1>& x)  // Return the squared norm of a vector
{
  std::vector<T> result = 0;
  for (int i = 0; i < x.rows(); i++)
  {
    result.push_back(x(i, 1));
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
    GRBModel model = GRBModel(*env);
    model.set(GRB_StringAttr_ModelName, "planning");

    int N = 20;
    double umax = 5;
    double q = 20000000000;
    double dt = 0.4;
    double dt2 = dt * dt / 2.0;
    double dt3 = dt * dt * dt / 6.0;
    std::vector<std::string> states = { "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az" };
    std::vector<std::string> inputs = { "jx", "jy", "jz" };

    std::vector<double> x0 = { 5, 11.5, 0.5, 0, 0, 0, 0, 0, 0 };

    std::vector<double> xf = { 14, 5, 2.5, 0, 0, 0, 0, 0, 0 };

    std::vector<std::vector<double>> As = { { 1, 0, 0, dt, 0, 0, dt2, 0, 0 },  /////////////////////////////////
                                            { 0, 1, 0, 0, dt, 0, 0, dt2, 0 },  /////////////////////////////////
                                            { 0, 0, 1, 0, 0, dt, 0, 0, dt2 },  /////////////////////////////////
                                            { 0, 0, 0, 1, 0, 0, dt, 0, 0 },    /////////////////////////////////
                                            { 0, 0, 0, 0, 1, 0, 0, dt, 0 },    /////////////////////////////////
                                            { 0, 0, 0, 0, 0, 1, 0, 0, dt },    /////////////////////////////////
                                            { 0, 0, 0, 0, 0, 0, 1, 0, 0 },     /////////////////////////////////
                                            { 0, 0, 0, 0, 0, 0, 0, 1, 0 },     /////////////////////////////////
                                            { 0, 0, 0, 0, 0, 0, 0, 0, 1 } };

    std::vector<std::vector<double>> Bs = { { dt3, 0, 0 },   /////////////////////////////////
                                            { 0, dt3, 0 },   /////////////////////////////////
                                            { 0, 0, dt3 },   /////////////////////////////////
                                            { dt2, 0, 0 },   /////////////////////////////////
                                            { 0, dt2, 0 },   /////////////////////////////////
                                            { 0, 0, dt2 },   /////////////////////////////////
                                            { dt, 0, 0 },    /////////////////////////////////
                                            { 0, dt, 0 },    /////////////////////////////////
                                            { 0, 0, dt } };  /////////////////////////////////

    std::cout << "here1" << std::endl;

    std::vector<std::vector<GRBVar>> x;
    std::vector<std::vector<GRBVar>> u;
    for (int i = 0; i < 9; i++)
    {
      std::vector<GRBVar> row_i;
      for (int t = 0; t < N + 1; t++)
      {
        row_i.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, states[i] + std::to_string(t)));
      }
      x.push_back(row_i);
    }

    for (int i = 0; i < 3; i++)
    {
      std::vector<GRBVar> row_i;
      for (int t = 0; t < N; t++)
      {
        row_i.push_back(model.addVar(-umax, umax, 0, GRB_CONTINUOUS, inputs[i] + std::to_string(t)));
      }
      u.push_back(row_i);
    }

    // Constraints x_t+1=Ax_t+Bu_t
    for (int t = 0; t < N; t++)
    {
      std::vector<GRBVar> xt = GetColumn(x, t);
      std::vector<GRBVar> ut = GetColumn(u, t);
      std::vector<GRBLinExpr> Ax_tm1 = MatrixMultiply(As, xt) + MatrixMultiply(Bs, ut);  //  x_t+1=Ax_t+Bu_t

      for (int i = 0; i < 9; i++)
      {
        std::cout << "i=" << i << "  t=" << t << std::endl;
        // std::cout << "x[i][t]" << x[i][t] << std::endl;
        model.addConstr(x[i][t + 1] == Ax_tm1[i]);
        // std::cout << "Abajo" << std::endl;
      }
    }
    // model.update();

    // Constraints x_0=x_initial
    for (int i = 0; i < 9; i++)
    {
      model.addConstr(x[i][0] == x0[i]);
    }

    GRBQuadExpr control_cost = 0;
    for (int t = 0; t < N; t++)
    {
      std::vector<GRBVar> ut = GetColumn(u, t);
      control_cost = control_cost + GetNorm2(ut);
    }

    GRBQuadExpr final_state_cost = 0;
    std::vector<GRBVar> xFinal = GetColumn(x, N);
    // std::vector<GRBLinExpr> prueba = xFinal - xf;
    final_state_cost = GetNorm2(xFinal - xf);
    final_state_cost = q * final_state_cost;

    model.setObjective(control_cost + final_state_cost, GRB_MINIMIZE);

    // Solve*/
    model.update();
    model.write("debug.lp");
    model.optimize();

    std::cout << "\nOBJECTIVE: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
    std::cout << "Positions X:" << std::endl;
    for (int t = 0; t < N + 1; t++)
    {
      std::cout << x[0][t].get(GRB_DoubleAttr_X) << std::endl;
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
