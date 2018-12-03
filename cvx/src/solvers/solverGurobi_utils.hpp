/* Author: Jesus Tordesillas Torres
   1-November-2018
   I tried during 4 hours to implement it using Eigen, but didn't succeed. I think it's possible using NumTraits and
   ScalarBinaryOpTraits templates of Eigen (to allow "symbolic" multiplications)
 */

// Continuous version, and with polytope constraints.

#pragma once
#ifndef SOLVER_GUROBI_UTILS_HPP
#define SOLVER_GUROBI_UTILS_HPP

#include "gurobi_c++.h"
#include <sstream>
#include <Eigen/Dense>
#include <type_traits>
// using namespace std;

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

template <typename T>
std::vector<GRBLinExpr> MatrixMultiply(const std::vector<std::vector<double>>& A, const std::vector<T>& x)
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

template <typename T>
std::vector<GRBLinExpr> operator-(const std::vector<T>& x, const std::vector<double>& b)
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

#endif