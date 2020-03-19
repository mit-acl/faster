/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once
#ifndef SOLVER_GUROBI_UTILS_HPP
#define SOLVER_GUROBI_UTILS_HPP

#include "gurobi_c++.h"
#include <sstream>
#include <Eigen/Dense>
#include <type_traits>
// using namespace std;

inline double MinPositiveElement(std::vector<double> v)
{
  std::sort(v.begin(), v.end());  // sorted in ascending order
  double min_value = 0;
  for (int i = 0; i < v.size(); i++)
  {
    if (v[i] > 0)
    {
      min_value = v[i];
      break;
    }
  }
  return min_value;
}

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

/*std::vector<GRBLinExpr> MatrixMultiply(const std::vector<std::vector<double>>& A, const std::vector<GRBLinExpr>& x)
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
}*/

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

#endif