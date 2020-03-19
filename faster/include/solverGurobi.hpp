/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#ifndef SOLVER_GUROBI_HPP
#define SOLVER_GUROBI_HPP
#include <Eigen/Dense>
#include "gurobi_c++.h"
#include <sstream>
#include <Eigen/Dense>
#include <type_traits>
#include <fstream>
#include "termcolor.hpp"

#include <decomp_ros_utils/data_ros_utils.h>
#include <unsupported/Eigen/Polynomials>
#include "faster_types.hpp"
using namespace termcolor;

// TODO: This function is the same as solvePolyOrder2 but with other name (weird conflicts...)
inline double solvePolynomialOrder2(Eigen::Vector3f& coeff)
{
  // std::cout << "solving\n" << coeff.transpose() << std::endl;
  double a = coeff(0);
  double b = coeff(1);
  double c = coeff(2);
  double dis = b * b - 4 * a * c;
  if (dis >= 0)
  {
    double x1 = (-b - sqrt(dis)) / (2 * a);  // x1 will always be smaller than x2
    double x2 = (-b + sqrt(dis)) / (2 * a);

    if (x1 >= 0)
    {
      return x1;
    }
    if (x2 >= 0)
    {
      return x2;
    }
  }
  printf("No solution found to the equation\n");
  return std::numeric_limits<float>::max();
}

class mycallback : public GRBCallback
{
public:
  bool should_terminate_;
  mycallback();  // constructor
  // void abortar();

protected:
  void callback();
};

class SolverGurobi
{
public:
  SolverGurobi();

  // void setQ(double q);
  void setN(int N);
  void setX0(state& data);
  // void set_u0(double u0[]);
  void setXf(state& data);
  void resetX();
  void setBounds(double max_values[3]);
  bool genNewTraj();
  bool callOptimizer();
  double getDTInitial();

  void setDC(double dc);
  void setPolytopes(std::vector<LinearConstraint3D> polytopes);
  void setPolytopesConstraints();
  void findDT(double factor);
  void fillX();
  void setObjective();
  void setConstraintsXf();
  void setConstraintsX0();
  void setDynamicConstraints();
  void setForceFinalConstraint(bool forceFinalConstraint);

  // For the jackal
  void setWMax(double w_max);
  bool isWmaxSatisfied();

  void setMaxConstraints();
  void createVars();
  void setThreads(int threads);
  void setVerbose(int verbose);

  void StopExecution();
  void ResetToNormalState();

  void setDistances(vec_Vecf<3>& samples, std::vector<double> dist_near_obs);

  // void setSamplesPenalize(vec_Vecf<3>& samples_penalize);

  void setDistanceConstraints();

  void setMode(int mode);
  void setFactorInitialAndFinalAndIncrement(double factor_initial, double factor_final, double factor_increment);

  GRBLinExpr getPos(int t, double tau, int ii);
  GRBLinExpr getVel(int t, double tau, int ii);
  GRBLinExpr getAccel(int t, double tau, int ii);
  GRBLinExpr getJerk(int t, double tau, int ii);

  GRBLinExpr getA(int t, int ii);
  GRBLinExpr getB(int t, int ii);
  GRBLinExpr getC(int t, int ii);
  GRBLinExpr getD(int t, int ii);

  // Getters of the Normalized coefficients
  GRBLinExpr getAn(int t, int ii);
  GRBLinExpr getBn(int t, int ii);
  GRBLinExpr getCn(int t, int ii);
  GRBLinExpr getDn(int t, int ii);

  std::vector<GRBLinExpr> getCP0(int t);
  std::vector<GRBLinExpr> getCP1(int t);
  std::vector<GRBLinExpr> getCP2(int t);
  std::vector<GRBLinExpr> getCP3(int t);

  std::vector<state> X_temp_;
  double dt_;  // time step found by the solver
  int trials_ = 0;
  int temporal_ = 0;
  double runtime_ms_ = 0;
  double factor_that_worked_ = 0;
  int N_ = 10;
  mycallback cb_;

protected:
  double cost_;

  double xf_[3 * 3];
  double x0_[3 * 3];
  double v_max_;
  double a_max_;
  double j_max_;
  double DC;
  // double q_;  // weight to the 2nd term in the cost function
  double** x_;
  double** u_;

  int N_of_polytopes_ = 3;

  GRBEnv* env = new GRBEnv();
  GRBModel m = GRBModel(*env);

  std::vector<GRBConstr> at_least_1_pol_cons;  // Constraints at least in one polytope
  std::vector<GRBGenConstr> polytopes_cons;    // Used for the whole trajectory
  std::vector<GRBConstr> polytope_cons;        // Used for the rescue path
  std::vector<GRBConstr> dyn_cons;
  std::vector<GRBConstr> init_cons;
  std::vector<GRBConstr> final_cons;

  std::vector<GRBQConstr> distances_cons;

  std::vector<std::vector<GRBVar>> b;  // binary variables
  std::vector<std::vector<GRBVar>> x;
  std::vector<std::vector<GRBVar>> u;

  vec_Vecf<3> samples_;           // Samples along the rescue path
  vec_Vecf<3> samples_penalize_;  // Samples along the rescue path

  std::vector<double> dist_near_obs_;
  std::vector<LinearConstraint3D> polytopes_;

  std::ofstream times_log;

  int mode_;
  bool forceFinalConstraint_ = true;
  double factor_initial_ = 2;
  double factor_final_ = 2;
  double factor_increment_ = 2;

  int total_not_solved = 0;
  double w_max_ = 1;
};
#endif