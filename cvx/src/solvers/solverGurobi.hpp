#ifndef SOLVER_GUROBI_HPP
#define SOLVER_GUROBI_HPP
#include <Eigen/Dense>
#include "gurobi_c++.h"
#include <sstream>
#include <Eigen/Dense>
#include <type_traits>
#include <fstream>

//#include "../utils.hpp"
// using namespace std;

//#include "mlinterp.hpp"
// Convex Decomposition includes
#include <decomp_ros_utils/data_ros_utils.h>
#include <unsupported/Eigen/Polynomials>

// TODO: This function is the same as solvePolyOrder2 but with other name (weird conflicts...)
inline float solvePolynomialOrder2(Eigen::Vector3f coeff)
{
  // std::cout << "solving\n" << coeff << std::endl;
  float a = coeff[0];
  float b = coeff[1];
  float c = coeff[2];
  float dis = b * b - 4 * a * c;
  if (dis >= 0)
  {
    float x1 = (-b - sqrt(dis)) / (2 * a);  // x1 will always be smaller than x2
    float x2 = (-b + sqrt(dis)) / (2 * a);

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

class SolverGurobi
{
public:
  SolverGurobi();
  void interpolate(int var, double** u, double** x);
  void obtainByDerivation(double** u, double** x);
  double getCost();
  Eigen::MatrixXd getX();
  Eigen::MatrixXd getU();
  void setQ(double q);
  void setN(int N);
  void setX0(double x0[]);
  // void set_u0(double u0[]);
  void setXf(double xf[]);
  void resetXandU();
  void set_max(double max_values[3]);
  void genNewTraj();
  void callOptimizer();
  double getDTInitial();
  int getN();
  void setDC(double dc);
  int setPolytopes(std::vector<LinearConstraint3D> l_constraints);
  void findDT();
  void fillXandU();
  void setObjective();
  void setConstraintsXf();
  void setConstraintsX0();
  void setDynamicConstraints();

  void setMaxConstraints();
  void createVars();

  void setDistances(vec_Vecf<3>& samples, std::vector<double> dist_near_obs);
  void setDistanceConstraints();

protected:
  Eigen::MatrixXd U_temp_;
  Eigen::MatrixXd X_temp_;
  double cost_;
  double dt_;  // time step found by the solver
  int N_ = 10;
  double xf_[3 * 3];
  double x0_[3 * 3];
  double v_max_;
  double a_max_;
  double j_max_;
  double DC;
  double q_;  // weight to the 2nd term in the cost function
  double** x_;
  double** u_;

  int N_of_polytopes_ = 3;

  GRBEnv* env = new GRBEnv();
  GRBModel m = GRBModel(*env);

  std::vector<GRBConstr> at_least_1_pol_cons;  // Constraints at least in one polytope
  std::vector<GRBGenConstr> polytopes_cons;
  std::vector<GRBConstr> dyn_cons;
  std::vector<GRBConstr> init_cons;
  std::vector<GRBConstr> final_cons;

  std::vector<GRBQConstr> distances_cons;

  std::vector<std::vector<GRBVar>> b;  // binary variables
  std::vector<std::vector<GRBVar>> x;
  std::vector<std::vector<GRBVar>> u;

  vec_Vecf<3> samples_;  // Samples along the rescue path
  std::vector<double> dist_near_obs_;

  std::ofstream times_log;

  int temporal = 0;
};
#endif