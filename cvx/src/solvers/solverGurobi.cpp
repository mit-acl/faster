#include "solverGurobi.hpp"
#include "solverGurobi_utils.hpp"  //This must go here, and not in solverGurobi.hpp

SolverGurobi::SolverGurobi()
{
  v_max_ = 5;
  a_max_ = 3;
  j_max_ = 5;
  N_ = 10;

  // Model
  /*  env = new GRBEnv();
    m = GRBModel(*env);*/
  m.set(GRB_StringAttr_ModelName, "planning");
  std::vector<std::string> coeff = { "ax", "ay", "az", "bx", "by", "bz", "cx", "cy", "cz", "dx", "dy", "dz" };

  // Variables: Coefficients of the polynomials
  for (int t = 0; t < N_ + 1; t++)
  {
    std::vector<GRBVar> row_t;
    for (int i = 0; i < 12; i++)
    {
      row_t.push_back(m.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, coeff[i] + std::to_string(t)));
    }
    x.push_back(row_t);
  }

  GRBQuadExpr control_cost = 0;
  for (int t = 0; t < N_; t++)
  {
    std::vector<GRBLinExpr> ut = { getJerk(t, 0, 0, false, x), getJerk(t, 0, 1, false, x), getJerk(t, 0, 2, false, x) };
    control_cost = control_cost + GetNorm2(ut);
  }
  m.setObjective(control_cost, GRB_MINIMIZE);

  double x0[9] = { 5, 11.5, 0.5, 0, 0, 0, 0, 0, 0 };
  double xf[9] = { 14, 5, 2.5, 0, 0, 0, 0, 0, 0 };
  double max_values[3] = { v_max_, a_max_, j_max_ };

  /*  setDC(0.01);  LO HAGO EN CVX
  set_max(max_values);  LO HAGO EN CVX

    setX0(x0);   LO HAGO EN CVX
    setXf(xf);   LO HAGO EN CVX


    setMaxConstraints();  // Only needed to set once///////////////////////


 // setPolytopes();  LO HAGO EN CVX

    genNewTraj();



  // Solve*/

  /*catch (GRBException e)
  {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  }*/
  /*catch (...)
  {
    cout << "Exception during optimization" << endl;
  }*/

  // delete env;
}

void SolverGurobi::fillXandU()
{
  double t = 0;
  int interval = 0;
  for (int i = 0; i < U_temp_.rows(); i++)
  {
    // std::cout << "row=" << i << std::endl;
    t = t + DC;
    // std::cout << "t=" << t << std::endl;
    if (t > dt_ * (interval + 1))
    {
      interval = interval + 1;
      // std::cout << "**Interval=" << interval << std::endl;
    }

    // std::cout << "t_rel=" << t - interval * dt_ << std::endl;
    double jerkx = getJerk(interval, t - interval * dt_, 0, true, x).getValue();
    double jerky = getJerk(interval, t - interval * dt_, 1, true, x).getValue();
    double jerkz = getJerk(interval, t - interval * dt_, 2, true, x).getValue();

    Eigen::Matrix<double, 1, 3> input;
    input << jerkx, jerky, jerkz;

    U_temp_.row(i) = input;

    double posx = getPos(interval, t - interval * dt_, 0, true, x).getValue();
    double posy = getPos(interval, t - interval * dt_, 1, true, x).getValue();
    double posz = getPos(interval, t - interval * dt_, 2, true, x).getValue();

    double velx = getVel(interval, t - interval * dt_, 0, true, x).getValue();
    double vely = getVel(interval, t - interval * dt_, 1, true, x).getValue();
    double velz = getVel(interval, t - interval * dt_, 2, true, x).getValue();

    double accelx = getAccel(interval, t - interval * dt_, 0, true, x).getValue();
    double accely = getAccel(interval, t - interval * dt_, 1, true, x).getValue();
    double accelz = getAccel(interval, t - interval * dt_, 2, true, x).getValue();

    Eigen::Matrix<double, 1, 9> states;
    states << posx, posy, posz, velx, vely, velz, accelx, accely, accelz;
    X_temp_.row(i) = states;
  }

  /*  std::cout << "***********The states are***********" << std::endl;
    std::cout << X_temp_ << std::endl;
    std::cout << "***********The input is***********" << std::endl;
    std::cout << U_temp_ << std::endl;*/
}

int SolverGurobi::setPolytopes(std::vector<LinearConstraint3D> l_constraints)
{
  std::cout << "Setting POLYTOPES=" << l_constraints.size() << std::endl;

  // Remove previous polytopes constraints
  for (int i = 0; i < polytopes_cons.size(); i++)
  {
    m.remove(polytopes_cons[i]);
  }
  std::cout << "abajo 1" << std::endl;

  polytopes_cons.clear();

  std::cout << "abajo 1.5" << std::endl;
  // Remove previous at_least_1_pol_cons constraints
  for (int i = 0; i < at_least_1_pol_cons.size(); i++)
  {
    m.remove(at_least_1_pol_cons[i]);
  }

  at_least_1_pol_cons.clear();

  std::cout << "abajo 2" << std::endl;
  // Remove previous binary variables  (They depend on the number of polytopes--> I can't reuse them)
  for (int i = 0; i < b.size(); i++)
  {
    for (int j = 0; j < b[i].size(); j++)
    {
      m.remove(b[i][j]);
    }
  }
  b.clear();

  std::cout << "Cleared everything" << std::endl;

  // Declare binary variables
  for (int t = 0; t < N_ + 1; t++)
  {
    std::vector<GRBVar> row;
    for (int i = 0; i < l_constraints.size(); i++)  // For all the polytopes
    {
      GRBVar variable =
          m.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_BINARY, "s" + std::to_string(i) + "_" + std::to_string(t));
      row.push_back(variable);
    }
    b.push_back(row);
  }

  std::cout << "2 everything" << std::endl;

  // Polytope constraints (if binary_varible==1 --> In that polytope) and at_least_1_pol_cons (at least one polytope)
  // constraints
  for (int t = 0; t < N_; t++)
  {
    GRBLinExpr sum = 0;
    for (int col = 0; col < b[0].size(); col++)
    {
      sum = sum + b[t][col];
    }
    at_least_1_pol_cons.push_back(m.addConstr(sum == 1));

    std::vector<GRBLinExpr> pos = { getPos(t, 0, 0, false, x), getPos(t, 0, 1, false, x), getPos(t, 0, 2, false, x) };

    std::cout << "going to poly" << std::endl;
    for (int n_poly = 0; n_poly < l_constraints.size(); n_poly++)  // Loop over the number of polytopes
    {
      // Constraint A1x<=b1
      Eigen::MatrixXd A1 = l_constraints[n_poly].A();
      auto b1 = l_constraints[n_poly].b();

      std::vector<std::vector<double>> A1std = eigenMatrix2std(A1);

      std::cout << "mas abajo" << std::endl;
      for (int i = 0; i < b1.rows(); i++)
      {
        polytopes_cons.push_back(m.addGenConstrIndicator(b[t][0], 1, MatrixMultiply(A1std, pos)[i], '<',
                                                         b1[i]));  // If b[t,0]==1, then...
      }
      /*      for (int i = 0; i < b2.rows(); i++)
            {
              m.addGenConstrIndicator(b[t][1], 1, MatrixMultiply(A2std, pos)[i], '<',
                                      b2[i]);  // If b[t,1]==1, then...
            }
            for (int i = 0; i < b3.rows(); i++)
            {
              m.addGenConstrIndicator(b[t][2], 1, MatrixMultiply(A3std, pos)[i], '<',
                                      b3[i]);  // If b[t,2]==1, then...
            }*/
    }
  }
  std::cout << "10 everything" << std::endl;
}

int SolverGurobi::getN()
{
  return N_;
}

void SolverGurobi::setDC(double dc)
{
  DC = dc;
}

Eigen::MatrixXd SolverGurobi::getX()
{
  return X_temp_;
}

Eigen::MatrixXd SolverGurobi::getU()
{
  return U_temp_;
}

void SolverGurobi::setX0(double x0[])
{
  int input_order = 3;
  for (int i = 0; i < 3 * input_order; i++)
  {
    x0_[i] = x0[i];
  }
}

void SolverGurobi::setConstraintsXf()
{
  // Remove previous final constraints
  for (int i = 0; i < final_cons.size(); i++)
  {
    m.remove(final_cons[i]);
  }

  final_cons.clear();

  // Constraint xT==x_final
  for (int i = 0; i < 3; i++)
  {
    final_cons.push_back(m.addConstr(getPos(N_ - 1, dt_, i, false, x) - xf_[i] <= 0.2));   // Final position
    final_cons.push_back(m.addConstr(getPos(N_ - 1, dt_, i, false, x) - xf_[i] >= -0.2));  // Final position

    final_cons.push_back(m.addConstr(getVel(N_ - 1, dt_, i, false, x) - xf_[i + 3] <= 0.2));   // Final velocity
    final_cons.push_back(m.addConstr(getVel(N_ - 1, dt_, i, false, x) - xf_[i + 3] >= -0.2));  // Final velocity

    final_cons.push_back(m.addConstr(getAccel(N_ - 1, dt_, i, false, x) - xf_[i + 6] <= 0.2));   // Final acceleration
    final_cons.push_back(m.addConstr(getAccel(N_ - 1, dt_, i, false, x) - xf_[i + 6] >= -0.2));  // Final acceleration
  }
}

void SolverGurobi::setConstraintsX0()
{
  // Remove previous initial constraints
  for (int i = 0; i < init_cons.size(); i++)
  {
    m.remove(init_cons[i]);
  }

  init_cons.clear();
  // Constraint x0==x_initial
  for (int i = 0; i < 3; i++)
  {
    init_cons.push_back(m.addConstr(getPos(0, 0, i, false, x) == x0_[i]));        // Initial position
    init_cons.push_back(m.addConstr(getVel(0, 0, i, false, x) == x0_[i + 3]));    // Initial velocity
    init_cons.push_back(m.addConstr(getAccel(0, 0, i, false, x) == x0_[i + 6]));  // Initial acceleration}
  }
}

void SolverGurobi::setXf(double xf[])
{
  int input_order = 3;
  for (int i = 0; i < 3 * input_order; i++)
  {
    xf_[i] = xf[i];
  }
}

void SolverGurobi::resetXandU()
{
  int size = (int)(N_)*dt_ / DC;
  // size = (size < 2) ? 2 : size;  // force size to be at least 2
  U_temp_ = Eigen::MatrixXd::Zero(size, 3);
  X_temp_ = Eigen::MatrixXd::Zero(size, 9);
}

/*double SolverGurobi::getCost()
{
  double term_cost = 0;

  for (int i = 0; i < 3 * INPUT_ORDER; i++)
  {
    term_cost = term_cost + q_ * pow(x_[N_][i] - xf_[i], 2);
  }
        printf("real total cost=%f\n", jerk_get_cost());
        printf("real jerk cost=%f\n", jerk_get_cost() - term_cost);
  cost_ = (jerk_get_cost() - term_cost) * N_ * dt_ / (j_max_ * j_max_);

  return cost_;
}*/

void SolverGurobi::setMaxConstraints()
{
  // Constraint v<=vmax, a<=amax, u<=umax
  for (int t = 0; t < N_ - 1; t++)
  {
    for (int i = 0; i < 3; i++)
    {
      m.addConstr(getVel(t, dt_, i, false, x) <= v_max_);
      m.addConstr(getVel(t, dt_, i, false, x) >= -v_max_);

      m.addConstr(getAccel(t, dt_, i, false, x) <= a_max_);
      m.addConstr(getAccel(t, dt_, i, false, x) >= -a_max_);

      m.addConstr(getJerk(t, dt_, i, false, x) <= j_max_);
      m.addConstr(getJerk(t, dt_, i, false, x) >= -j_max_);
    }
  }
}

void SolverGurobi::set_max(double max_values[3])
{
  v_max_ = max_values[0];
  a_max_ = max_values[1];
  j_max_ = max_values[2];

  setMaxConstraints();
}

// var is the type of variable to interpolate (POS=1, VEL=2, ACCEL=3,...)
/*void SolverGurobi::interpolate(int var, double** u, double** x)
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
}*/

void SolverGurobi::genNewTraj()
{
  // findDT();
  dt_ = 5.0 / N_;
  setConstraintsX0();
  setConstraintsXf();
  setDynamicConstraints();
  // printf("In genNewTraj\n");
  std::cout << "callOptimizer, x0_=" << x0_[0] << x0_[1] << x0_[2] << "xf_=" << xf_[0] << xf_[1] << xf_[2] << std::endl;

  resetXandU();
  callOptimizer();
  fillXandU();
  // printf("In genNewTraj0.5\n");
  /*  resetXandU();
    // printf("In genNewTraj1\n");

    x_ = jerk_get_state();
    u_ = jerk_get_control();

    interpolate(POS, u_, x_);    // interpolate POS
    interpolate(VEL, u_, x_);    // ...
    interpolate(ACCEL, u_, x_);  // ...
    interpolate(JERK, u_, x_);*/
}

void SolverGurobi::findDT()
{
  double dt = 4 * getDTInitial();
  dt_ = dt;
}

void SolverGurobi::setDynamicConstraints()
{
  // Remove the previous dynamic constraints
  for (int i = 0; i < dyn_cons.size(); i++)
  {
    m.remove(dyn_cons[i]);
  }
  dyn_cons.clear();

  // Dynamic Constraints
  for (int t = 0; t < N_ - 1; t++)
  {
    for (int i = 0; i < 3; i++)
    {
      dyn_cons.push_back(
          m.addConstr(getPos(t, dt_, i, false, x) == getPos(t + 1, 0, i, false, x)));  // Continuity in position
      dyn_cons.push_back(
          m.addConstr(getVel(t, dt_, i, false, x) == getVel(t + 1, 0, i, false, x)));  // Continuity in velocity
      dyn_cons.push_back(
          m.addConstr(getAccel(t, dt_, i, false, x) == getAccel(t + 1, 0, i, false, x)));  // Continuity in acceleration
    }
  }
}

void SolverGurobi::callOptimizer()
{
  std::cout << "CALLING OPTIMIZER OF GUROBI" << std::endl;
  m.update();
  m.write("debug_gurobi.lp");
  std::cout << "WRITTEN" << std::endl;
  m.optimize();

  std::cout << "\nOBJECTIVE: " << m.get(GRB_DoubleAttr_ObjVal) << std::endl;
  std::cout << "Positions X:" << std::endl;
  for (int t = 0; t < N_ + 1; t++)
  {
    std::cout << getPos(t, 0, 0, true, x) << std::endl;
  }

  /*  // printf("In callOptimizer\n");
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

      jerk_load_default_data(dt, v_max_, a_max_, j_max_, x0_, xf_, q_);
      r = jerk_optimize();
      if (r == 1)
      {
        x = jerk_get_state();
        converged = checkConvergence(x[N_]);
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
    // ROS_INFO("converged, dt = %f", dt);*/
}

double SolverGurobi::getDTInitial()
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

  std::cout << "get DT, x0_=" << x0_[0] << x0_[1] << x0_[2] << "xf_=" << xf_[0] << xf_[1] << xf_[2] << std::endl;
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

  std::cout << "Coefficients" << coeffx.transpose() << coeffy.transpose() << coeffz.transpose() << std::endl;

  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolvex(coeffx);
  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolvey(coeffy);
  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolvez(coeffz);

  std::vector<double> realRootsx;
  std::vector<double> realRootsy;
  std::vector<double> realRootsz;
  psolvex.realRoots(realRootsx);
  psolvey.realRoots(realRootsy);
  psolvez.realRoots(realRootsz);

  printf("before min elements\n");
  t_jx = *std::min_element(realRootsx.begin(), realRootsx.end());
  t_jy = *std::min_element(realRootsy.begin(), realRootsy.end());
  t_jz = *std::min_element(realRootsz.begin(), realRootsz.end());

  printf("after: t_jx, t_jy, t_jz:\n");
  std::cout << t_jx << "  " << t_jy << "  " << t_jz << std::endl;

  float accelx = copysign(1, xf_[0] - x0_[0]) * a_max_;
  float accely = copysign(1, xf_[1] - x0_[1]) * a_max_;
  float accelz = copysign(1, xf_[2] - x0_[2]) * a_max_;
  // Solve equation xf=x0+v0t+0.5*a*t^2
  t_ax = solvePolynomialOrder2(Eigen::Vector3f(0.5 * accelx, v0x, x0_[0] - xf_[0]));
  t_ay = solvePolynomialOrder2(Eigen::Vector3f(0.5 * accely, v0y, x0_[1] - xf_[1]));
  t_az = solvePolynomialOrder2(Eigen::Vector3f(0.5 * accelz, v0z, x0_[2] - xf_[2]));

  dt_initial = std::max({ t_vx, t_vy, t_vz, t_ax, t_ay, t_az, t_jx, t_jy, t_jz }) / N_;
  if (dt_initial > 10000)  // happens when there is no solution to the previous eq.
  {
    printf("there is not a solution to the previous equations");
    dt_initial = 0;
  }
  // printf("dt_initial=%f", dt_initial);
  return dt_initial;
}

/*int main(int argc, char* argv[])
{
  SolverGurobi();
}*/