#include "solverGurobi.hpp"
#include "solverGurobi_utils.hpp"  //This must go here, and not in solverGurobi.hpp
#include <chrono>
#include <unistd.h>

SolverGurobi::SolverGurobi()
{
  v_max_ = 5;
  a_max_ = 3;
  j_max_ = 5;
  // N_ = 10;  // Segments: 0,1,...,N_-1

  // Model
  /*  env = new GRBEnv();
    m = GRBModel(*env);*/
  m.set(GRB_StringAttr_ModelName, "planning");

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

void SolverGurobi::setN(int N)
{
  N_ = N;
}

void SolverGurobi::createVars()
{
  std::vector<std::string> coeff = { "ax", "ay", "az", "bx", "by", "bz", "cx", "cy", "cz", "dx", "dy", "dz" };

  // Variables: Coefficients of the polynomials
  for (int t = 0; t < N_; t++)
  {
    std::vector<GRBVar> row_t;
    for (int i = 0; i < 12; i++)
    {
      row_t.push_back(m.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, coeff[i] + std::to_string(t)));
    }
    x.push_back(row_t);
  }
}

void SolverGurobi::setObjective()  // I need to set it every time, because the objective depends on the xFinal
{
  GRBQuadExpr control_cost = 0;
  GRBQuadExpr final_state_cost = 0;
  std::vector<GRBLinExpr> xFinal = {
    getPos(N_ - 1, 0, 0, false, x),   getPos(N_ - 1, 0, 1, false, x),
    getPos(N_ - 1, 0, 2, false, x),  //////////////////////////////////
    getVel(N_ - 1, 0, 0, false, x),   getVel(N_ - 1, 0, 1, false, x),
    getVel(N_ - 1, 0, 2, false, x),  /////////////////////////////
    getAccel(N_ - 1, 0, 0, false, x), getAccel(N_ - 1, 0, 1, false, x),
    getAccel(N_ - 1, 0, 2, false, x)  /////////////////////////////
  };
  std::vector<double> xf(std::begin(xf_), std::end(xf_));  // Transform xf_ into a std vector
  final_state_cost = q_ * GetNorm2(xFinal - xf);

  for (int t = 0; t < N_; t++)
  {
    std::vector<GRBLinExpr> ut = { getJerk(t, 0, 0, false, x), getJerk(t, 0, 1, false, x), getJerk(t, 0, 2, false, x) };
    control_cost = control_cost + GetNorm2(ut);
  }
  m.setObjective(control_cost + final_state_cost, GRB_MINIMIZE);
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
      // std::cout << "*****Interval=" << interval << std::endl;
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

  // Force the final states to be the final conditions
  Eigen::Matrix<double, 1, 9> final_cond;
  final_cond << xf_[0], xf_[1], xf_[2], xf_[3], xf_[4], xf_[5], xf_[6], xf_[7], xf_[8];
  X_temp_.row(X_temp_.rows() - 1) = final_cond;

  // Force the final input to be 0 (I'll keep applying this input if when I arrive to the final state I still
  // haven't planned again).
  U_temp_.row(U_temp_.rows() - 1) = Eigen::Vector3d::Zero().transpose();

  /*  std::cout << "***********The final states are***********" << std::endl;
    std::cout << X_temp_.row(X_temp_.rows() - 1).transpose() << std::endl;

    std::cout << "***********The final conditions were***********" << std::endl;
    std::cout << xf_[0] << std::endl;
    std::cout << xf_[1] << std::endl;
    std::cout << xf_[2] << std::endl;
    std::cout << xf_[3] << std::endl;
    std::cout << xf_[4] << std::endl;
    std::cout << xf_[5] << std::endl;
    std::cout << xf_[6] << std::endl;
    std::cout << xf_[7] << std::endl;
    std::cout << xf_[8] << std::endl;*/

  /*  std::cout << "***********The states are***********" << std::endl;
    std::cout << X_temp_ << std::endl;
    std::cout << "***********The input is***********" << std::endl;
    std::cout << U_temp_ << std::endl;*/
}

void SolverGurobi::setDistances(vec_Vecf<3>& samples,
                                std::vector<double> dist_near_obs)  // Distance values (used for the
                                                                    // rescue path, used in the distance constraint4s)
{
  samples_ = samples;
  dist_near_obs_ = dist_near_obs;
}

void SolverGurobi::setDistanceConstraints()  // Set the distance constraints
{
  // Remove previous distance constraints
  for (int i = 0; i < distances_cons.size(); i++)
  {
    m.remove(distances_cons[i]);
  }
  distances_cons.clear();

  for (int t = 0; t < samples_.size(); t++)
  {
    std::vector<GRBLinExpr> p = { samples_[t][0], samples_[t][1], samples_[t][2] };
    double tau = 0;
    double interval = t;
    if (t == samples_.size() - 1)  // If the last interval --> constraint at the end of that interval
    {
      tau = dt_;
      interval = t - 1;
    }
    GRBLinExpr posx = getPos(interval, tau, 0, false, x);
    GRBLinExpr posy = getPos(interval, tau, 1, false, x);
    GRBLinExpr posz = getPos(interval, tau, 2, false, x);
    std::vector<GRBLinExpr> var = { posx, posy, posz };
    double epsilon = dist_near_obs_[t] * dist_near_obs_[t];
    // printf("Cons with distance=%f ", sqrt(epsilon));
    // std::cout << "For the sample=" << samples_[t].transpose() << std::endl;

    // TODO: THERE IS SOMETHING THAT IS WRONG HERE, IT DOESNT WORK!! Not sure if my fault, or Gurobi's one
    // ONE POSSIBLE WORKAROUND IS TRANSFORM THIS INTO A BOX CONSTRAINT (WITH ABSOLUTES VALUES, INSTEAD OF A QUADRATIC
    // CONSTRAINT)
    distances_cons.push_back(m.addQConstr(GetNorm2(var - p) <= epsilon));
  }
}

int SolverGurobi::setPolytopes(std::vector<LinearConstraint3D> l_constraints)
{
  // std::cout << "Setting POLYTOPES=" << l_constraints.size() << std::endl;

  // Remove previous polytopes constraints
  for (int i = 0; i < polytopes_cons.size(); i++)
  {
    m.remove(polytopes_cons[i]);
  }

  polytopes_cons.clear();

  // Remove previous at_least_1_pol_cons constraints
  for (int i = 0; i < at_least_1_pol_cons.size(); i++)
  {
    m.remove(at_least_1_pol_cons[i]);
  }

  at_least_1_pol_cons.clear();

  // Remove previous binary variables  (They depend on the number of polytopes--> I can't reuse them)
  for (int i = 0; i < b.size(); i++)
  {
    for (int j = 0; j < b[i].size(); j++)
    {
      m.remove(b[i][j]);
    }
  }
  b.clear();

  if (l_constraints.size() > 0)  // If there are polytope constraints
  {
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

      for (int n_poly = 0; n_poly < l_constraints.size(); n_poly++)  // Loop over the number of polytopes
      {
        // Constraint A1x<=b1
        Eigen::MatrixXd A1 = l_constraints[n_poly].A();
        auto b1 = l_constraints[n_poly].b();

        std::vector<std::vector<double>> A1std = eigenMatrix2std(A1);

        std::vector<GRBLinExpr> Ax = MatrixMultiply(A1std, pos);
        for (int i = 0; i < b1.rows(); i++)
        {
          // Now let's shrink the polytopes with the size of the drone
          double norm_a = (A1.row(i)).norm();
          double distance = 0;  ///!!!!CHANGE THIS, SETTING THE DISTANCE TO THE ONE OF THE DRONE
          polytopes_cons.push_back(m.addGenConstrIndicator(b[t][n_poly], 1, Ax[i], '<',
                                                           b1[i] - distance * norm_a));  // If b[t,0]==1, then...
        }
      }
    }
  }
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
  // printf("Setting initial condition:\n");
  int input_order = 3;
  for (int i = 0; i < 9; i++)
  {
    // std::cout << x0[i] << "  ";
    x0_[i] = x0[i];
  }
  // std::cout << std::endl;
}

void SolverGurobi::setXf(double xf[])
{
  printf("Setting final condition:\n");

  for (int i = 0; i < 9; i++)
  {
    std::cout << xf[i] << "  ";
    xf_[i] = xf[i];
  }
  std::cout << std::endl;
}

void SolverGurobi::setConstraintsXf()
{
  // Remove previous final constraints
  for (int i = 0; i < final_cons.size(); i++)
  {
    m.remove(final_cons[i]);
  }

  final_cons.clear();

  // std::cout << "Setting Final Constraints!!" << std::endl;
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

void SolverGurobi::setQ(double q)
{
  q_ = q;
}

void SolverGurobi::set_max(double max_values[3])
{
  v_max_ = max_values[0];
  a_max_ = max_values[1];
  j_max_ = max_values[2];

  setMaxConstraints();
}

bool SolverGurobi::genNewTraj()
{
  bool solved;
  findDT();
  // dt_ = 5.0 / N_;

  setConstraintsX0();
  setConstraintsXf();
  setDynamicConstraints();
  setDistanceConstraints();
  // printf("In genNewTraj\n");
  /*  std::cout << "dt is=" << dt_ << std::endl;
    std::cout << "callOptimizer, x0_=" << x0_[0] << " " << x0_[1] << " " << x0_[2] << " "
              << "xf_=" << xf_[0] << " " << xf_[1] << " " << xf_[2] << " " << std::endl;*/

  setObjective();

  resetXandU();
  solved = callOptimizer();
  if (solved == true)
  {
    fillXandU();
  }
  return solved;
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
  // dt_ = 1;
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
  for (int t = 0; t < N_ - 1; t++)  // From 0....N_-2
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

bool SolverGurobi::callOptimizer()
{
  bool solved = true;
  // std::cout << "CALLING OPTIMIZER OF GUROBI" << std::endl;

  // Select these parameteres with the tuning Tool of Gurobi
  // m.set("MIPFocus", "2");
  // m.set("PreQLinearize", "1");

  m.update();
  temporal = temporal + 1;
  printf("Writing into model.lp number=%d", temporal);
  m.write("/home/jtorde/Desktop/ws/src/acl-planning/cvx/models/model_" + std::to_string(temporal) + ".lp");
  m.set("OutputFlag", "0");  // 1 if you want verbose

  std::cout << "*************************Starting Optimization" << std::endl;
  auto start = std::chrono::steady_clock::now();
  m.optimize();
  auto end = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  std::cout << "*************************Finished Optimization: " << elapsed << " ms" << std::endl;
  std::cout << "*************************Gurobi RUNTIME: " << m.get(GRB_DoubleAttr_Runtime) * 1000 << " ms"
            << std::endl;

  times_log.open("/home/jtorde/Desktop/ws/src/acl-planning/cvx/models/times_log.txt", std::ios_base::app);
  times_log << elapsed << "\n";
  times_log.close();

  printf("Going to check status");
  int optimstatus = m.get(GRB_IntAttr_Status);
  if (optimstatus == GRB_OPTIMAL)
  {
    printf("GUROBI SOLUTION: Optimal");

    /*    if (polytopes_cons.size() > 0)  // Print the binary matrix only if I've included the polytope constraints
        {
          std::cout << "Solution: Binary Matrix:" << std::endl;

          for (int poly = 0; poly < b[0].size(); poly++)
          {
            for (int t = 0; t < N_; t++)
            {
              std::cout << b[t][poly].get(GRB_DoubleAttr_X) << " ";
            }
            std::cout << std::endl;
          }
        }
        std::cout << "Solution: Positions:" << std::endl;
        for (int t = 0; t < N_; t++)
        {
          std::cout << getPos(t, 0, 0, true, x) << "   ";
          std::cout << getPos(t, 0, 1, true, x) << "   ";
          std::cout << getPos(t, 0, 2, true, x) << std::endl;
        }

        std::cout << getPos(N_ - 1, dt_, 0, true, x) << "   ";
        std::cout << getPos(N_ - 1, dt_, 1, true, x) << "   ";
        std::cout << getPos(N_ - 1, dt_, 2, true, x) << std::endl;

        std::cout << "Solution: Coefficients d:" << std::endl;
        for (int t = 0; t < N_; t++)
        {
          std::cout << x[t][9].get(GRB_DoubleAttr_X) << "   ";
          std::cout << x[t][10].get(GRB_DoubleAttr_X) << "   ";
          std::cout << x[t][11].get(GRB_DoubleAttr_X) << std::endl;
        }*/
  }

  else
  {  // No solution
    solved = false;
    if (optimstatus == GRB_INF_OR_UNBD)
    {
      printf("GUROBI SOLUTION: Unbounded or Infeasible. Maybe too small dt?");
    }

    if (optimstatus == GRB_NUMERIC)
    {
      printf("GUROBI Status: Numerical issues");
      printf("Model may be infeasible or unbounded");  // Taken from the Gurobi documentation
    }
  }
  return solved;

  // printf("Going out from callOptimizer, optimstatus= %d\n", optimstatus);
  // std::cout << "*************************Finished Optimization" << std::endl;

  /*  std::cout << "\nOBJECTIVE: " << m.get(GRB_DoubleAttr_ObjVal) << std::endl;


*/

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

  // std::cout << "get DT, x0_=" << x0_[0] << x0_[1] << x0_[2] << "xf_=" << xf_[0] << xf_[1] << xf_[2] << std::endl;
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

  // std::cout << "Coefficients" << coeffx.transpose() << coeffy.transpose() << coeffz.transpose() << std::endl;

  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolvex(coeffx);
  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolvey(coeffy);
  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolvez(coeffz);

  std::vector<double> realRootsx;
  std::vector<double> realRootsy;
  std::vector<double> realRootsz;
  psolvex.realRoots(realRootsx);
  psolvey.realRoots(realRootsy);
  psolvez.realRoots(realRootsz);

  t_jx = *std::min_element(realRootsx.begin(), realRootsx.end());
  t_jy = *std::min_element(realRootsy.begin(), realRootsy.end());
  t_jz = *std::min_element(realRootsz.begin(), realRootsz.end());

  // printf("after: t_jx, t_jy, t_jz:\n");
  // std::cout << t_jx << "  " << t_jy << "  " << t_jz << std::endl;

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