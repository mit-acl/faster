#include "solver.h"
#include "interface_vel.h"

#define CONCATENATE_INPUT(x) vel_##x

Vars vars;
Params params;
Workspace work;
Settings settings;

int CONCATENATE_INPUT(initialize_optimizer)(void)
{
  set_defaults();
  setup_indexing();
}

int CONCATENATE_INPUT(optimize)(void)
{
  // printf("in optimize vel\n");
  solve();
  // for(i=1;i<19;i++){
  // printf("%0.2f %0.2f %0.2f \n",vars.x[i][0],vars.x[i][3],vars.u[i][0]);
  // }
  // printf("%0.2f %0.2f %0.2f \n",vars.x[19][3],vars.x[19][4],vars.x[19][5]);
  return work.converged;
}

double CONCATENATE_INPUT(get_cost)(void)
{
  return eval_objv();
}

double** CONCATENATE_INPUT(get_state)(void)
{
  return vars.x;
}

double** CONCATENATE_INPUT(get_control)(void)
{
  return vars.u;
}

void CONCATENATE_INPUT(load_default_data)(double dt, double v_max, double x0[], double xf[], double q)
{
  // printf("loading data vel\n");
  // double dt = 0.5;
  // double q = 100000;
  params.xf[0] = xf[0];
  params.xf[1] = xf[1];
  params.xf[2] = xf[2];

  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  // First Row
  params.Q_final[0] = q;
  params.Q_final[3] = 0;
  params.Q_final[6] = 0;
  // Second Row
  params.Q_final[1] = 0;
  params.Q_final[4] = q;
  params.Q_final[7] = 0;
  // Third Row
  params.Q_final[2] = 0;
  params.Q_final[5] = 0;
  params.Q_final[8] = q;

  // Note that the indexing is 0....2 (first column), 6...17 (second column)..
  // First column
  params.A[0] = 1;
  params.A[1] = 0;
  params.A[2] = 0;
  // Second column
  params.A[3] = 0;
  params.A[4] = 1;
  params.A[5] = 0;
  // Third column
  params.A[6] = 0;
  params.A[7] = 0;
  params.A[8] = 1;

  // End of A
  params.x_0[0] = x0[0];
  params.x_0[1] = x0[1];
  params.x_0[2] = x0[2];
  // B
  // 1st column
  params.B[0] = dt;
  params.B[1] = 0;
  params.B[2] = 0;
  // 2nd column
  params.B[3] = 0;
  params.B[4] = dt;
  params.B[5] = 0;
  // 3rd column
  params.B[6] = 0;
  params.B[7] = 0;
  params.B[8] = dt;

  params.v_max[0] = v_max;
}

#undef CONCATENATE_INPUT
