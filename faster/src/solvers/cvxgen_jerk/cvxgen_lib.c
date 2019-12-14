/* Produced by CVXGEN, 2018-03-30 10:48:24 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */

#include "solver.h"
#include "interface_jerk.h"

#define CONCATENATE_INPUT(x) jerk_##x

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
  // printf("in optimize jerk\n");
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

void CONCATENATE_INPUT(load_default_data)(double dt, double v_max, double a_max, double j_max, double x0[], double xf[],
                                          double q)
{
  // printf("loading data jerk\n");
  // double dt = 0.5;
  // double q = 100000;
  params.xf[0] = xf[0];
  params.xf[1] = xf[1];
  params.xf[2] = xf[2];
  params.xf[3] = xf[3];
  params.xf[4] = xf[4];
  params.xf[5] = xf[5];
  params.xf[6] = xf[6];
  params.xf[7] = xf[7];
  params.xf[8] = xf[8];

  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q_final[0] = q;
  params.Q_final[9] = 0;
  params.Q_final[18] = 0;
  params.Q_final[27] = 0;
  params.Q_final[36] = 0;
  params.Q_final[45] = 0;
  params.Q_final[54] = 0;
  params.Q_final[63] = 0;
  params.Q_final[72] = 0;
  params.Q_final[1] = 0;
  params.Q_final[10] = q;
  params.Q_final[19] = 0;
  params.Q_final[28] = 0;
  params.Q_final[37] = 0;
  params.Q_final[46] = 0;
  params.Q_final[55] = 0;
  params.Q_final[64] = 0;
  params.Q_final[73] = 0;
  params.Q_final[2] = 0;
  params.Q_final[11] = 0;
  params.Q_final[20] = q;
  params.Q_final[29] = 0;
  params.Q_final[38] = 0;
  params.Q_final[47] = 0;
  params.Q_final[56] = 0;
  params.Q_final[65] = 0;
  params.Q_final[74] = 0;
  params.Q_final[3] = 0;
  params.Q_final[12] = 0;
  params.Q_final[21] = 0;
  params.Q_final[30] = q;
  params.Q_final[39] = 0;
  params.Q_final[48] = 0;
  params.Q_final[57] = 0;
  params.Q_final[66] = 0;
  params.Q_final[75] = 0;
  params.Q_final[4] = 0;
  params.Q_final[13] = 0;
  params.Q_final[22] = 0;
  params.Q_final[31] = 0;
  params.Q_final[40] = q;
  params.Q_final[49] = 0;
  params.Q_final[58] = 0;
  params.Q_final[67] = 0;
  params.Q_final[76] = 0;
  params.Q_final[5] = 0;
  params.Q_final[14] = 0;
  params.Q_final[23] = 0;
  params.Q_final[32] = 0;
  params.Q_final[41] = 0;
  params.Q_final[50] = q;
  params.Q_final[59] = 0;
  params.Q_final[68] = 0;
  params.Q_final[77] = 0;
  params.Q_final[6] = 0;
  params.Q_final[15] = 0;
  params.Q_final[24] = 0;
  params.Q_final[33] = 0;
  params.Q_final[42] = 0;
  params.Q_final[51] = 0;
  params.Q_final[60] = q;
  params.Q_final[69] = 0;
  params.Q_final[78] = 0;
  params.Q_final[7] = 0;
  params.Q_final[16] = 0;
  params.Q_final[25] = 0;
  params.Q_final[34] = 0;
  params.Q_final[43] = 0;
  params.Q_final[52] = 0;
  params.Q_final[61] = 0;
  params.Q_final[70] = q;
  params.Q_final[79] = 0;
  params.Q_final[8] = 0;
  params.Q_final[17] = 0;
  params.Q_final[26] = 0;
  params.Q_final[35] = 0;
  params.Q_final[44] = 0;
  params.Q_final[53] = 0;
  params.Q_final[62] = 0;
  params.Q_final[71] = 0;
  params.Q_final[80] = q;
  // Note that the indexing is 0....8 (first column), 9...17 (second column)..
  // First column
  params.A[0] = 1;
  params.A[1] = 0;
  params.A[2] = 0;
  params.A[3] = 0;
  params.A[4] = 0;
  params.A[5] = 0;
  params.A[6] = 0;
  params.A[7] = 0;
  params.A[8] = 0;
  // Second column
  params.A[9] = 0;
  params.A[10] = 1;
  params.A[11] = 0;
  params.A[12] = 0;
  params.A[13] = 0;
  params.A[14] = 0;
  params.A[15] = 0;
  params.A[16] = 0;
  params.A[17] = 0;
  // Third column
  params.A[18] = 0;
  params.A[19] = 0;
  params.A[20] = 1;
  params.A[21] = 0;
  params.A[22] = 0;
  params.A[23] = 0;
  params.A[24] = 0;
  params.A[25] = 0;
  params.A[26] = 0;
  // 4th column
  params.A[27] = dt;
  params.A[28] = 0;
  params.A[29] = 0;
  params.A[30] = 1;
  params.A[31] = 0;
  params.A[32] = 0;
  params.A[33] = 0;
  params.A[34] = 0;
  params.A[35] = 0;
  // 5th column
  params.A[36] = 0;
  params.A[37] = dt;
  params.A[38] = 0;
  params.A[39] = 0;
  params.A[40] = 1;
  params.A[41] = 0;
  params.A[42] = 0;
  params.A[43] = 0;
  params.A[44] = 0;
  // 6th column
  params.A[45] = 0;
  params.A[46] = 0;
  params.A[47] = dt;
  params.A[48] = 0;
  params.A[49] = 0;
  params.A[50] = 1;
  params.A[51] = 0;
  params.A[52] = 0;
  params.A[53] = 0;
  // 7th column
  params.A[54] = 0.5 * dt * dt;
  params.A[55] = 0;
  params.A[56] = 0;
  params.A[57] = dt;
  params.A[58] = 0;
  params.A[59] = 0;
  params.A[60] = 1;
  params.A[61] = 0;
  params.A[62] = 0;
  // 8th column
  params.A[63] = 0;
  params.A[64] = 0.5 * dt * dt;
  params.A[65] = 0;
  params.A[66] = 0;
  params.A[67] = dt;
  params.A[68] = 0;
  params.A[69] = 0;
  params.A[70] = 1;
  params.A[71] = 0;
  // 9th column
  params.A[72] = 0;
  params.A[73] = 0;
  params.A[74] = 0.5 * dt * dt;
  params.A[75] = 0;
  params.A[76] = 0;
  params.A[77] = dt;
  params.A[78] = 0;
  params.A[79] = 0;
  params.A[80] = 1;
  // End of A
  params.x_0[0] = x0[0];
  params.x_0[1] = x0[1];
  params.x_0[2] = x0[2];
  params.x_0[3] = x0[3];
  params.x_0[4] = x0[4];
  params.x_0[5] = x0[5];
  params.x_0[6] = x0[6];
  params.x_0[7] = x0[7];
  params.x_0[8] = x0[8];
  // B
  // 1st column
  params.B[0] = (1 / 6.0) * dt * dt * dt;
  params.B[1] = 0;
  params.B[2] = 0;
  params.B[3] = 0.5 * dt * dt;
  params.B[4] = 0;
  params.B[5] = 0;
  params.B[6] = dt;
  params.B[7] = 0;
  params.B[8] = 0;
  // 2nd column
  params.B[9] = 0;
  params.B[10] = (1 / 6.0) * dt * dt * dt;
  params.B[11] = 0;
  params.B[12] = 0;
  params.B[13] = 0.5 * dt * dt;
  params.B[14] = 0;
  params.B[15] = 0;
  params.B[16] = dt;
  params.B[17] = 0;
  // 3rd column
  params.B[18] = 0;
  params.B[19] = 0;
  params.B[20] = (1 / 6.0) * dt * dt * dt;
  params.B[21] = 0;
  params.B[22] = 0;
  params.B[23] = 0.5 * dt * dt;
  params.B[24] = 0;
  params.B[25] = 0;
  params.B[26] = dt;
  params.v_max[0] = v_max;
  params.a_max[0] = a_max;
  params.j_max[0] = j_max;
}

#undef CONCATENATE_INPUT