/* Produced by CVXGEN, 2018-03-30 10:48:24 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 1000
int main(int argc, char **argv) {
    int num_iters;
  #if (NUMTESTS > 0)
    int i;
    double time;
    double time_per;
  #endif
    set_defaults();
    setup_indexing();
    load_default_data();
    /* Solve problem instance for the record. */
    settings.verbose = 0;
    num_iters = solve();
  #ifndef ZERO_LIBRARY_MODE
  #if (NUMTESTS > 0)
    /* Now solve multiple problem instances for timing purposes. */
    settings.verbose = 0;
    tic();
    for (i = 0; i < NUMTESTS; i++) {
      solve();
    }

    if (work.converged==1){
      printf("final pos: %0.2f %0.2f %0.2f \n",vars.x[19][0],vars.x[19][1],vars.x[19][2]);
      printf("final vel: %0.2f %0.2f %0.2f \n",vars.x[19][3],vars.x[19][4],vars.x[19][5]);
      // for(i=1;i<20;i++){
      //   printf("%0.2f %0.2f %0.2f \n",vars.x[i][2],vars.x[i][5],vars.u[i][2]);
      // }
    }

    time = tocq();
    printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
    time_per = time / NUMTESTS;
    if (time_per > 1) {
      printf("Actual time taken per solve: %.3g s.\n", time_per);
    } else if (time_per > 1e-3) {
      printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
    } else {
      printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
    }
  #endif
  #endif
    return 0;
}
void load_default_data(void) {
  
  double dt = 0.5;
  double q = 100000;
  params.xf[0] = 1.0;
  params.xf[1] = 0.0;
  params.xf[2] = 0.0;
  params.xf[3] = 0.0;
  params.xf[4] = 0.0;
  params.xf[5] = 0.0;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q_final[0] = q;
  params.Q_final[6] = 0;
  params.Q_final[12] = 0;
  params.Q_final[18] = 0;
  params.Q_final[24] = 0;
  params.Q_final[30] = 0;
  params.Q_final[1] = 0;
  params.Q_final[7] = q;
  params.Q_final[13] = 0;
  params.Q_final[19] = 0;
  params.Q_final[25] = 0;
  params.Q_final[31] = 0;
  params.Q_final[2] = 0;
  params.Q_final[8] = 0;
  params.Q_final[14] = q;
  params.Q_final[20] = 0;
  params.Q_final[26] = 0;
  params.Q_final[32] = 0;
  params.Q_final[3] = 0;
  params.Q_final[9] = 0;
  params.Q_final[15] = 0;
  params.Q_final[21] = q;
  params.Q_final[27] = 0;
  params.Q_final[33] = 0;
  params.Q_final[4] = 0;
  params.Q_final[10] = 0;
  params.Q_final[16] = 0;
  params.Q_final[22] = 0;
  params.Q_final[28] = q;
  params.Q_final[34] = 0;
  params.Q_final[5] = 0;
  params.Q_final[11] = 0;
  params.Q_final[17] = 0;
  params.Q_final[23] = 0;
  params.Q_final[29] = 0;
  params.Q_final[35] = q;
  params.A[0] = 1;
  params.A[1] = 0;
  params.A[2] = 0;
  params.A[3] = 0;
  params.A[4] = 0;
  params.A[5] = 0;
  params.A[6] = 0;
  params.A[7] = 1;
  params.A[8] = 0;
  params.A[9] = 0;
  params.A[10] = 0;
  params.A[11] = 0;
  params.A[12] = 0;
  params.A[13] = 0;
  params.A[14] = 1;
  params.A[15] = 0;
  params.A[16] = 0;
  params.A[17] = 0;
  params.A[18] = dt;
  params.A[19] = 0;
  params.A[20] = 0;
  params.A[21] = 1;
  params.A[22] = 0;
  params.A[23] = 0;
  params.A[24] = 0;
  params.A[25] = dt;
  params.A[26] = 0;
  params.A[27] = 0;
  params.A[28] = 1;
  params.A[29] = 0;
  params.A[30] = 0;
  params.A[31] = 0;
  params.A[32] = dt;
  params.A[33] = 0;
  params.A[34] = 0;
  params.A[35] = 1;
  params.x_0[0] = -5;
  params.x_0[1] =  2;
  params.x_0[2] = -3;
  params.x_0[3] = -1;
  params.x_0[4] =  0;
  params.x_0[5] =  2;
  params.B[0] = 0.5*dt*dt;
  params.B[1] = 0;
  params.B[2] = 0;
  params.B[3] = dt;
  params.B[4] = 0;
  params.B[5] = 0;
  params.B[6] = 0;
  params.B[7] = 0.5*dt*dt;
  params.B[8] = 0;
  params.B[9] = 0;
  params.B[10] = dt;
  params.B[11] = 0;
  params.B[12] = 0;
  params.B[13] = 0;
  params.B[14] = 0.5*dt*dt;
  params.B[15] = 0;
  params.B[16] = 0;
  params.B[17] = dt;
  params.u_max[0] = 5.0;
}
