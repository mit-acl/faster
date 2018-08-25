/* Produced by CVXGEN, 2018-08-24 19:18:00 -0400.  */
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
#define NUMTESTS 0
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
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
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
  params.xf[0] = 0.20319161029830202;
  params.xf[1] = 0.8325912904724193;
  params.xf[2] = -0.8363810443482227;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q_final[0] = 1.510827605197663;
  params.Q_final[3] = 0;
  params.Q_final[6] = 0;
  params.Q_final[1] = 0;
  params.Q_final[4] = 1.8929469543476547;
  params.Q_final[7] = 0;
  params.Q_final[2] = 0;
  params.Q_final[5] = 0;
  params.Q_final[8] = 1.896293088933438;
  params.A[0] = -1.497658758144655;
  params.A[1] = -1.171028487447253;
  params.A[2] = -1.7941311867966805;
  params.A[3] = -0.23676062539745413;
  params.A[4] = -1.8804951564857322;
  params.A[5] = -0.17266710242115568;
  params.A[6] = 0.596576190459043;
  params.A[7] = -0.8860508694080989;
  params.A[8] = 0.7050196079205251;
  params.x_0[0] = 0.3634512696654033;
  params.x_0[1] = -1.9040724704913385;
  params.x_0[2] = 0.23541635196352795;
  params.B[0] = -0.9629902123701384;
  params.B[1] = -0.3395952119597214;
  params.B[2] = -0.865899672914725;
  params.B[3] = 0.7725516732519853;
  params.B[4] = -0.23818512931704205;
  params.B[5] = -1.372529046100147;
  params.B[6] = 0.17859607212737894;
  params.B[7] = 1.1212590580454682;
  params.B[8] = -0.774545870495281;
  params.v_max[0] = 0.4439157678643628;
}
