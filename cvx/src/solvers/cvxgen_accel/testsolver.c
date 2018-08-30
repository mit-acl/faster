/* Produced by CVXGEN, 2018-08-30 09:34:39 -0400.  */
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
  params.xf[3] = 0.04331042079065206;
  params.xf[4] = 1.5717878173906188;
  params.xf[5] = 1.5851723557337523;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q_final[0] = 1.1255853104638363;
  params.Q_final[6] = 0;
  params.Q_final[12] = 0;
  params.Q_final[18] = 0;
  params.Q_final[24] = 0;
  params.Q_final[30] = 0;
  params.Q_final[1] = 0;
  params.Q_final[7] = 1.2072428781381868;
  params.Q_final[13] = 0;
  params.Q_final[19] = 0;
  params.Q_final[25] = 0;
  params.Q_final[31] = 0;
  params.Q_final[2] = 0;
  params.Q_final[8] = 0;
  params.Q_final[14] = 1.0514672033008299;
  params.Q_final[20] = 0;
  params.Q_final[26] = 0;
  params.Q_final[32] = 0;
  params.Q_final[3] = 0;
  params.Q_final[9] = 0;
  params.Q_final[15] = 0;
  params.Q_final[21] = 1.4408098436506365;
  params.Q_final[27] = 0;
  params.Q_final[33] = 0;
  params.Q_final[4] = 0;
  params.Q_final[10] = 0;
  params.Q_final[16] = 0;
  params.Q_final[22] = 0;
  params.Q_final[28] = 1.0298762108785668;
  params.Q_final[34] = 0;
  params.Q_final[5] = 0;
  params.Q_final[11] = 0;
  params.Q_final[17] = 0;
  params.Q_final[23] = 0;
  params.Q_final[29] = 0;
  params.Q_final[35] = 1.456833224394711;
  params.A[0] = 0.596576190459043;
  params.A[1] = -0.8860508694080989;
  params.A[2] = 0.7050196079205251;
  params.A[3] = 0.3634512696654033;
  params.A[4] = -1.9040724704913385;
  params.A[5] = 0.23541635196352795;
  params.A[6] = -0.9629902123701384;
  params.A[7] = -0.3395952119597214;
  params.A[8] = -0.865899672914725;
  params.A[9] = 0.7725516732519853;
  params.A[10] = -0.23818512931704205;
  params.A[11] = -1.372529046100147;
  params.A[12] = 0.17859607212737894;
  params.A[13] = 1.1212590580454682;
  params.A[14] = -0.774545870495281;
  params.A[15] = -1.1121684642712744;
  params.A[16] = -0.44811496977740495;
  params.A[17] = 1.7455345994417217;
  params.A[18] = 1.9039816898917352;
  params.A[19] = 0.6895347036512547;
  params.A[20] = 1.6113364341535923;
  params.A[21] = 1.383003485172717;
  params.A[22] = -0.48802383468444344;
  params.A[23] = -1.631131964513103;
  params.A[24] = 0.6136436100941447;
  params.A[25] = 0.2313630495538037;
  params.A[26] = -0.5537409477496875;
  params.A[27] = -1.0997819806406723;
  params.A[28] = -0.3739203344950055;
  params.A[29] = -0.12423900520332376;
  params.A[30] = -0.923057686995755;
  params.A[31] = -0.8328289030982696;
  params.A[32] = -0.16925440270808823;
  params.A[33] = 1.442135651787706;
  params.A[34] = 0.34501161787128565;
  params.A[35] = -0.8660485502711608;
  params.x_0[0] = -0.8880899735055947;
  params.x_0[1] = -0.1815116979122129;
  params.x_0[2] = -1.17835862158005;
  params.x_0[3] = -1.1944851558277074;
  params.x_0[4] = 0.05614023926976763;
  params.x_0[5] = -1.6510825248767813;
  params.B[0] = -0.06565787059365391;
  params.B[1] = -0.5512951504486665;
  params.B[2] = 0.8307464872626844;
  params.B[3] = 0.9869848924080182;
  params.B[4] = 0.7643716874230573;
  params.B[5] = 0.7567216550196565;
  params.B[6] = -0.5055995034042868;
  params.B[7] = 0.6725392189410702;
  params.B[8] = -0.6406053441727284;
  params.B[9] = 0.29117547947550015;
  params.B[10] = -0.6967713677405021;
  params.B[11] = -0.21941980294587182;
  params.B[12] = -1.753884276680243;
  params.B[13] = -1.0292983112626475;
  params.B[14] = 1.8864104246942706;
  params.B[15] = -1.077663182579704;
  params.B[16] = 0.7659100437893209;
  params.B[17] = 0.6019074328549583;
  params.a_max[0] = 1.4478782788749642;
  params.v_max[0] = 0.9501772212688626;
}
