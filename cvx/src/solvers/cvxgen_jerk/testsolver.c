/* Produced by CVXGEN, 2018-08-16 18:14:51 -0400.  */
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
  params.xf[6] = -1.497658758144655;
  params.xf[7] = -1.171028487447253;
  params.xf[8] = -1.7941311867966805;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q_final[0] = 1.4408098436506365;
  params.Q_final[9] = 0;
  params.Q_final[18] = 0;
  params.Q_final[27] = 0;
  params.Q_final[36] = 0;
  params.Q_final[45] = 0;
  params.Q_final[54] = 0;
  params.Q_final[63] = 0;
  params.Q_final[72] = 0;
  params.Q_final[1] = 0;
  params.Q_final[10] = 1.0298762108785668;
  params.Q_final[19] = 0;
  params.Q_final[28] = 0;
  params.Q_final[37] = 0;
  params.Q_final[46] = 0;
  params.Q_final[55] = 0;
  params.Q_final[64] = 0;
  params.Q_final[73] = 0;
  params.Q_final[2] = 0;
  params.Q_final[11] = 0;
  params.Q_final[20] = 1.456833224394711;
  params.Q_final[29] = 0;
  params.Q_final[38] = 0;
  params.Q_final[47] = 0;
  params.Q_final[56] = 0;
  params.Q_final[65] = 0;
  params.Q_final[74] = 0;
  params.Q_final[3] = 0;
  params.Q_final[12] = 0;
  params.Q_final[21] = 0;
  params.Q_final[30] = 1.6491440476147607;
  params.Q_final[39] = 0;
  params.Q_final[48] = 0;
  params.Q_final[57] = 0;
  params.Q_final[66] = 0;
  params.Q_final[75] = 0;
  params.Q_final[4] = 0;
  params.Q_final[13] = 0;
  params.Q_final[22] = 0;
  params.Q_final[31] = 0;
  params.Q_final[40] = 1.2784872826479754;
  params.Q_final[49] = 0;
  params.Q_final[58] = 0;
  params.Q_final[67] = 0;
  params.Q_final[76] = 0;
  params.Q_final[5] = 0;
  params.Q_final[14] = 0;
  params.Q_final[23] = 0;
  params.Q_final[32] = 0;
  params.Q_final[41] = 0;
  params.Q_final[50] = 1.6762549019801312;
  params.Q_final[59] = 0;
  params.Q_final[68] = 0;
  params.Q_final[77] = 0;
  params.Q_final[6] = 0;
  params.Q_final[15] = 0;
  params.Q_final[24] = 0;
  params.Q_final[33] = 0;
  params.Q_final[42] = 0;
  params.Q_final[51] = 0;
  params.Q_final[60] = 1.5908628174163508;
  params.Q_final[69] = 0;
  params.Q_final[78] = 0;
  params.Q_final[7] = 0;
  params.Q_final[16] = 0;
  params.Q_final[25] = 0;
  params.Q_final[34] = 0;
  params.Q_final[43] = 0;
  params.Q_final[52] = 0;
  params.Q_final[61] = 0;
  params.Q_final[70] = 1.0239818823771654;
  params.Q_final[79] = 0;
  params.Q_final[8] = 0;
  params.Q_final[17] = 0;
  params.Q_final[26] = 0;
  params.Q_final[35] = 0;
  params.Q_final[44] = 0;
  params.Q_final[53] = 0;
  params.Q_final[62] = 0;
  params.Q_final[71] = 0;
  params.Q_final[80] = 1.5588540879908819;
  params.A[0] = -0.9629902123701384;
  params.A[1] = -0.3395952119597214;
  params.A[2] = -0.865899672914725;
  params.A[3] = 0.7725516732519853;
  params.A[4] = -0.23818512931704205;
  params.A[5] = -1.372529046100147;
  params.A[6] = 0.17859607212737894;
  params.A[7] = 1.1212590580454682;
  params.A[8] = -0.774545870495281;
  params.A[9] = -1.1121684642712744;
  params.A[10] = -0.44811496977740495;
  params.A[11] = 1.7455345994417217;
  params.A[12] = 1.9039816898917352;
  params.A[13] = 0.6895347036512547;
  params.A[14] = 1.6113364341535923;
  params.A[15] = 1.383003485172717;
  params.A[16] = -0.48802383468444344;
  params.A[17] = -1.631131964513103;
  params.A[18] = 0.6136436100941447;
  params.A[19] = 0.2313630495538037;
  params.A[20] = -0.5537409477496875;
  params.A[21] = -1.0997819806406723;
  params.A[22] = -0.3739203344950055;
  params.A[23] = -0.12423900520332376;
  params.A[24] = -0.923057686995755;
  params.A[25] = -0.8328289030982696;
  params.A[26] = -0.16925440270808823;
  params.A[27] = 1.442135651787706;
  params.A[28] = 0.34501161787128565;
  params.A[29] = -0.8660485502711608;
  params.A[30] = -0.8880899735055947;
  params.A[31] = -0.1815116979122129;
  params.A[32] = -1.17835862158005;
  params.A[33] = -1.1944851558277074;
  params.A[34] = 0.05614023926976763;
  params.A[35] = -1.6510825248767813;
  params.A[36] = -0.06565787059365391;
  params.A[37] = -0.5512951504486665;
  params.A[38] = 0.8307464872626844;
  params.A[39] = 0.9869848924080182;
  params.A[40] = 0.7643716874230573;
  params.A[41] = 0.7567216550196565;
  params.A[42] = -0.5055995034042868;
  params.A[43] = 0.6725392189410702;
  params.A[44] = -0.6406053441727284;
  params.A[45] = 0.29117547947550015;
  params.A[46] = -0.6967713677405021;
  params.A[47] = -0.21941980294587182;
  params.A[48] = -1.753884276680243;
  params.A[49] = -1.0292983112626475;
  params.A[50] = 1.8864104246942706;
  params.A[51] = -1.077663182579704;
  params.A[52] = 0.7659100437893209;
  params.A[53] = 0.6019074328549583;
  params.A[54] = 0.8957565577499285;
  params.A[55] = -0.09964555746227477;
  params.A[56] = 0.38665509840745127;
  params.A[57] = -1.7321223042686946;
  params.A[58] = -1.7097514487110663;
  params.A[59] = -1.2040958948116867;
  params.A[60] = -1.3925560119658358;
  params.A[61] = -1.5995826216742213;
  params.A[62] = -1.4828245415645833;
  params.A[63] = 0.21311092723061398;
  params.A[64] = -1.248740700304487;
  params.A[65] = 1.808404972124833;
  params.A[66] = 0.7264471152297065;
  params.A[67] = 0.16407869343908477;
  params.A[68] = 0.8287224032315907;
  params.A[69] = -0.9444533161899464;
  params.A[70] = 1.7069027370149112;
  params.A[71] = 1.3567722311998827;
  params.A[72] = 0.9052779937121489;
  params.A[73] = -0.07904017565835986;
  params.A[74] = 1.3684127435065871;
  params.A[75] = 0.979009293697437;
  params.A[76] = 0.6413036255984501;
  params.A[77] = 1.6559010680237511;
  params.A[78] = 0.5346622551502991;
  params.A[79] = -0.5362376605895625;
  params.A[80] = 0.2113782926017822;
  params.x_0[0] = -1.2144776931994525;
  params.x_0[1] = -1.2317108144255875;
  params.x_0[2] = 0.9026784957312834;
  params.x_0[3] = 1.1397468137245244;
  params.x_0[4] = 1.8883934547350631;
  params.x_0[5] = 1.4038856681660068;
  params.x_0[6] = 0.17437730638329096;
  params.x_0[7] = -1.6408365219077408;
  params.x_0[8] = -0.04450702153554875;
  params.B[0] = 1.7117453902485025;
  params.B[1] = 1.1504727980139053;
  params.B[2] = -0.05962309578364744;
  params.B[3] = -0.1788825540764547;
  params.B[4] = -1.1280569263625857;
  params.B[5] = -1.2911464767927057;
  params.B[6] = -1.7055053231225696;
  params.B[7] = 1.56957275034837;
  params.B[8] = 0.5607064675962357;
  params.B[9] = -1.4266707301147146;
  params.B[10] = -0.3434923211351708;
  params.B[11] = -1.8035643024085055;
  params.B[12] = -1.1625066019105454;
  params.B[13] = 0.9228324965161532;
  params.B[14] = 0.6044910817663975;
  params.B[15] = -0.0840868104920891;
  params.B[16] = -0.900877978017443;
  params.B[17] = 0.608892500264739;
  params.B[18] = 1.8257980452695217;
  params.B[19] = -0.25791777529922877;
  params.B[20] = -1.7194699796493191;
  params.B[21] = -1.7690740487081298;
  params.B[22] = -1.6685159248097703;
  params.B[23] = 1.8388287490128845;
  params.B[24] = 0.16304334474597537;
  params.B[25] = 1.3498497306788897;
  params.B[26] = -1.3198658230514613;
  params.j_max[0] = 0.5206901454578303;
  params.v_max[0] = 1.3839550237456855;
  params.a_max[0] = 1.7911406562839671;
}
