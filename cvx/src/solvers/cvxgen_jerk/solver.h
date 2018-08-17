/* Produced by CVXGEN, 2018-08-16 18:14:51 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

#define CONCATENATE_INPUT(x) jerk_##x

#define optimize CONCATENATE_INPUT(optimize)
#define get_state CONCATENATE_INPUT(get_state)
#define get_control CONCATENATE_INPUT(get_control)
#define load_default_data CONCATENATE_INPUT(load_default_data)
#define initialize_optimizer CONCATENATE_INPUT(initialize_optimizer)

#ifdef __cplusplus
namespace Jerk
{
extern "C" {
#endif
/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_JERK_H
#define SOLVER_JERK_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t
{
  double xf[9];
  double Q_final[81];
  double A[81];
  double x_0[9];
  double B[27];
  double j_max[1];
  double v_max[1];
  double a_max[1];
  double *x[1];
} Params;
typedef struct Vars_t
{
  double *u_1;  /* 3 rows. */
  double *u_2;  /* 3 rows. */
  double *u_3;  /* 3 rows. */
  double *u_4;  /* 3 rows. */
  double *u_5;  /* 3 rows. */
  double *u_6;  /* 3 rows. */
  double *u_7;  /* 3 rows. */
  double *u_8;  /* 3 rows. */
  double *u_9;  /* 3 rows. */
  double *x_10; /* 9 rows. */
  double *t_01; /* 1 rows. */
  double *u_0;  /* 3 rows. */
  double *t_02; /* 1 rows. */
  double *t_03; /* 1 rows. */
  double *t_04; /* 1 rows. */
  double *t_05; /* 1 rows. */
  double *t_06; /* 1 rows. */
  double *t_07; /* 1 rows. */
  double *t_08; /* 1 rows. */
  double *t_09; /* 1 rows. */
  double *t_10; /* 1 rows. */
  double *t_11; /* 1 rows. */
  double *u_10; /* 3 rows. */
  double *t_12; /* 1 rows. */
  double *t_13; /* 1 rows. */
  double *x_1;  /* 9 rows. */
  double *t_14; /* 1 rows. */
  double *t_15; /* 1 rows. */
  double *t_16; /* 1 rows. */
  double *t_17; /* 1 rows. */
  double *x_2;  /* 9 rows. */
  double *t_18; /* 1 rows. */
  double *t_19; /* 1 rows. */
  double *t_20; /* 1 rows. */
  double *t_21; /* 1 rows. */
  double *x_3;  /* 9 rows. */
  double *t_22; /* 1 rows. */
  double *t_23; /* 1 rows. */
  double *t_24; /* 1 rows. */
  double *t_25; /* 1 rows. */
  double *x_4;  /* 9 rows. */
  double *t_26; /* 1 rows. */
  double *t_27; /* 1 rows. */
  double *t_28; /* 1 rows. */
  double *t_29; /* 1 rows. */
  double *x_5;  /* 9 rows. */
  double *t_30; /* 1 rows. */
  double *t_31; /* 1 rows. */
  double *t_32; /* 1 rows. */
  double *t_33; /* 1 rows. */
  double *x_6;  /* 9 rows. */
  double *t_34; /* 1 rows. */
  double *t_35; /* 1 rows. */
  double *t_36; /* 1 rows. */
  double *t_37; /* 1 rows. */
  double *x_7;  /* 9 rows. */
  double *t_38; /* 1 rows. */
  double *t_39; /* 1 rows. */
  double *t_40; /* 1 rows. */
  double *t_41; /* 1 rows. */
  double *x_8;  /* 9 rows. */
  double *t_42; /* 1 rows. */
  double *t_43; /* 1 rows. */
  double *t_44; /* 1 rows. */
  double *t_45; /* 1 rows. */
  double *x_9;  /* 9 rows. */
  double *t_46; /* 1 rows. */
  double *t_47; /* 1 rows. */
  double *t_48; /* 1 rows. */
  double *t_49; /* 1 rows. */
  double *t_50; /* 1 rows. */
  double *t_51; /* 1 rows. */
  double *t_52; /* 1 rows. */
  double *t_53; /* 1 rows. */
  double *t_54; /* 1 rows. */
  double *t_55; /* 1 rows. */
  double *t_56; /* 1 rows. */
  double *t_57; /* 1 rows. */
  double *t_58; /* 1 rows. */
  double *t_59; /* 1 rows. */
  double *t_60; /* 1 rows. */
  double *t_61; /* 1 rows. */
  double *t_62; /* 1 rows. */
  double *t_63; /* 1 rows. */
  double *t_64; /* 1 rows. */
  double *t_65; /* 1 rows. */
  double *t_66; /* 1 rows. */
  double *t_67; /* 1 rows. */
  double *t_68; /* 1 rows. */
  double *t_69; /* 1 rows. */
  double *t_70; /* 1 rows. */
  double *t_71; /* 1 rows. */
  double *t_72; /* 1 rows. */
  double *t_73; /* 1 rows. */
  double *t_74; /* 1 rows. */
  double *t_75; /* 1 rows. */
  double *t_76; /* 1 rows. */
  double *t_77; /* 1 rows. */
  double *t_78; /* 1 rows. */
  double *t_79; /* 1 rows. */
  double *t_80; /* 1 rows. */
  double *t_81; /* 1 rows. */
  double *t_82; /* 1 rows. */
  double *t_83; /* 1 rows. */
  double *t_84; /* 1 rows. */
  double *t_85; /* 1 rows. */
  double *t_86; /* 1 rows. */
  double *t_87; /* 1 rows. */
  double *t_88; /* 1 rows. */
  double *t_89; /* 1 rows. */
  double *t_90; /* 1 rows. */
  double *t_91; /* 1 rows. */
  double *u[11];
  double *x[11];
} Vars;
typedef struct Workspace_t
{
  double h[277];
  double s_inv[277];
  double s_inv_z[277];
  double b[90];
  double q[214];
  double rhs[858];
  double x[858];
  double *s;
  double *z;
  double *y;
  double lhs_aff[858];
  double lhs_cc[858];
  double buffer[858];
  double buffer2[858];
  double KKT[2515];
  double L[3337];
  double d[858];
  double v[858];
  double d_inv[858];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_416097357824[1];
  int converged;
} Workspace;
typedef struct Settings_t
{
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_params(void);
void setup_indexed_optvars(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
int initialize_optimizer(void);
int optimize(void);
void load_default_data(double dt, double v_max, double a_max, double j_max, double x0[], double xf[]);
double **get_state(void);
double **get_control(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long *idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#endif
#ifdef __cplusplus
}
}
#endif
#undef optimize
#undef get_state
#undef get_control
#undef load_default_data
#undef initialize_optimizer
