/*
 * maxdet, version alpha
 * C source for MAXDET-programs
 *
 * Shao-Po Wu, Lieven Vandenberghe and Stephen Boyd 
 * Apr. 27, 1996, last major update
*/
#ifndef MAXDET_VERBOSE 
#define MAXDET_VERBOSE 0
#endif

extern "C"
{

int maxdet(
 int m,                /* no of variables */
 int L,                /* no of blocks in F */
 double *F,            /* F_i's in packed storage */
 int *F_blkszs,        /* L-vector, dimensions of diagonal blocks of F */
 int K,                /* no of blocks in G */
 double *G,            /* G_i's in packed storage */
 int *G_blkszs,        /* K-vector, dimensions of diagonal blocks of G */
 double *c,            /* m-vector */
 double *x,            /* m-vector */
 double *Z,            /* block diagonal matrix in packed storage */
 double *W,            /* block diagonal matrix in packed storage */
 double *ul,           /* ul[0] = pr. obj, ul[1] = du. obj */
 double *hist,         /* history, 3-by-NTiter matrix */
 double gamma,         /* > 0 */
 double abstol,        /* absolute accuracy */
 double reltol,        /* relative accuracy */
 int *NTiters,         /* on entry: maximum number of (total) Newton iters,
                        * on exit: number of Newton iterations taken */
 double *work,         /* (double) work array */
 int lwork,            /* size of work */
 int *iwork,           /* (int) work array */
 int *info             /* status on termination */
);

}
