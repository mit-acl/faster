/*
 * maxdet, version alpha
 * C source for solving MAXDET problems
 *
 * Shao-Po Wu, Lieven Vandenberghe and Stephen Boyd
 * Feb. 23, 1996, last major update
 */


/*
 * macros
 */
#define SQR(x)    ((x)*(x))
#define MAX(x,y)  ((x) > (y) ? (x) : (y))
#define MIN(x,y)  ((x) < (y) ? (x) : (y))


/*
 * constant parameters
 */
#define NB        32        /* block size for dgels, must be at least 1 */
#define MINABSTOL 1e-10     /* min absolute tolerance allowed */
#define MAXITERS  100       /* default max number of iterations */
#define CENTOL    1e-3      /* tolerance on Newton decrement of centering */
#define LSTOL     1e-3      /* tolerance used in line search */
#define TOLC      1e-5      /* tolerance used for dual infeasibility */
#define SIGTOL    1e-5      /* tolerance used for detecting zero steps 
                             * dF or dZ */ 
#define MINRCOND  1e-10     /* minimum rcond to declare F_i dependent */
#define LSITERUB  30        /* maximum number of line-search iterations */

#define YES       1
#define NO        0

#define VERSION   "alpha (Apr. 1996)"


#ifdef nounderscores
#define daxpy_ daxpy
#define dcopy_ dcopy
#define ddot_ ddot
#define dgemv_ dgemv
#define dgels_ dgels
#define dlascl_ dlascl
#define dnrm2_ dnrm2
#define dpptrf_ dpptrf
#define dscal_ dscal
#define dspgst_ dspgst
#define dspev_ dspev
#define dspgv_ dspgv
#define dtptri_ dtptri
#define dtrcon_ dtrcon
#endif


/* BLAS 1 */
double dnrm2_( );
double ddot_( );
void dcopy_( );
void daxpy_( );
void dscal_( );

/* BLAS 2 */
void dgemv_( );

/* BLAS 3 */

/* LAPACK */
void dgels_( );
void dlascl_( );
void dpptrf_( );
void dspgst_( );
void dspev_( );
void dspgv_( );
void dtptri_( );
void dtrcon_( );

