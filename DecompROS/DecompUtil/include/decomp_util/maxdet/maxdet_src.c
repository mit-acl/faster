/*
 * maxdet, version alpha
 * C source for MAXDET-programs
 *
 * Shao-Po Wu, Lieven Vandenberghe and Stephen Boyd 
 * Apr. 27, 1996, last major update
 */

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <math.h>
#include "maxdet.h"


/* from sdpsol, for debugging */
void disp_imat(fp,ip,r,c,att)    /* print an integer matrix to a stream */
FILE *fp;        /* stream pointer */
int  *ip;        /* matrix (int) pointer */
int  r,c,att;    /* number of rows and columns, matrix attribute */
/* REM: att is not used for now !! */
{
    register int i,j;
    register int *rip;

    for (i=0; i<r; i++) {
        fprintf(fp,"| ");
        for (rip=ip+i, j=0; j<c; rip+=r, j++)
            fprintf(fp,"%8d  ",*rip);
        fprintf(fp,"|\n");
    }
    fprintf(fp,"\n");
}

void disp_mat(fp,dp,r,c,att)    /* print a double matrix to a stream */
FILE   *fp;        /* stream pointer */
double *dp;        /* matrix (double) pointer */
int    r,c,att;    /* number of rows and columns, matrix attribute */
/* REM: att is not used for now !! */
{
    register int    i,j;
    register double *rdp,dtmp,maxmag=0;

    /* test r and c */
    if (!r || !c) {    /* empty mat */
        fprintf(fp,"[]\n\n");
        return;
    }
    /* get the largest (in magnitude) entry */
    for (i=0, rdp=dp; i<r*c; i++, rdp++) {
        dtmp = fabs(*rdp);
        if (dtmp>maxmag)
            maxmag = dtmp;
    }
    /* print */
    if (maxmag>=1e4 || (maxmag<=1e-4 && maxmag>0)) {
        maxmag = pow((double) 10.0,(double) floor(log10(maxmag)));
        fprintf(fp,"%e *\n",maxmag);
        for (i=0; i<r; i++) {
            fprintf(fp,"%s",(i ? "\n " : "["));
            for (rdp=dp+i, j=0; j<c; rdp+=r, j++)
                fprintf(fp,"% 10.4f,",*rdp/maxmag);
            fprintf(fp,"\b;");
        }
        fprintf(fp,"\b ]\n\n");
    } else {
        for (i=0; i<r; i++) {
            fprintf(fp,"%s",(i ? "\n " : "["));
            for (rdp=dp+i, j=0; j<c; rdp+=r, j++)
                fprintf(fp,"% 10.4f,",*rdp);
            fprintf(fp,"\b;");
        }
        fprintf(fp,"\b ]\n\n");
    }
}


/* from SP */

double inprd(X,Z,L,blck_szs)
double *X, *Z;
int    L, *blck_szs;
/*
 * Computes Tr X*Z
 *
 * Arguments:
 * X,Z:       block diagonal matrices with L blocks X^0, ..., X^{L-1},
 *            and Z^0, ..., Z^{L-1}.  X^j and Z^j have size 
 *            blck_szs[j] times blck_szs[j].  Every block is stored 
 *            using packed storage of the lower triangle.
 * L:         number of blocks
 * blck_szs:  integer vector of length L 
 *            blck_szs[i], i=0,...,L-1 is the size of block i
 */
{
    register int i, j, k;
    double result;
    int    lngth, pos, sz, int1=1;
 
    /* sz = length of Z and X */  
    for (i=0, sz=0;  i<L;  i++)  sz += (blck_szs[i]*(blck_szs[i]+1))/2;

    /* result = Tr X Z + contributions of diagonal elements */
    result = 2.0*ddot_(&sz, X, &int1, Z, &int1);

    /* correct for diagonal elements 
     * loop over blocks, j=0,...,L-1  */
    for (j=0, pos=0;  j<L;  j++)

        /* loop over columns, k=0,...,blck_szs[j]-1 
         * pos is position of (k,k) element of block j 
         * lngth is length of column k */
        for (k=0, lngth=blck_szs[j];  k<blck_szs[j];  pos+=lngth, 
             lngth-=1, k++)

            /* subtract Z^j_{kk}*X^j_{kk} from result */
            result -= Z[pos]*X[pos];

    return result;
}


/* maxdet routines */

double eig_val(sig,ap,L,blck_szs,Npd,work)
double *sig, *ap, *work;
int    L, *blck_szs, Npd;
/* 
 * find eigenvalues and the minimum one of a symmetric block-diagonal
 * matrix in packed storage.
 * routine returns the minimum eigenvalue on exit.
 *
 * sig      -- (on exit) vector of eigenvalues
 * ap       -- symmetric block-diagonal matrix in packed storage
 * L        -- number of diagonal blocks
 * blck_szs -- sizes of the diagonal blocks
 * Npd      -- length of the maxtrix in packed storage
 * work     -- double precision workspace, dimension Npd+3*(max block size)
 *             work[0 -- Npd-1] : copy of ap
 *             work[Npd -- ]    : workspace for DSPEV
 *
 * NOTICE: This routine finds the eigenvalues block-by-block,
 *         LAPACK routine DSPEV is NOT used UNLESS the block size exceeds 2.
 */
{
    register int i;
    int    int1=1, nainfo;
    double min_ev=0.0, *dps, *sigptr;

    memcpy(work,ap,Npd*sizeof(double));
    for (i=0, sigptr=sig, dps=work; i<L; i++) {
        switch (blck_szs[i]) {
          case 1:    /* 1-by-1 block */
            *sigptr = *dps;
            min_ev = (i ? MIN(min_ev,*dps) : *dps);
            sigptr++;
            dps++;
            break;
          case 2:    /* 2-by-2 block */
            *sigptr = (dps[0]+dps[2]
                       -sqrt(SQR(dps[0]-dps[2])+4.0*SQR(dps[1])))/2.0;
            *(sigptr+1) = (dps[0]+dps[2]
                           +sqrt(SQR(dps[0]-dps[2])+4.0*SQR(dps[1])))/2.0;
            min_ev = (i ? MIN(min_ev,*sigptr) : *sigptr);
            sigptr += 2;
            dps += 3;
            break;
          default:   /* 3-by-3 or larger block */
            dspev_("N","L",blck_szs+i,dps,sigptr,NULL,&int1,work+Npd,&nainfo);
            if (nainfo) {
                fprintf(stderr,"Error in dspev(0), info = %d.\n", nainfo);
                exit(-1);
            }
            min_ev = (i ? MIN(min_ev,*sigptr) : *sigptr);
            sigptr += blck_szs[i];
            dps += blck_szs[i]*(blck_szs[i]+1)/2;
            break;
        }
    }
    return min_ev;
}




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
)


/*
 * Solves determinant-maximization (MAXDET) program 
 *
 * (P) minimize    c^Tx - \log\det G(x)
 *     subject to  G(x)\geq 0
 *                 F(x)\geq 0
 *
 * and its dual
 *
 * (D) maximize    \log\det W -\Tr ZF_0 -\Tr WG_0 + l
 *     subject to  W\geq 0, Z\geq 0
 *                 \Tr ZF_i + \Tr WG_i = c_i, i=1,...,m
 *
 * Convergence criteria: 
 * (1) maxiters is exceeded
 * (2) duality gap is less than abstol
 * (3) primal and dual objective are both positive and 
 *     duality gap is less than reltol * dual objective;         
 *     or, primal and dual objective are both negative and
 *     duality gap is less than reltol * minus the primal objective
 * 
 * Arguments:
 * - m:        number of variables x_i. m >= 1.
 * - L:        number of diagonal blocks in F_i. L >= 1.
 * - F:        block diagonal matrices F_i, i=0,...,m. 
 *             it is assumed that the matrices diag(F_i,G_i) are linearly 
 *             independent. 
 *             let F_i^j, i=0,..,m, j=0,...,L-1 denote the jth 
 *             diagonal block of F_i, 
 *             the array F contains F_0^0, ..., F_0^{L-1}, F_1^0, ..., 
 *             F_1^{L-1}, ..., F_m^0, ..., F_m^{L-1}, in this order, 
 *             using packed storage for the lower triangular part of 
 *             F_i^j.
 * - F_blkszs: an integer L-vector. F_blkszs[j], j=0,...,L-1 gives the 
 *             size of block j, ie, F_i^j has size F_blkszs[j] 
 *             times F_blkszs[j].
 * - K:        number of diagonal blocks in G_i. K >= 1.
 * - G:        the block diagonal matrices G_i, i=0,...,m. 
 *             it is assumed that the matrices diag(F_i,G_i) are linearly 
 *             independent. 
 * - G_blkszs: an integer K-vector. G_blkszs[j], j=0,...,K-1 gives the
 *             size of block j, ie, G_i^j has size G_blkszs[j]
 * - c:        m-vector, primal objective.
 * - x:        m-vector.  On entry, a strictly primal feasible point. 
 *             On exit, the last iterate for x.
 * - Z:        block diagonal matrix with L blocks Z^0, ..., Z^{L-1}.
 *             Z^j has size F_blkszs[j] times F_blkszs[j].
 *             Every block is stored using packed storage of the lower 
 *             triangular part.
 *             On entry, a strictly dual feasible point is OPTIONAL.
 *             On exit, the last dual iterate.
 * - W:        block diagonal matrix with K blocks W^0, ..., W^{K-1}.
 *             W^j has size G_blkszs[j] times G_blkszs[j].
 *             Every block is stored using packed storage of the lower 
 *             triangular part.
 *             On entry, a strictly dual feasible point is OPTIONAL.
 *             On exit, the last dual iterate.
 * - ul:       two-vector.  On exit, ul[0] is the primal objective value
 *             c^Tx-\logdet G(x);  ul[1] is the dual objective value
 *             \log\det W -\Tr ZF_0 -\Tr WG_0 + l.
 * - hist:     history, 3-by-NTiter matrix.
 *             hist = [objective; duality gap; # of NT iterations]
 * - gamma:    > 0. Controls how aggressively the algorithm works.
 * - abstol:   absolute tolerance, >= MINABSTOL (MINABSTOL in maxdet.h).
 * - reltol:   relative tolerance, >= 0.
 * - NTiters:  On entry: maximum number of Newton (and predictor) iterations,
 *             NTiters >= 0.
 *             On exit: the number of Newton iterations taken.
 * - work:     (double) work array of size lwork.
 * - lwork:    size of work, must be at least:
 *             2*(m+2)*sz + 2*(n+l) + ltemp, with 
 *             ltemp = max( m+sz*NB, 3*(m+m^2+max(G_sz,F_sz)),
 *                          3*(max(max_l,max_n)+max(G_sz,F_sz)) ),
 *             where 
 *             sz: space needed to store one matrix F_i AND one matrix
 *                 G_i in packed storage;
 *             F_sz: space needed to store one matrix F_i packed;
 *             G_sz: space needed to store one matrix G_i packed;
 *             max_n: max block size in F;
 *             max_l: max block size in G;
 *             NB >= 1, for best performance, NB should be at least
 *             equal to the optimal block size for dgels.
 * - iwork     (int) work array of size m
 * - info:     returns 1 if maximum Newton iterations exceeded,
 *             2 if absolute accuracy is reached,  3 if relative accuracy
 *             is reached;
 *             negative values indicate errors: -i means argument i 
 *             has an illegal value; -23 means \diag(G_i,F_i) dependent;
 *             -24 stands for all other errors.
 *
 * Returns 0 for normal exit, 1 if an error occurred.
 *
 */
 

{
    register int i, j, k;
    double *dps, *dpt, dtmp, dtmp2;
    int    itmp, iters, minlwork, lwkspc, maxNTiters;
    int    dual_equ_feas=YES, dual_PD_feas=NO;
    int    n, F_sz, F_upsz, max_n, l, G_sz, G_upsz, max_l, sz, M_sz, M_upsz;
    double t, sqrtt, t_old, y, gap, mu, lambda, alpha[2], beta, psi_ub;
    double grad1, grad2, hess1, hess2;
    double *rhs, *GaF, *ZWtmp, *Fsc, *Gsc, *XF, *XG, *dXF, *dXG, *dW, *dZ;
    double *LF, *LG, *LFinv, *LGinv;
    double *sigF, *sigG, *sigZ, *sigW;
    double *M1, *LM, *VM, *sigM, *v1, *v2;
    double *wkspc, *temp, *temp2, *temp3, *temp4;

    int    int1=1, int2=2, NAinfo;
    double dbl0=0.0, dbl1=1.0, dblm1=-1.0;


    if (m < 1) {
        fprintf(stderr,"maxdet: m must be at least one.\n");
        *info = -1;
        return 1;
    }
    if (L < 1) {
        fprintf(stderr,"maxdet: L must be at least one.\n");
        *info = -2;
        return 1;
    }
    if (K < 1) {
        fprintf(stderr,"maxdet: K must be at least one.\n");
        *info = -5;
        return 1;
    }
    for (i=0; i<L; i++)
        if (F_blkszs[i] < 1) {
            fprintf(stderr,"maxdet: F_blkszs[%d] must be at least one.\n",i);
            *info = -4;
            return 1;
        }
    for (i=0; i<K; i++)
        if (G_blkszs[i] < 1) {
            fprintf(stderr,"maxdet: G_blkszs[%d] must be at least one.\n",i);
            *info = -7;
            return 1;
        }
    if (gamma <= 0) {
        fprintf(stderr,"maxdet: gamma must be greater than zero.\n");
        *info = -14;
        return 1;
    }
    if (reltol < 0) {
        fprintf(stderr,"maxdet: reltol must be non-negative.\n");
        *info = -16;
        return 1;
    }
    if (abstol < MINABSTOL)
        fprintf(stderr,"maxdet: (warning) abstol less than MINABSTOL=%10.2e\n\
        MINABSTOL is used instead.\n",MINABSTOL);



    /*
     * calculate dimensions:
     * n:      (total) size of F(x)
     * F_sz:   length of one block-diagonal matrix in packed storage
     * F_upsz: length of one block-diagonal matrix in unpacked storage
     * max_n:  size of biggest block in F
     * l:      (total) size of G(x)
     * G_sz:   length of one block-diagonal matrix in packed storage
     * G_upsz: length of one block-diagonal matrix in unpacked storage
     * max_l:  size of biggest block in G
     */
    for (i=0, n=0, F_sz=0, F_upsz=0, max_n=0;  i<L;  i++) {
        n += F_blkszs[i];
        F_sz += F_blkszs[i]*(F_blkszs[i]+1)/2;
        F_upsz += F_blkszs[i]*F_blkszs[i];
        max_n  = MAX(max_n, F_blkszs[i]);
    } 
    for (i=0, l=0, G_sz=0, G_upsz=0, max_l=0;  i<K;  i++) {
        l += G_blkszs[i];
        G_sz += G_blkszs[i]*(G_blkszs[i]+1)/2;
        G_upsz += G_blkszs[i]*G_blkszs[i];
        max_l  = MAX(max_l, G_blkszs[i]);
    }
    sz = F_sz+G_sz;
    M_sz = m*(m+1)/2;
    M_upsz = SQR(m);
    if (m > sz) {
        fprintf(stderr, "maxdet: matrices diag(Fi,Gi), i=1,...,m are linearly\
 dependent.\n");
        *info = -23;
        return 1;
    }


    /*
     * organize workspace
     *
     * work:  2*(m+2)*sz + 2*(n+l) + ltemp
     * minimum ltemp: the maximum of the following
     *   m+sz*NB, 3*(MAX(max_l,max_n)+MAX(G_sz,F_sz)),
     *   MAX(G_sz+3*max_l,F_sz+3*max_n), 3*(m+m^2+MAX(G_sz,F_sz))
     * 
     * for dgels:        m + sz*NB,
     * for dspev:        3*MAX(max_n,max_l)
     * for dspgv:        3*MAX(max_n,max_l)
     * for dtrcon:       (double) 3*m;  (int) m
     * for eig_val:      MAX(G_sz+3*max_l,F_sz+3*max_n)
     * 
     * rhs  (sz):        work
     * GaF  (m*sz):      work+sz
     * ZWtmp(sz):        work+(m+1)*sz
     * Fsc  (m*F_sz):    work+(m+2)*sz
     * Gsc  (m*G_sz):    work+(m+2)*sz+m*F_sz
     * LF   (F_sz):      work+(2*m+2)*sz
     * LG   (G_sz):      work+(2*m+2)*sz+F_sz
     * XF   (F_sz):      work+(2*m+3)*sz
     * XG   (G_sz):      work+(2*m+3)*sz+F_sz
     * dXF  (F_sz):      work+(2*m+4)*sz
     * dXG  (G_sz):      work+(2*m+4)*sz+F_sz
     * sigF (n):         work+(2*m+5)*sz
     * sigG (l):         work+(2*m+5)*sz+n
     * sigZ (n):         work+(2*m+5)*sz+(n+l)
     * sigW (l):         work+(2*m+5)*sz+(2*n+l)
     * wkspc(ltemp):     work+(2*m+5)*sz+2*(n+l)
     *
     * M1   (M_upsz):    wkspc+3*m
     * LM   (M_upsz):    wkspc+3*m+m^2
     * VM   (M_upsz):    wkspc+3*m+2*m^2
     * sigM (m):         work+(2*m+3)*sz
     * v1   (m):         work+(2*m+4)*sz
     * v2   (m):         work+(2*m+4)*sz+(n+l)
     * LFinv(F_sz):      temp
     * LGinv(G_sz):      temp
     * dZ   (F_sz):      work+sz+G_sz
     * dW   (G_sz):      work+sz
     *
     * temp  (MAX(F_sz,G_sz)):  wkspc+(lwkspc-3*MAX(G_sz,F_sz))
     * temp2 (MAX(F_sz,G_sz)):  temp+MAX(G_sz,F_sz)
     * temp3 (MAX(F_sz,G_sz)):  temp+2*MAX(G_sz,F_sz)
     * temp4 (sz):              work+(2*m+1)*sz
     */

    /* check lwork */
    minlwork = (2*m+5)*sz + 2*(n+l) + 
        MAX(m+sz*NB,MAX(3*(m+M_upsz+MAX(G_sz,F_sz)),
                        MAX(3*(MAX(max_l,max_n)+MAX(G_sz,F_sz)),
                            MAX(G_sz+3*max_l,F_sz+3*max_n))));
    if (lwork < minlwork){
        fprintf(stderr, "maxdet: not enough workspace, need at least\
 %d*sizeof(double).\n", minlwork);
        *info = -20;
        return 1;
    } 
    lwkspc = lwork - (2*m+5)*sz - 2*(n+l);
    //fprintf(stdout,"workspace %d bytes.\n",8*lwork+4*m);

    rhs   = work;          /* rhs for LS problem, also for dx */
    GaF   = rhs + sz;      /* lhs for LS problem */
    ZWtmp = GaF + sz;      /* temporary storage of Z and W */
    Fsc   = ZWtmp + m*sz;  /* scaled F_i's */
    Gsc   = Fsc + m*F_sz;  /* scaled G_i's */
    LF    = Gsc + m*G_sz;  /* cholesky of F */
    LG    = LF + F_sz;     /* cholesky of G */
    XF    = LG + G_sz;     /* F */
    XG    = XF + F_sz;     /* G */
    dXF   = XG + G_sz;     /* dF */
    dXG   = dXF + F_sz;    /* dG */
    sigF  = dXG + G_sz;    /* eig(dXF,XF) */
    sigG  = sigF + n;      /* eig(dXG,XG) */
    sigZ  = sigG + l;      /* eig(dZ,Z) */
    sigW  = sigZ + n;      /* eig(dW,W) */

    wkspc = work + (2*m+5)*sz + 2*(n+l);        /* workspace for LAPACK */
    temp  = wkspc + (lwkspc-3*MAX(G_sz,F_sz));  /* size MAX(G_sz,F_sz) */
    temp2 = temp + MAX(G_sz,F_sz);              /* size MAX(G_sz,F_sz) */
    temp3 = temp2 + MAX(G_sz,F_sz);             /* size MAX(G_sz,F_sz) */
    temp4 = LF;                                 /* size sz */

    LFinv = temp;          /* LF^{-1} */
    LGinv = temp;          /* LG^{-1} */

    M1 = wkspc + 3*m;      /* used in preliminary phase */
    LM = M1 + M_upsz;      /* chol(M1+M2) */
    VM = LM + M_upsz;      /* eigen vectors of eig(M1,M1+M2) */
    sigM = dXF;            /* eig(M1,M1+M2), used in preliminary phase */
    v1 = sigF;             /* used in preliminary phase */
    v2 = sigZ;             /* used in preliminary phase */

    dW = GaF;              /* dW */
    dZ = dW + G_sz;        /* dZ */

    /* misc parameters and constants */
    maxNTiters = (*NTiters >= 1) ? *NTiters : MAXITERS;
    *NTiters = 0;          /* reset *NTiters */
    *info = -24;           /* default: all other errors */
    t = 1;
#if MAXDET_VERBOSE
    fprintf(stdout," iters        obj            gap\n"); 
#endif
    /*
     * (outer) iterations begin here
     */
    for (iters=1; ; iters++) {


        int    pos, pos2, NTcount;
        double mat_norm, rcond;

        /* compute F(x) = F_0 + x_1*F_1 + ... + x_m*F_m, store in XF
         * also    G(x) = G_0 + x_1*G_1 + ... + x_m*G_m, store in XG */
        dcopy_(&F_sz,F,&int1,XF,&int1);
        dgemv_("N",&F_sz,&m,&dbl1,F+F_sz,&F_sz,x,&int1,&dbl1,XF,&int1);
        dcopy_(&G_sz,G,&int1,XG,&int1);
        dgemv_("N",&G_sz,&m,&dbl1,G+G_sz,&G_sz,x,&int1,&dbl1,XG,&int1);

        if (iters==1) {

            /* preliminary phase (cf. maxdet paper):
             *   1. check dual equalities
             *   2. if (1) is satisfied, check dual positive-definiteness
             *   3. if (2) is satisfied, compute initial t using the dual
             *   4. otherwise use the heuristic preliminary centering
             *      (minimizing newton decrement) to obtain initial t */

            /* check if \Tr ZF_i + \Tr WG_i = c_i, i=1,...,m */
            mat_norm = sqrt(inprd(Z,Z,L,F_blkszs)+inprd(W,W,K,G_blkszs));
            i = 0;
            while (dual_equ_feas==YES && i<m) {
                if (fabs(inprd(F+(i+1)*F_sz,Z,L,F_blkszs)+
                         inprd(G+(i+1)*G_sz,W,K,G_blkszs)-c[i])
                    > mat_norm*TOLC)
                    dual_equ_feas = NO;
                i++;
            }
            /* check dual positive-definiteness */
            if (dual_equ_feas && eig_val(sigZ,Z,L,F_blkszs,F_sz,wkspc)>0 &&
                eig_val(sigW,W,K,G_blkszs,G_sz,wkspc)>0)
                dual_PD_feas = YES;
        }

        /* Newton's method begins here */
        sqrtt = sqrt(t);
        for (NTcount=0; *NTiters<maxNTiters; ) {
            (*NTiters)++;
            NTcount++;
            /* cholesky decompositions of F and G are used for scaling
             *   XF = LF*LF^T;  XG = LG*LG^T
             * and the scaled F_i's and G_i's
             *   Gsc = LG^{-1}*G_i*LG^{-T}
             *   Fsc = LF^{-1}*F_i*LF^{-T} */
            memcpy(Gsc,G+G_sz,m*G_sz*sizeof(double));  /* copy G to Gsc */
            memcpy(Fsc,F+F_sz,m*F_sz*sizeof(double));  /* copy F to Fsc */
            memcpy(LG,XG,G_sz*sizeof(double));         /* copy XG to LG */
            memcpy(LF,XF,F_sz*sizeof(double));         /* copy XF to LF */
            for (i=0, pos=0, dpt=LG; i<K;
                 pos+=G_blkszs[i]*(G_blkszs[i]+1)/2, dpt=LG+pos, i++) {
                /* loop over blocks, do the cholesky decomp and form Gsc.
                 * DPPTRF is called only if the block size exceeds 2 */
                switch (G_blkszs[i]) {
                  case 1:    /* 1-by-1 block */
                    if (*dpt<=0) {
                        fprintf(stderr,"maxdet: x infeasible because G(x)<=0.\n");
                        *info = -9;
                        return 1;
                    }
                    *dpt = sqrt(*dpt);
                    break;
                  case 2:    /* 2-by-2 block */
                    if (*dpt + *(dpt+2)<=0 ||
                        *dpt * *(dpt+2) - SQR(*(dpt+1))<=0) {
                        fprintf(stderr,"maxdet: x infeasible because G(x)<=0.\n");
                        *info = -9;
                        return 1;
                    }
                    *dpt = sqrt(*dpt);
                    *(dpt+1) = *(dpt+1) / *dpt;
                    *(dpt+2) = sqrt(*(dpt+2)-SQR(*(dpt+1)));
                    break;
                  default:   /* 3-by-3 or larger block */
                    dpptrf_("L",G_blkszs+i,dpt,&NAinfo);
                    if (NAinfo>0) {
                        fprintf(stderr,"maxdet: x infeasible because G(x)<=0.\n");
                        *info = -9;
                        return 1;
                    } else if (NAinfo) {
                        fprintf(stderr,"Error in dpptrf(0), info = %d.\n",NAinfo);
                        return 1;
                    }
                    break;
                }
                /* ith blocks of scaled matrices Gsc_j, j=1,...,m */
                for (j=0; j<m; j++) {
                    dspgst_(&int1,"L",G_blkszs+i,Gsc+j*G_sz+pos,dpt,&NAinfo);
                    if (NAinfo){ 
                        fprintf(stderr,"Error in dspgst(0), info = %d.\n",NAinfo);
                        return 1;
                    }
                }
            }
            for (i=0, pos=0, dpt=LF; i<L;
                 pos+=F_blkszs[i]*(F_blkszs[i]+1)/2, dpt=LF+pos, i++) {
                /* loop over blocks, do the cholesky decomp and form Fsc.
                 * DPPTRF is called only if the block size exceeds 2 */
                switch (F_blkszs[i]) {
                  case 1:    /* 1-by-1 block */
                    if (*dpt<=0) {
                        fprintf(stderr,"maxdet: x infeasible because F(x)<=0.\n");
                        *info = -9;
                        return 1;
                    }
                    *dpt = sqrt(*dpt);
                    break;
                  case 2:    /* 2-by-2 block */
                    if (*dpt + *(dpt+2)<=0 ||
                        *dpt * *(dpt+2) - SQR(*(dpt+1))<=0) {
                        fprintf(stderr,"maxdet: x infeasible because F(x)<=0.\n");
                        *info = -9;
                        return 1;
                    }
                    *dpt = sqrt(*dpt);
                    *(dpt+1) = *(dpt+1) / *dpt;
                    *(dpt+2) = sqrt(*(dpt+2)-SQR(*(dpt+1)));
                    break;
                  default:   /* 3-by-3 or larger block */
                    dpptrf_("L",F_blkszs+i,dpt,&NAinfo);
                    if (NAinfo>0) {
                        fprintf(stderr,"maxdet: x infeasible because F(x)<=0.\n");
                        *info = -9;
                        return 1;
                    } else if (NAinfo) {
                        fprintf(stderr,"Error in dpptrf(1), info = %d.\n",NAinfo);
                        return 1;
                    }
                    break;
                }
                /* ith blocks of scaled matrices Fsc_j, j=1,...,m */
                for (j=0; j<m; j++) {
                    dspgst_(&int1,"L",F_blkszs+i,Fsc+j*F_sz+pos,dpt,&NAinfo);
                    if (NAinfo){ 
                        fprintf(stderr,"Error in dspgst(1), info = %d.\n",NAinfo);
                        return 1;
                    }
                }
            }

            if (iters==1 && !dual_PD_feas) {
                /* preliminary Newton iteration:
                 * each iteration updates t such that Newton decrement
                 * is minimized. Denote the hessian by
                 *   H = tM_1 + M_2  (t > 0)
                 * we will simultaneously diagonalize M_1 and M_2 using
                 * the following generalized eigen-decomp (M_1+M_2>0 is known):
                 * M_1*V = (M_1+M_2)*V*Lambda such that
                 *   V^T*(M_1+M_2)*V = I
                 *   V^T*M_1*V       = Lambda
                 * thus, V^T*H*V = I + (t-1)*Lambda and
                 * g^T*H^{-1}*g = g^T*V*(V^T*H*V)^{-1}*V^T*g
                 *              = \tilde{g}^T\tilde{H}^{-1}\tilde{g}
                 * where
                 *   \tilde{g} = tv_1 + v_2;
                 *   \tilde{H} = tSigM + (I-SigM)
                 * now we could easily compute the
                 * gradient and hessian of Newton decrement over t.
                 * Newton direction is then computed directly from -H^{-1}g */

                /* (M1)_{ij} = \Tr(Gsc_i,Gsc_j);
                   (M2)_{ij} = \Tr(Fsc_i,Fsc_j);  (M2 stored in LM) */
                for (i=0, dps=M1, dpt=LM; i<m; i++)
                    for(j=i; j<m; j++, dps++, dpt++) {
                        *dps = inprd(Gsc+i*G_sz,Gsc+j*G_sz,K,G_blkszs);
                        *dpt = inprd(Fsc+i*F_sz,Fsc+j*F_sz,L,F_blkszs);
                    }
                for (i=0, dpt=LM, dps=M1; i<M_sz; i++, dpt++, dps++)
                    *dpt += *dps;    /* LM = M2+M1, in packed storage */
                /* generalize eigen-decomp: eig(M1,M1+M2) */
                dspgv_(&int1,"V","L",&m,M1,LM,sigM,VM,&m,wkspc,&NAinfo);
                if (NAinfo>m) {
                    fprintf(stderr, "maxdet: matrices diag(Fi,Gi), i=1,...,m\
 are linearly dependent\n        (or F(x) and/or G(x) are very badly\
 conditioned).\n");
                    *info = -23;
                    return 1;
                } else if (NAinfo) {
                    fprintf(stderr,"Error in dspgv(0), info = %d.\n",NAinfo);
                    return 1;
                }
                /* v1 = VM^T*(c-\Tr(Gsc,I));  v2 = VM^T*(-\TR(Fsc,I)) */
                memcpy(temp,c,m*sizeof(double));    /* copy c to temp */
                for (i=0; i<m; i++)
                    for (j=0, dps=Gsc+i*G_sz; j<K; j++)
                        for (k=G_blkszs[j]; k>0; dps+=k, k--)
                            *(temp+i) -= *dps;      /* temp = c-\Tr(Gsc,I) */
                dgemv_("T",&m,&m,&dbl1,VM,&m,temp,&int1,&dbl0,v1,&int1);
                memset(temp,0,m*sizeof(double));
                for (i=0; i<m; i++)
                    for (j=0, dps=Fsc+i*F_sz; j<L; j++)
                        for (k=F_blkszs[j]; k>0; dps+=k, k--)
                            *(temp+i) -= *dps;      /* temp = -\Tr(Fsc,I) */
                dgemv_("T",&m,&m,&dbl1,VM,&m,temp,&int1,&dbl0,v2,&int1);
                /* Newton's method for minimizing t */
                /* t_old = t; */
                for (i=0; i<LSITERUB; i++) {
                    grad1 = 0.0;
                    hess1 = 0.0;
                    for (j=0, dps=v1, dpt=sigM; j<m; j++, dps++, dpt++) {
                        dtmp = t * *dps + *(v2+j);     /* \tilde{g} */
                        dtmp2 = (t-1) * *dpt + 1.0;    /* \tilde{H} */
                        grad1 += 2 * *dps * dtmp / dtmp2 -
                            SQR(dtmp) * *dpt / SQR(dtmp2);
                        hess1 += 2 * SQR(*dps) / dtmp2 -
                            4 * dtmp * *dps * *dpt / SQR(dtmp2) +
                            2 * SQR(dtmp) * SQR(*dpt) / (SQR(dtmp2)*dtmp2);
                    }
                    if (hess1<=0) {
                        lambda = 0;
                    } else {
                        lambda = sqrt(SQR(grad1)/hess1);
                        t = MAX(1,t-1/(1+lambda)*(grad1/hess1)); 
                                  /* guarded by 1 */
                    }
                    if (lambda < LSTOL)
                        break;    /* break line search loop */
                }
                t = MAX(1,t);     /* t is guarded by 1 */
#ifdef debug
printf("t=%10.4e from preliminary phase\n",t);
#endif
                /* dx = -VM*((t*v1+v2)./((t-1)*sigM+I)), stored in rhs */
                for (i=0, dpt=rhs; i<m; i++, dpt++)
                    *dpt = -( (t * *(v1+i) + *(v2+i)) /
                              ((t-1) * *(sigM+i)+ 1.0) );
                memcpy(temp,rhs,m*sizeof(double));
                dgemv_("N",&m,&m,&dbl1,VM,&m,temp,&int1,&dbl0,rhs,&int1);
                sqrtt = sqrt(t);

            } else {

                if (NTcount==1 && iters==1) {
                    /* compute initial t from duality gap:
                     *   t = MAX(1, n/gap)
                     *   gap = c^Tx + \TrZF_0 + \TrWG_0 - l - \logdet G*W
                     *       = \TrZF + TrWG - l -logdet GW
                     *         (where GW = LG^T*W*LG)
                     *       = \TrZF + sum_i eig(GW)_i -1 - \log eig(GW)_i */
                    gap = inprd(Z,XF,L,F_blkszs);      /* \TrZF */
                    /* compute GW = LG^T*W*LG */
                    memcpy(rhs,W,G_sz*sizeof(double)); /* copy W to rhs(=GW) */
                    for (i=0, pos=0, dpt=LG; i<K;
                         pos+=G_blkszs[i]*(G_blkszs[i]+1)/2, dpt=LG+pos, i++) {
                        dspgst_(&int2,"L",G_blkszs+i,rhs+pos,dpt,&NAinfo);
                        if (NAinfo){ 
                            fprintf(stderr,"Error in dspgst(2), info = %d.\n",
                                    NAinfo);
                            return 1;
                        }
                    }
                    eig_val(sigG,rhs,K,G_blkszs,G_sz,wkspc);
                    for (i=0; i<l; i++)    /* sigG = eig(GW) here */
                        gap += sigG[i]-1.0-log(sigG[i]);
                    t = MAX(1.0 , n/gap);
                    sqrtt = sqrt(t);
#ifdef debug
printf("t=%10.2e from feasible dual\n",t);
#endif
                }

                /* Newton iteration:
                 * solve primal LS problem to find Newton direction */
                memcpy(rhs,W,G_sz*sizeof(double));       /* copy -W to rhs */
                dscal_(&G_sz,&dblm1,rhs,&int1);
                memcpy(rhs+G_sz,Z,F_sz*sizeof(double));  /* copy -t*Z to rhs */
                dtmp = -t;
                dscal_(&F_sz,&dtmp,rhs+G_sz,&int1);
                /* form rhs of primal LS (G, W related) */
                for (i=0, pos=0, dpt=LG; i<K;
                     pos+=G_blkszs[i]*(G_blkszs[i]+1)/2, dpt=LG+pos, i++) {
                    /* ith block of rhs is sqrt(t)*(I-GW)
                     *   GW = LG^T*W*LG */
                    dspgst_(&int2,"L",G_blkszs+i,rhs+pos,dpt,&NAinfo);
                    if (NAinfo){ 
                        fprintf(stderr,"Error in dspgst(3), info = %d.\n",NAinfo);
                        return 1;
                    }
                    /* add 1 to diagonal elements of rhs and scale them
                     * by 1/sqrt(2) */
                    for (k=0, pos2=pos; k<G_blkszs[i]; pos2+=G_blkszs[i]-k,
                         k++)
                        *(rhs+pos2) = (*(rhs+pos2)+1.0)/sqrt(2.0);
                    /* scale rhs by sqrt(t) (2 lines below) */
                }
                dscal_(&G_sz,&sqrtt,rhs,&int1);
                /* form rhs of primal LS (F, Z related) */
                for (i=0, pos=0, dpt=LF; i<L;
                     pos+=F_blkszs[i]*(F_blkszs[i]+1)/2, dpt=LF+pos, i++) {
                    /* ith block of rhs is I-t*FZ
                     *   FZ = LF^T*Z*LF */
                    dspgst_(&int2,"L",F_blkszs+i,rhs+G_sz+pos,dpt,&NAinfo);
                    if (NAinfo){ 
                        fprintf(stderr,"Error in dspgst(4), info = %d.\n",NAinfo);
                        return 1;
                    }
                    /* add 1 to diagonal elements of rhs and scale them
                     * by 1/sqrt(2) */
                    for (k=0, pos2=pos; k<F_blkszs[i]; pos2+=F_blkszs[i]-k,
                         k++)
                        *(rhs+G_sz+pos2) = (*(rhs+G_sz+pos2)+1.0)/sqrt(2.0);
                }
                /* prepare the left-hand-side of the primal LS problem */
                for (i=0, dpt=GaF; i<m; i++, dpt+=sz) {
                    memcpy(dpt,Gsc+i*G_sz,G_sz*sizeof(double));
                    for (j=0, dps=dpt; j<K; j++)
                        for (k=G_blkszs[j]; k>0; dps+=k, k--)
                            *dps /= sqrt(2.0);   /* scale diagonal elements */
                    memcpy(dpt+G_sz,Fsc+i*F_sz,F_sz*sizeof(double));
                    for (j=0, dps=dpt+G_sz; j<L; j++)
                        for (k=F_blkszs[j]; k>0; dps+=k, k--)
                            *dps /= sqrt(2.0);   /* scale diagonal elements */
                }
                dlascl_("G",NULL,NULL,&dbl1,&sqrtt,&G_sz,&m,GaF,&sz,&NAinfo);
                if (NAinfo){ 
                    fprintf(stderr,"Error in dlascl(0), info = %d.\n",NAinfo);
                    return 1;
                }
                /* solve the primal LS problem */
                dgels_("N",&sz,&m,&int1,GaF,&sz,rhs,&sz,wkspc,&lwkspc,&NAinfo);
                if (NAinfo){ 
                    fprintf(stderr,"Error in dgels(0), info = %d.\n",NAinfo);
                    return 1;
                }

                /* check the rank of GaF (for the first iteration only):
                 * estimate the condition number in 1-norm of R of the QR-decomp
                 * of GaF (stored in upper-triangular part of GaF after calling
                 *  dgels). if rcond < MINRCOND, GaF is low-rank */
                if (iters == 1) {
                    dtrcon_("1","U","N",&m,GaF,&sz,&rcond,wkspc,iwork,&NAinfo);
                    if (NAinfo < 0){
                        fprintf(stderr,"Error in dtrcon(0), info = %d.\n", NAinfo);
                        return 1;
                    }
                    if (rcond < MINRCOND) {
                        fprintf(stderr,"maxdet: matrices diag(G_i,F_i), i=1,...,m\
 are linearly dependent\n        (or F(x) and/or G(x) are very badly\
 conditioned).\n");
                        *info = -23;
                        return 1;
                    }
                }
            }

            /* compute dXG and dXF */
            dgemv_("N",&G_sz,&m,&dbl1,G+G_sz,&G_sz,rhs,&int1,&dbl0,dXG,&int1);
            dgemv_("N",&F_sz,&m,&dbl1,F+F_sz,&F_sz,rhs,&int1,&dbl0,dXF,&int1);

            /* find sigG=eig(dXG,XG) and sigF=eig(dXF,XF) */
            memcpy(temp,XG,G_sz*sizeof(double));      /* copy XG to temp */
            memcpy(temp2,dXG,G_sz*sizeof(double));    /* copy dXG to temp2 */
            for (i=0, pos=0, dpt=sigG; i<K; dpt+=G_blkszs[i],
                 pos+=G_blkszs[i]*(G_blkszs[i]+1)/2, i++) {
                switch (G_blkszs[i]) {
                  case 1:
                    *dpt = *(temp2+pos) / *(temp+pos);
                    break;
                  default:
                    dspgv_(&int1,"N","L",G_blkszs+i,temp2+pos,temp+pos,dpt,
                           NULL,&int1,wkspc,&NAinfo);  /* workspace 3*max_l */
                    if (NAinfo) {
                        fprintf(stderr,"Error in dspgv(1), info = %d.\n",NAinfo);
                        return 1;
                    }
                    break;
                }
            }
            memcpy(temp,XF,F_sz*sizeof(double));      /* copy XF to temp */
            memcpy(temp2,dXF,F_sz*sizeof(double));    /* copy dXF to temp2 */
            for (i=0, pos=0, dpt=sigF; i<L; dpt+=F_blkszs[i],
                 pos+=F_blkszs[i]*(F_blkszs[i]+1)/2, i++) {
                switch (F_blkszs[i]) {
                  case 1:
                    *dpt = *(temp2+pos) / *(temp+pos);
                    break;
                  default:
                    dspgv_(&int1,"N","L",F_blkszs+i,temp2+pos,temp+pos,dpt,
                           NULL,&int1,wkspc,&NAinfo);  /* workspace 3*max_n */
                    if (NAinfo) {
                        fprintf(stderr,"Error in dspgv(2), info = %d.\n",NAinfo);
                        return 1;
                    }
                    break;
                }
            }

            /* check dual infeasibility (for the preliminary phase ONLY):
             *   if sigG and sigF are all non-negative, and c^Tdx <=0,
             *   we conclude dual infeasibility */
            if (iters==1 && !dual_PD_feas) {

                int neg_sig=NO;

                for (i=0, dps=sigG; (i<l && !neg_sig); i++, dps++)
                    if (*dps<0)  neg_sig=YES;
                for (i=0, dps=sigF; (i<n && !neg_sig); i++, dps++)
                    if (*dps<0)  neg_sig=YES;
                if (ddot_(&m,c,&int1,rhs,&int1)<=0 && !neg_sig) {
                    fprintf(stderr,"maxdet: The dual problem is infeasible.\n");
                    return 1;
                }
            }

            /* compute Newton decrement (denoted by mu) */
            mu = 0.0;
            for (i=0, dps=sigG; i<l; i++, dps++)
                mu += SQR(*dps);
            mu *= t;
            for (i=0, dps=sigF; i<n; i++, dps++)
                mu += SQR(*dps);
            mu = sqrt(mu);
            if (mu < CENTOL)
                break;    /* break the loop of NTcount */

            /* determine step length:
             *   mu > .5 -- use line search
             *   mu <=.5 -- use Newton step obtained */
            if (mu > .5) {
                for (i=0, *alpha=0.0; i<LSITERUB; i++) {
                    grad1 = 0.0;
                    hess1 = 0.0;
                    for (j=0, dps=sigG; j<l; j++, dps++) {
                        dtmp = 1 + *alpha * *dps;
                        grad1 -= *dps / dtmp;
                        hess1 += SQR(*dps) / SQR(dtmp);
                    }
                    grad1 *= t;
                    hess1 *= t;
                    for (j=0, dps=sigF; j<n; j++, dps++) {
                        dtmp = 1 + *alpha * *dps;
                        grad1 -= *dps / dtmp;
                        hess1 += SQR(*dps) / SQR(dtmp);
                    }
                    grad1 += t * ddot_(&m,c,&int1,rhs,&int1);
                    lambda = sqrt(SQR(grad1)/hess1);
                    *alpha -= 1/(1+lambda)*(grad1/hess1);
                    if (lambda < LSTOL)
                        break;    /* break line search loop */
                }
            } else {
                *alpha = 1.0;
            }

            /* corrector step updates */
            daxpy_(&m,alpha,rhs,&int1,x,&int1);      /* x  += alpha * dx  */
            daxpy_(&G_sz,alpha,dXG,&int1,XG,&int1);  /* XG += alpha * dXG */
            daxpy_(&F_sz,alpha,dXF,&int1,XF,&int1);  /* XF += alpha * dXF */

        }   /* end of Newton iterations loop */


        /* update dual:
         *   W = LG^{-T}*(I-Gsc*dx)*LG^{-1}
         *   Z = (1/t)*(LF^{-T}*(I-Fsc*dx)*LF^{-1}) */
        memcpy(ZWtmp,Z,F_sz*sizeof(double));      /* copy of Z for later use */
        memcpy(ZWtmp+F_sz,W,G_sz*sizeof(double)); /* copy of W for later use */
        memset(W,0,G_sz*sizeof(double));
        memset(Z,0,F_sz*sizeof(double));
        for (i=0, dpt=W; i<K; i++)    /* store I in W */
            for (j=G_blkszs[i]; j>0; dpt+=j, j--)
                *dpt = 1.0;
        dgemv_("N",&G_sz,&m,&dblm1,Gsc,&G_sz,rhs,&int1,&dbl1,W,&int1);
        memcpy(LGinv,LG,G_sz*sizeof(double));
        /* loop over the blocks for scaling of W */
        for (i=0, pos=0; i<K; pos+=G_blkszs[i]*(G_blkszs[i]+1)/2, i++) {
            dtptri_("L","N",G_blkszs+i,LGinv+pos,&NAinfo);  /* find LG^{-1} */
            if (NAinfo) {
                fprintf(stderr,"Error in dtptri(0), info = %d.\n",NAinfo);
                return 1;
            }
            dspgst_(&int2,"L",G_blkszs+i,W+pos,LGinv+pos,&NAinfo);
            if (NAinfo) {
                fprintf(stderr,"Error in dspgst(5), info = %d.\n",NAinfo);
                return 1;
            }
        }
        for (i=0, dpt=Z; i<L; i++)    /* store I in Z */
            for (j=F_blkszs[i]; j>0; dpt+=j, j--)
                *dpt = 1.0;
        dgemv_("N",&F_sz,&m,&dblm1,Fsc,&F_sz,rhs,&int1,&dbl1,Z,&int1);
        memcpy(LFinv,LF,F_sz*sizeof(double));
        /* loop over the blocks for scaling of Z*/
        for (i=0, pos=0; i<L; pos+=F_blkszs[i]*(F_blkszs[i]+1)/2, i++) {
            dtptri_("L","N",F_blkszs+i,LFinv+pos,&NAinfo);  /* find LF^{-1} */
            if (NAinfo) {
                fprintf(stderr,"Error in dtptri(1), info = %d.\n",NAinfo);
                return 1;
            }
            dspgst_(&int2,"L",F_blkszs+i,Z+pos,LFinv+pos,&NAinfo);
            if (NAinfo) {
                fprintf(stderr,"Error in dspgst(6), info = %d.\n",NAinfo);
                return 1;
            }
        }
        dtmp=1/t;
        dscal_(&F_sz,&dtmp,Z,&int1);

        /* check the positive-definiteness of Z and W: (test only if
         * within preliminary phase or maxNTiters exceeded)
         *   if not,
         *     within the preliminary phase it indicates the dual
         *     is probably infeasible.
         *     if maxNTiters is exceeded (non-PD of W and Z is caused by
         *     prematural termination of Newton's method), then retrieve
         *     the previous W and Z saved */
        if ((iters==1 && !dual_PD_feas) || *NTiters>=maxNTiters)
            if (eig_val(sigZ,Z,L,F_blkszs,F_sz,wkspc)<=0 ||
                eig_val(sigW,W,K,G_blkszs,G_sz,wkspc)<=0)
                if (iters==1 && !dual_PD_feas) {
                    fprintf(stderr,"maxdet: Infeasible dual updates, the dual\
 problem is likely to be infeasible.\n");
                    return 1;
                } else {
                    memcpy(Z,ZWtmp,F_sz*sizeof(double));       /* recover Z */
                    memcpy(W,ZWtmp+F_sz,G_sz*sizeof(double));  /* recover W */
                }

        /* update duality gap:
         *   gap = c^Tx + \TrZF_0 + \TrWG_0 - l - \logdet G - \logdet W
         *       = c^Tx + \TrZF_0 + \TrWG_0 - l - \logdet (I - dXG*G^{-1})
         *       = \TrZF + sum_i eig(GW)_i -1 - \log eig(GW)_i
         *   where GW = LG^T*W*LG
         *
         * NOTE: the 3rd formula is used to evaluate the gap, even it requires
         *       eigen-decompositions of GW. the second one has serious
         *       numerical problems, especially at center where XG~=W^{-1} */
        gap = inprd(Z,XF,L,F_blkszs);         /* \TrZF */
        /* compute GW = LG^T*W*LG */
        memcpy(rhs,W,G_sz*sizeof(double));    /* copy W to rhs(=GW) */
        for (i=0, pos=0, dpt=LG; i<K;
             pos+=G_blkszs[i]*(G_blkszs[i]+1)/2, dpt=LG+pos, i++) {
            dspgst_(&int2,"L",G_blkszs+i,rhs+pos,dpt,&NAinfo);
            if (NAinfo){ 
                fprintf(stderr,"Error in dspgst(7), info = %d.\n",NAinfo);
                return 1;
            }
        }
        eig_val(sigG,rhs,K,G_blkszs,G_sz,wkspc);
        for (i=0; i<l; i++)    /* sigG = eig(GW) here */
            gap += sigG[i]-1.0-log(sigG[i]);
 
        /* recompute ul[0] */
        eig_val(sigG,XG,K,G_blkszs,G_sz,wkspc);  /* sigG = eig(G) here */
        ul[0] = ddot_(&m,c,&int1,x,&int1);
        for (i=0; i<l; i++)
            ul[0] -= log(sigG[i]);

        /* update ul[1] from gap and ul[0] */
        ul[1] = ul[0] - gap;

        /* update hist */
        dpt = hist+(iters-1)*3;
        *dpt = ul[0];
        *(dpt+1) = gap;
        *(dpt+2) = NTcount;
#if MAXDET_VERBOSE
        /* check convergence */
        fprintf(stdout,"  %3d     %10.2e     %10.2e\n",*NTiters,ul[0],gap);
#endif
        if (gap <= MAX(abstol,MINABSTOL)) {
            *info = 2;    /* absolute accuracy achieved */
            return(0);
        }
        if ( (ul[1] > 0 && gap <= reltol*ul[1]) ||
             (ul[0] < 0 && gap <= reltol*(-ul[0])) ) {
            *info = 3;    /* relative accuracy achieved */
            return(0);
        }
        if (*NTiters >= maxNTiters) {
            *info = 1;    /* max number of total Newton iterations exceeded */
            return(0);
        }



        /* PREDICTOR step:
         *   1. update t by at least l(t^+/t - 1 - \log t^+/t) = \gamma
         *   2. compute tangential direction
         *   3. minimize \psi_ub along the tangential direction (plane search)
         *   4. update t such that \psi_ub = \gamma (Newton's method)
         *   5. repeat 3 and 4 until either min \psi_ub in 3 is close to
         *      \gamma, or t is greater than 10 times n/abstol */

        /* decompositions of XG and XF for the PREDICTOR step */
        memcpy(Gsc,G+G_sz,m*G_sz*sizeof(double));  /* copy G to Gsc */
        memcpy(Fsc,F+F_sz,m*F_sz*sizeof(double));  /* copy F to Fsc */
        memcpy(LG,XG,G_sz*sizeof(double));         /* copy XG to LG */
        memcpy(LF,XF,F_sz*sizeof(double));         /* copy XF to LF */
        memcpy(rhs,W,G_sz*sizeof(double));         /* copy -W to rhs(1:) */
        dscal_(&G_sz,&dblm1,rhs,&int1);
        memcpy(rhs+G_sz,Z,F_sz*sizeof(double));    /* copy -t*Z to rhs(:sz) */
        dtmp = -t;
        dscal_(&F_sz,&dtmp,rhs+G_sz,&int1);
        /* decomposition of XG */
        for (i=0, pos=0, dpt=LG; i<K;
             pos+=G_blkszs[i]*(G_blkszs[i]+1)/2, dpt=LG+pos, i++) {
            /* loop over blocks, to compute rhs and Gsc.
             * DPPTRF is called only if the block size exceeds 2 */
            switch (G_blkszs[i]) {
              case 1:    /* 1-by-1 block */
                if (*dpt<=0) {
                  fprintf(stderr,"maxdet: x infeasible because G(x)<=0.\n");
                  *info = -9;
                  return 1;
                }
                *dpt = sqrt(*dpt);
                break;
              case 2:    /* 2-by-2 block */
                if (*dpt + *(dpt+2)<=0 ||
                    *dpt * *(dpt+2) - SQR(*(dpt+1))<=0) {
                  fprintf(stderr,"maxdet: x infeasible because G(x)<=0.\n");
                  *info = -9;
                  return 1;
                }
                *dpt = sqrt(*dpt);
                *(dpt+1) = *(dpt+1) / *dpt;
                *(dpt+2) = sqrt(*(dpt+2)-SQR(*(dpt+1)));
                break;
              default:   /* 3-by-3 or larger block */
                dpptrf_("L",G_blkszs+i,dpt,&NAinfo);
                if (NAinfo>0) {
                  fprintf(stderr,"maxdet: x infeasible because G(x)<=0.\n");
                  *info = -9;
                  return 1;
                } else if (NAinfo) {
                    fprintf(stderr,"Error in dpptrf(2), info = %d.\n",NAinfo);
                    return 1;
                }
                break;
            }
            /* ith blocks of scaled matrices Gsc_j, j=1,...,m
             *   Gsc = LG^{-1}*G_i*LG^{-T} */
            for (j=0; j<m; j++) {
                dspgst_(&int1,"L",G_blkszs+i,Gsc+j*G_sz+pos,dpt,&NAinfo);
                if (NAinfo){ 
                    fprintf(stderr,"Error in dspgst(8), info = %d.\n",NAinfo);
                    return 1;
                }
            }
            /* ith block of rhs is sqrt(t)*(I-GW)
             *   GW = LG^T*W*LG */
            dspgst_(&int2,"L",G_blkszs+i,rhs+pos,dpt,&NAinfo);
            if (NAinfo){ 
                fprintf(stderr,"Error in dspgst(9), info = %d.\n",NAinfo);
                return 1;
            }
            /* add 1 to diagonal elements of rhs and scale them
             * by 1/sqrt(2) */
            for (k=0, pos2=pos; k<G_blkszs[i]; pos2+=G_blkszs[i]-k,k++)
                *(rhs+pos2) = (*(rhs+pos2)+1.0)/sqrt(2.0);
            /* scale rhs by sqrt(t) (2 lines below) */
        }
        dscal_(&G_sz,&sqrtt,rhs,&int1);
        /* decomposition of XF */
        for (i=0, pos=0, dpt=LF; i<L;
             pos+=F_blkszs[i]*(F_blkszs[i]+1)/2, dpt=LF+pos, i++) {
            /* loop over blocks, to compute rhs and Fsc.
             * DPPTRF is called only if the block size exceeds 2 */
            switch (F_blkszs[i]) {
              case 1:    /* 1-by-1 block */
                if (*dpt<=0) {
                  fprintf(stderr,"maxdet: x infeasible because F(x)<=0.\n");
                  *info = -9;
                  return 1;
                }
                *dpt = sqrt(*dpt);
                break;
              case 2:    /* 2-by-2 block */
                if (*dpt + *(dpt+2)<=0 ||
                    *dpt * *(dpt+2) - SQR(*(dpt+1))<=0) {
                  fprintf(stderr,"maxdet: x infeasible because F(x)<=0.\n");
                  *info = -9;
                  return 1;
                }
                *dpt = sqrt(*dpt);
                *(dpt+1) = *(dpt+1) / *dpt;
                *(dpt+2) = sqrt(*(dpt+2)-SQR(*(dpt+1)));
                break;
              default:   /* 3-by-3 or larger block */
                dpptrf_("L",F_blkszs+i,dpt,&NAinfo);
                if (NAinfo>0) {
                  fprintf(stderr,"maxdet: x infeasible because F(x)<=0.\n");
                  *info = -9;
                  return 1;
                } else if (NAinfo) {
                    fprintf(stderr,"Error in dpptrf(3), info = %d.\n",NAinfo);
                    return 1;
                }
                break;
            }
            /* ith blocks of scaled matrices Fsc_j, j=1,...,m
             *   Fsc = LF^{-1}*F_i*LF^{-T} */
            for (j=0; j<m; j++) {
                dspgst_(&int1,"L",F_blkszs+i,Fsc+j*F_sz+pos,dpt,&NAinfo);
                if (NAinfo){ 
                    fprintf(stderr,"Error in dspgst(10), info = %d.\n",NAinfo);
                    return 1;
                }
            }
            /* ith block of rhs is -t*FZ
             *   FZ = LF^T*Z*LF */
            dspgst_(&int2,"L",F_blkszs+i,rhs+G_sz+pos,dpt,&NAinfo);
            if (NAinfo){ 
                fprintf(stderr,"Error in dspgst(11), info = %d.\n",NAinfo);
                return 1;
            }
            /* scale diagonal elements of rhs by 1/sqrt(2) */
            for (k=0, pos2=pos; k<F_blkszs[i]; pos2+=F_blkszs[i]-k,k++)
                *(rhs+G_sz+pos2) = *(rhs+G_sz+pos2) / sqrt(2.0);
        }
        /* prepare the left-hand-side of the LS problem */
        for (i=0, dpt=GaF; i<m; i++, dpt+=sz) {
            memcpy(dpt,Gsc+i*G_sz,G_sz*sizeof(double));
            for (j=0, dps=dpt; j<K; j++)       /* scale diagonal elements */
                for (k=G_blkszs[j]; k>0; dps+=k, k--)
                    *dps /= sqrt(2.0);
            memcpy(dpt+G_sz,Fsc+i*F_sz,F_sz*sizeof(double));
            for (j=0, dps=dpt+G_sz; j<L; j++)  /* scale diagonal elements */
                for (k=F_blkszs[j]; k>0; dps+=k, k--)
                    *dps /= sqrt(2.0);
        }
        dlascl_("G",NULL,NULL,&dbl1,&sqrtt,&G_sz,&m,GaF,&sz,&NAinfo);
        if (NAinfo){ 
            fprintf(stderr,"Error in dlascl(1), info = %d.\n",NAinfo);
            return 1;
        }
        /* solve the PREDICTOR LS problem */
        dgels_("N",&sz,&m,&int1,GaF,&sz,rhs,&sz,wkspc,&lwkspc,&NAinfo);
        if (NAinfo){ 
            fprintf(stderr,"Error in dgels(1), info = %d.\n",NAinfo);
            return 1;
        }
        /* increase *NTiters by 1 */
        (*NTiters)++;

        /* compute dXG and dXF */
        dgemv_("N",&G_sz,&m,&dbl1,G+G_sz,&G_sz,rhs,&int1,&dbl0,dXG,&int1);
        dgemv_("N",&F_sz,&m,&dbl1,F+F_sz,&F_sz,rhs,&int1,&dbl0,dXF,&int1);

        /* compute dW = t*(-W + G^{-1} - G^{-1}*dXG*G^{-1})
         *            = t*(-W + LG^{-T}*(I-Gsc*dx)*LG^{-1})
         *         dZ = -t*Z - F^{-1}*dXF*F^{-1}
         *            = -t*Z + LF^{-T}*(-Fsc*dx)*LF^{-1} */
        memset(dW,0,G_sz*sizeof(double));
        memset(dZ,0,F_sz*sizeof(double));
        for (i=0, dpt=dW; i<K; i++)    /* store I in dW */
            for (j=G_blkszs[i]; j>0; dpt+=j, j--)
                *dpt = 1.0;
        dgemv_("N",&G_sz,&m,&dblm1,Gsc,&G_sz,rhs,&int1,&dbl1,dW,&int1);
        memcpy(LGinv,LG,G_sz*sizeof(double));
        /* loop over the blocks for scaling of dW */
        for (i=0, pos=0; i<K; pos+=G_blkszs[i]*(G_blkszs[i]+1)/2, i++) {
            dtptri_("L","N",G_blkszs+i,LGinv+pos,&NAinfo);  /* find LG^{-1} */
            if (NAinfo) {
                fprintf(stderr,"Error in dtptri(2), info = %d.\n",NAinfo);
                return 1;
            }
            dspgst_(&int2,"L",G_blkszs+i,dW+pos,LGinv+pos,&NAinfo);
            if (NAinfo) {
                fprintf(stderr,"Error in dspgst(12), info = %d.\n",NAinfo);
                return 1;
            }
        }                            /* dW = LG^{-T}*(I-Gsc*dx)*LG^{-1} */
        for (i=0, dpt=dW, dps=W; i<G_sz; i++, dpt++, dps++)
            *dpt = t*(*dpt - *dps);  /* dW=t*(-W+LG^{-T}*(I-Gsc*dx)*LG^{-1}) */

        dgemv_("N",&F_sz,&m,&dblm1,Fsc,&F_sz,rhs,&int1,&dbl0,dZ,&int1);
        memcpy(LFinv,LF,F_sz*sizeof(double));
        /* loop over the blocks for scaling of dZ*/
        for (i=0, pos=0; i<L; pos+=F_blkszs[i]*(F_blkszs[i]+1)/2, i++) {
            dtptri_("L","N",F_blkszs+i,LFinv+pos,&NAinfo);  /* find LF^{-1} */
            if (NAinfo) {
                fprintf(stderr,"Error in dtptri(3), info = %d.\n",NAinfo);
                return 1;
            }
            dspgst_(&int2,"L",F_blkszs+i,dZ+pos,LFinv+pos,&NAinfo);
            if (NAinfo) {
                fprintf(stderr,"Error in dspgst(13), info = %d.\n",NAinfo);
                return 1;
            }
        }                            /* dZ = LF^{-T}*(-Fsc*dx)*LF^{-1} */
        dtmp = -t;                   /* dZ = -t*Z + LF^{-T}*(-Fsc*dx)*LF^{-1} */
        daxpy_(&F_sz,&dtmp,Z,&int1,dZ,&int1);

        /* find least t update: y = t^+/t via solving 
         *   n(y-1-\log y) = \gamma  (for the first iteration only) */
        if (iters == 1) {
            dtmp = gamma/n;
            y = 1+sqrt(2.0*dtmp);    /* lower bound of the solution */
            dtmp2 = y;
            lambda = y-1-log(y)-dtmp;
            for (i=0; i<LSITERUB; i++) {
                y = y-lambda/(1-1/y);
                lambda = y-1-log(y)-dtmp;
                if (fabs(lambda)<LSTOL)
                    break;
            }
            y = MAX(y,dtmp2);
        }

        /* least t update */
        t *= y;

        /* predictor-step plane search iterations */
        for (i=0; i<LSITERUB; i++) {

            double norm_p, norm_d, norm_max;
            double gap_upd1=0.0, gap_upd2=0.0;

            /* general eigen-decomp for plane-search:
             * find sigG=eig(dXG,XG) and sigW=eig(dW,W) */
            memcpy(temp,XG,G_sz*sizeof(double));
            memcpy(temp2,dXG,G_sz*sizeof(double));
            memcpy(temp3,W,G_sz*sizeof(double));
            memcpy(temp4,dW,G_sz*sizeof(double));
            for (j=0, pos=0, dpt=sigG, dps=sigW; j<K; dpt+=G_blkszs[j],
                 dps+=G_blkszs[j], pos+=G_blkszs[j]*(G_blkszs[j]+1)/2, j++) {
                switch (G_blkszs[j]) {
                  case 1:
                    *dpt = *(dXG+pos) / *(XG+pos);
                    *dps = *(dW+pos) / *(W+pos);
                    break;
                  default:
                    dspgv_(&int1,"N","L",G_blkszs+j,temp2+pos,temp+pos,dpt,
                           NULL,&int1,wkspc,&NAinfo);
                    if (NAinfo) {
                        fprintf(stderr,"Error in dspgv(3), info = %d.\n",NAinfo);
                        return 1;
                    }
                    dspgv_(&int1,"N","L",G_blkszs+j,temp4+pos,temp3+pos,dps,
                           NULL,&int1,wkspc,&NAinfo);
                    if (NAinfo) {
                        fprintf(stderr,"Error in dspgv(4), info = %d.\n",NAinfo);
                        return 1;
                    }
                    break;
                }
            }
            /* find sigF=eig(dXF,XF) and sigZ=eig(dZ,Z) */
            memcpy(temp,XF,F_sz*sizeof(double));
            memcpy(temp2,dXF,F_sz*sizeof(double));
            memcpy(temp3,Z,F_sz*sizeof(double));
            memcpy(temp4,dZ,F_sz*sizeof(double));
            for (j=0, pos=0, dpt=sigF, dps=sigZ; j<L; dpt+=F_blkszs[j],
                 dps+=F_blkszs[j], pos+=F_blkszs[j]*(F_blkszs[j]+1)/2, j++) {
                switch (F_blkszs[j]) {
                  case 1:
                    *dpt = *(dXF+pos) / *(XF+pos);
                    *dps = *(dZ+pos) / *(Z+pos);
                    break;
                  default:
                    dspgv_(&int1,"N","L",F_blkszs+j,temp2+pos,temp+pos,dpt,
                           NULL,&int1,wkspc,&NAinfo);
                    if (NAinfo) {
                        fprintf(stderr,"Error in dspgv(5), info = %d.\n",NAinfo);
                        return 1;
                    }
                    dspgv_(&int1,"N","L",F_blkszs+j,temp4+pos,temp3+pos,dps,
                           NULL,&int1,wkspc,&NAinfo);
                    if (NAinfo) {
                        fprintf(stderr,"Error in dspgv(6), info = %d.\n",NAinfo);
                        return 1;
                    }
                    break;
                }
            }

            /* check degenerate cases:
             * plane search degenerates to line search if norm_p or norm_d
             * is zero. */
            norm_p = dnrm2_(&n,sigF,&int1);
            norm_p += dnrm2_(&l,sigG,&int1);
            norm_d = dnrm2_(&n,sigZ,&int1);
            norm_d += dnrm2_(&l,sigW,&int1);
            norm_max = MAX(norm_p,norm_d);

            /* predictor-step plane-search:
             *   minimize \psi_{ub} over the feasible rectangle */
            alpha[0]=0.0;
            alpha[1]=0.0;
            for (j=0; j<LSITERUB; j++) {
                grad1 = 0.0;
                grad2 = 0.0;
                hess1 = 0.0;
                hess2 = 0.0;
                for (k=0, dps=sigG, dpt=sigW; k<l; k++, dps++, dpt++) {
                    dtmp = 1 + alpha[0] * *dps;
                    dtmp2 = 1 + alpha[1] * *dpt;
                    grad1 -= t * *dps / dtmp;
                    grad2 -= t * *dpt / dtmp2;
                    hess1 += t * SQR(*dps) / SQR(dtmp);
                    hess2 += t * SQR(*dpt) / SQR(dtmp2);
                }
                for (k=0, dps=sigF, dpt=sigZ; k<n; k++, dps++, dpt++) {
                    dtmp = 1 + alpha[0] * *dps;
                    dtmp2 = 1 + alpha[1] * *dpt;
                    grad1 -= *dps / dtmp;
                    grad2 -= *dpt / dtmp2;
                    hess1 += SQR(*dps) / SQR(dtmp);
                    hess2 += SQR(*dpt) / SQR(dtmp2);
                }
                grad1 += t*ddot_(&m,c,&int1,rhs,&int1);
                grad2 += t*(inprd(dW,G,K,G_blkszs)+inprd(dZ,F,L,F_blkszs));
                /* check degenrate cases */
                if (norm_p>SIGTOL*norm_max && norm_d>SIGTOL*norm_max) {
                    lambda = sqrt(SQR(grad1)/hess1+SQR(grad2)/hess2);
                    dtmp = 1/(1+lambda);
                    alpha[0] -= dtmp*grad1/hess1;
                    alpha[1] -= dtmp*grad2/hess2;
                } else if (norm_p>SIGTOL*norm_max) {
                    lambda = sqrt(SQR(grad1)/hess1);
                    alpha[0] -= 1/(1+lambda)*grad1/hess1;
                } else if (norm_d>SIGTOL*norm_max) {
                    lambda = sqrt(SQR(grad2)/hess2);
                    alpha[1] -= 1/(1+lambda)*grad2/hess2;
                } else
                    lambda = 0;
                if (lambda < LSTOL)
                    break;
            }

            /* update gap, ul[0], beta and \psi_{ub}:
             * stop plane search if \psi_{ub} here is close to \gamma
             *   gap = c^T(x+alpha_0*dx)
             *         +\Tr(Z+alpha_1*dZ)F_0+\Tr(W+alpha_1*dW)G_0-l
             *         -\logdet(G+alpha_0*dG)-\logdet(W+alpha_1*dW)
             *       = gap_old+alpha0*c^Tdx+alpha_1(\TrdWG_0+\TrdZF_0)
             *         -\logdet(I+alpha_0*G^{-1}dG)
             *         -\logdet(I+alpha_1*W^{-1}dW)
             *   beta = -\logdet(F+alpha_0*dF)-\logdet(Z+alpha_1*dZ)-n
             *        = beta_old-\logdet(I+alpha_0*F^{-1}dF)
             *          -\logdet(I+alpha_1*Z^{-1}dZ) 
             *   \psi_{ub} = t*gap+beta-n*log(t) */
            if (!i) {
                gap_upd1 = ddot_(&m,c,&int1,rhs,&int1);
                gap_upd2 = inprd(dZ,F,L,F_blkszs)+inprd(dW,G,K,G_blkszs);
            }
            dtmp = alpha[0]*gap_upd1;
            gap += dtmp+alpha[1]*gap_upd2;
            ul[0] += dtmp;
            for (j=0; j<l; j++) {
                gap -= (log(1.0 + alpha[0] * *(sigG+j))
                    +log(1.0 + alpha[1] * *(sigW+j)));
                ul[0] -= log(1.0 + alpha[0] * *(sigG+j));
            }

            /* predictor step updates */
            daxpy_(&m,alpha,rhs,&int1,x,&int1);      /* x  += alpha[0] * dx  */
            daxpy_(&G_sz,alpha,dXG,&int1,XG,&int1);  /* XG += alpha[0] * dXG */
            daxpy_(&F_sz,alpha,dXF,&int1,XF,&int1);  /* XF += alpha[0] * dXF */
            daxpy_(&G_sz,alpha+1,dW,&int1,W,&int1);  /* W += alpha[1] * dW */
            daxpy_(&F_sz,alpha+1,dZ,&int1,Z,&int1);  /* Z += alpha[1] * dZ */

            if (!i) {
                beta = -n;
                eig_val(sigF,XF,L,F_blkszs,F_sz,wkspc); /* sigF=eig(XF) here */
                eig_val(sigZ,Z,L,F_blkszs,F_sz,wkspc);  /* sigZ=eig(Z) here */
                for (j=0; j<n; j++)
                    beta += log(1 / *(sigF+j)) + log(1 / *(sigZ+j));
            } else {
                for (j=0; j<n; j++)
                    beta -= log(1.0 + alpha[0] * *(sigF+j))
                        +log(1.0 + alpha[1] * *(sigZ+j));
            }

            /* find \psi_{ub} and check \gamma-\psi_{ub} */
            psi_ub = t*gap+beta-n*log(t);
            dtmp = MAX(abstol,MAX(MINABSTOL,reltol*MAX(-ul[0],ul[1])));
            if (gamma-psi_ub < LSTOL || t > 10*n/MAX(abstol,MINABSTOL)
                || gap<dtmp)
                break;    /* break plane search iteration */

            /* update t: compute t^+ from \psi_{ub}=\gamma using
             * Newton's method */
            t_old = t;
            t = t_old+(gamma-psi_ub)/gap;    /* lower bound of solution */
            dtmp = t;
            lambda = t*gap+beta-n*log(t)-gamma;  /* psi_ub - gamma */
            for (j=0; j<LSITERUB; j++) {
                t -= lambda/(gap-n/t);
                lambda = t*gap+beta-n*log(t)-gamma;
                if (fabs(lambda) < LSTOL)
                    break;
            }
            t = MAX(t,dtmp);

        }   /* end of predictor plane search iteration loop */

    }   /* end of the outer iteration loop */

    return 1;    /* should never return from here */
}

