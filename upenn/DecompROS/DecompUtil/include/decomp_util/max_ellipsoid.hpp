#ifndef MAX_ELLIPSOID_H
#define MAX_ELLIPSOID_H

#include <decomp_util/maxdet/maxdet_src.h>

static inline bool max_ellipsoid(const LinearConstraint3f& C,
		Ellipsoid& e){
	constexpr int m = 9;
  constexpr int nn = 10;
  const MatD3f A = C.first;
  const VecDf b = C.second;
  const int L = A.rows();

  //***** F
  std::array<Eigen::Matrix<double, Eigen::Dynamic, nn>, m+1> fs;
  fs.fill(Eigen::Matrix<double, Eigen::Dynamic, nn>(L, nn));

  for(int i = 0; i < L; i++){
    fs[0].row(i) << b[i], 0, 0, 0, b[i], 0, 0, b[i], 0, b[i];
    fs[1].row(i) << 0, 0, 0, A(i,0), 0, 0, 0, 0, 0, 0;
    fs[2].row(i) << 0, 0, 0, A(i,1), 0, 0, A(i,0), 0, 0, 0;
    fs[3].row(i) << 0, 0, 0, A(i,2), 0, 0, 0, 0, A(i,0), 0;
    fs[4].row(i) << 0, 0, 0, 0, 0, 0, A(i,1), 0, 0, 0;
    fs[5].row(i) << 0, 0, 0, 0, 0, 0, A(i,2), 0, A(i,1), 0;
    fs[6].row(i) << 0, 0, 0, 0, 0, 0, 0, 0, A(i,2), 0;
    fs[7].row(i) << -A(i,0), 0, 0, 0, -A(i,0), 0, 0, -A(i,0), 0, -A(i,0);
    fs[8].row(i) << -A(i,1), 0, 0, 0, -A(i,1), 0, 0, -A(i,1), 0, -A(i,1);
    fs[9].row(i) << -A(i,2), 0, 0, 0, -A(i,2), 0, 0, -A(i,2), 0, -A(i,2);
  }

  int cnt = 0;
  double F[(m+1)*L*nn];
  for(int j = 0; j < m+1; j++){
    for(int i = 0; i < L; i++){
      for(int k = 0; k < nn; k++){
        F[cnt] = fs[j](i,k);
        cnt ++;
      }
    }
  }

  //**** F_blkszs
  int F_blkszs[L];
  for(int i = 0; i < L; i++)
    F_blkszs[i] = 4;


  //**** G
  double G[(m+1)*6] = {
    0,0,0,0,0,0,
    1,0,0,0,0,0,
    0,1,0,0,0,0,
    0,0,1,0,0,0,
    0,0,0,1,0,0,
    0,0,0,0,1,0,
    0,0,0,0,0,1,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0
  };

  //**** G_blkszs
  constexpr int K = 1;
  int G_blkszs[K] = {3};

  //**** c
  double c[m] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  //**** initial guess x[], inherit from E
  Mat3f init_E = e.first;
  Vec3f init_d = e.second;
  double x[m] = {init_E(0,0), init_E(1,0), init_E(2,0),
  init_E(1,1), init_E(2,1), init_E(2,2),
  init_d(0), init_d(1), init_d(2)};

  //**** dual matrix, not used
  double Z[nn*L];
  for(int i = 0; i < nn*L; ++i)
    Z[i] = 0;

  double W[6] = {};

  //**** obj value
  double ul[2];

  //**** params for optimization
  int NTiters = 15;
  constexpr double gamma = 1000;
  constexpr double abstol = 1e-2;
  constexpr double reltol = 1e-2;

  double hist[3*NTiters];

  //***** params for calculating lwork
  int Fpd = L*nn;
  int Gpd = K*6;
  int Npd = Fpd + Gpd;
  int Nmax = std::max(Fpd, Gpd);
  int lwork = (2 * m + 5) * Npd + 2*(L*4 + K*3) +
    std::max(std::max(m + Npd * 32, 3*(4 + Nmax)),
             3*(m+m*m+Nmax));

  double work[lwork];
  int iwork[m];
  int info;

  //**** main function
  int result = maxdet(m, L, F, F_blkszs, K,
                      G, G_blkszs, c, x, Z, W, ul,
                      hist, gamma, abstol, reltol,
                      &NTiters, work, lwork, iwork, &info);
  if(result == 0){
    Mat3f E;
    E << x[0], x[1], x[2],
      x[1], x[3], x[4],
      x[2], x[4], x[5];

    Vec3f d(x[6], x[7], x[8]);
    e.first = E;
    e.second = d;
    //printf("volume: %f\n", exp(-ul[0]));
    return true;
  }
  else{
    printf("exit code: %d\n", info);
    return false;
  }
}

#endif
