/**
 * @file data_type.h
 * @brief Defines all data types used in this lib

 * Mostly alias from Eigen Library.
 */

#include <stdio.h>
#include <math.h>
#include <limits>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#ifndef BASIC_COLOR_H
#define BASIC_COLOR_H
///Set red font in printf funtion 
#define ANSI_COLOR_RED "\x1b[31m"
///Set green font in printf funtion 
#define ANSI_COLOR_GREEN "\x1b[32m"
///Set yellow font in printf funtion 
#define ANSI_COLOR_YELLOW "\x1b[33m"
///Set blue font in printf funtion 
#define ANSI_COLOR_BLUE "\x1b[34m"
///Set magenta font in printf funtion 
#define ANSI_COLOR_MAGENTA "\x1b[35m"
///Set cyan font in printf funtion 
#define ANSI_COLOR_CYAN "\x1b[36m"
///Reset font color in printf funtion 
#define ANSI_COLOR_RESET "\x1b[0m"
#endif

#ifndef BASIC_DATA_H
#define BASIC_DATA_H
/*! \brief Rename the float type used in lib 

    Default is set to be double, but user can change it to float.
*/
typedef double decimal_t;


///Pre-allocated std::vector for Eigen using vec_E
template <typename T> 
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
///Eigen 1D float vector
template <int N> 
using Vecf = Eigen::Matrix<decimal_t, N, 1>;
///Eigen 1D int vector
template <int N> 
using Veci = Eigen::Matrix<int, N, 1>;
///MxN Eigen matrix
template <int M, int N> 
using Matf = Eigen::Matrix<decimal_t, M, N>;
///MxN Eigen matrix with M unknown
template <int N> 
using MatDNf = Eigen::Matrix<decimal_t, Eigen::Dynamic, N>;
///Vector of Eigen 1D float vector
template <int N> 
using vec_Vecf = vec_E<Vecf<N>>;
///Vector of Eigen 1D int vector
template <int N> 
using vec_Veci = vec_E<Veci<N>>;

///Eigen 1D float vector of size 2
typedef Vecf<2> Vec2f;
///Eigen 1D int vector of size 2
typedef Veci<2> Vec2i;
///Eigen 1D float vector of size 3
typedef Vecf<3> Vec3f;
///Eigen 1D int vector of size 3
typedef Veci<3> Vec3i;
///Eigen 1D float vector of size 4
typedef Vecf<4> Vec4f;
///Column vector in float of size 6
typedef Vecf<6> Vec6f;

///Vector of type Vec2f.
typedef vec_E<Vec2f> vec_Vec2f;
///Vector of type Vec2i.
typedef vec_E<Vec2i> vec_Vec2i;
///Vector of type Vec3f.
typedef vec_E<Vec3f> vec_Vec3f;
///Vector of type Vec3i.
typedef vec_E<Vec3i> vec_Vec3i;

///2x2 Matrix in float
typedef Matf<2, 2> Mat2f;
///3x3 Matrix in float
typedef Matf<3, 3> Mat3f;
///4x4 Matrix in float
typedef Matf<4, 4> Mat4f;
///6x6 Matrix in float
typedef Matf<6, 6> Mat6f;

///Dynamic Nx1 Eigen float vector
typedef Vecf<Eigen::Dynamic> VecDf;
///Mx3 Eigen float matrix
typedef MatDNf<3> MatD3f;
///Dynamic MxN Eigen float matrix
typedef Matf<Eigen::Dynamic, Eigen::Dynamic> MatDf;

///Allias of Eigen::Affine2d
typedef Eigen::Transform<decimal_t, 2, Eigen::Affine> Aff2f;
///Allias of Eigen::Affine3d
typedef Eigen::Transform<decimal_t, 3, Eigen::Affine> Aff3f;

///Ellipsoid: first is the Affine Transform, second is the center
typedef std::pair<Mat3f, Vec3f> Ellipsoid;
///Vector of Ellipsoids
typedef vec_E<Ellipsoid> vec_Ellipsoid;

///Face class
class Face {
  public:
    Vec3f p;
    Vec3f n;
    bool pass;

    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Face(Vec3f _p, Vec3f _n):
      p(_p), n(_n), pass(true) {}
    Face(Vec3f _p, Vec3f _n, bool _pass):
      p(_p), n(_n), pass(_pass) {}
};

///Polyhedron, consists of faces
typedef vec_E<Face> Polyhedron; // composed by planes with form (p, n)
///Vector of Polyhedron
typedef vec_E<Polyhedron> Polyhedra;

///Extreme points of a polyhedron
typedef vec_E<vec_Vec3f> BoundVec3f; // compose by extreme points


#endif

#ifndef BASIC_DECOMP_H
#define BASIC_DECOMP_H
//Allias of Eigen::Translation
typedef Eigen::Translation<decimal_t, 3> Trans3f;
//Allias of Eigen::AngleAxis
typedef Eigen::AngleAxis<decimal_t> Anglef;

///Allias of Eigen::Quaterniond
typedef Eigen::Quaternion<decimal_t> Quatf;
///std::pair of Eigen::Vector3d
typedef std::pair<Vec3f, Vec3f> pair_Vec3f;
///[A, b] for \f$Ax <= b\f$
typedef std::pair<MatD3f, VecDf> LinearConstraint3f; 
///Vector of LinearConstraint
typedef vec_E<LinearConstraint3f> vec_LinearConstraint3f;


#endif
