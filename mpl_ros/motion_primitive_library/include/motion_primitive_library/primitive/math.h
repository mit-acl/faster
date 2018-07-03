/**
 * @file math.h
 * @brief Polynomial roots solver

 * Solving real roots for n-th order polynomial:
    if n < 5, the closed form solution will be calculated;
    if n >= 5, using Eigen Polynomials solver which is slower but correct.
 */
#pragma once
#include <motion_primitive_library/common/data_type.h>
#include <unsupported/Eigen/Polynomials>
#include <iostream>

///Quadratic equation: \f$b*t^2+c*t+d = 0\f$
std::vector<decimal_t> quad(decimal_t b, decimal_t c, decimal_t d) ;
///Cubic equation: \f$a*t^3+b*t^2+c*t+d = 0\f$
std::vector<decimal_t> cubic(decimal_t a, decimal_t b, decimal_t c, decimal_t d) ;
///Quartic equation: \f$a*t^4+b*t^3+c*t^2+d*t+e = 0\f$
std::vector<decimal_t> quartic(decimal_t a, decimal_t b, decimal_t c, decimal_t d, decimal_t e) ;
/*! \brief General solver for \f$a*t^4+b*t^3+c*t^2+d*t+e = 0\f$

  \f$a, b, c\f$ can be zero. The function itself checks the highest order of the polynomial.
  */
std::vector<decimal_t> solve(decimal_t a, decimal_t b, decimal_t c, decimal_t d, decimal_t e);
///A more general solver for \f$a*t^6+b*t^5+c*t^4+d*t^3+e*t^2+f*t+g = 0\f$
std::vector<decimal_t> solve(decimal_t a, decimal_t b, decimal_t c, decimal_t d, decimal_t e, decimal_t f, decimal_t g);

///Return \f$n!\f$
int factorial(int n);

///Return \f$t^n\f$
decimal_t power(decimal_t t, int n);
 
