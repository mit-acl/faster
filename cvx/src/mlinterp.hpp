/**
 * MIT License
 *
 * Copyright (c) 2017 Parsiad Azimzadeh
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MLINTERP_HPP
#define MLINTERP_HPP

#include <cassert>
#include <cstddef>
#include <limits>

namespace mlinterp {

namespace {

template <typename T, typename...> struct helper {
  template <typename Index> void run(const Index *, Index, Index *, T *) {}
};

template <typename T, typename T1, typename T2, typename... Args>
struct helper<T, T1, T2, Args...> : helper<T, Args...> {
  const T *xd;
  const T *xi;

  helper(T1 xd, T2 xi, Args... args)
      : helper<T, Args...>(args...), xd(xd), xi(xi) {}

  template <typename Index>
  void run(const Index *nd, Index n, Index *indices, T *weights) {
    // Must have at least one point per axis
    assert(*nd > 0);

    const T x = xi[n];

    Index mid;
    T weight;

    if (*nd == 1 || x <= xd[0]) {
      // Data point is less than left boundary
      mid = 0;
      weight = 1.;
    } else if (x >= xd[*nd - 1]) {
      // Data point is greater than right boundary
      mid = *nd - 2;
      weight = 0.;
    } else {
      // Binary search to find tick
      Index lo = 0, hi = *nd - 2;
      mid = 0;
      weight = 0.;
      while (lo <= hi) {
        mid = lo + (hi - lo) / 2;
        if (x < xd[mid]) {
          hi = mid - 1;
        } else if (x >= xd[mid + 1]) {
          lo = mid + 1;
        } else {
          weight = (xd[mid + 1] - x) / (xd[mid + 1] - xd[mid]);
          break;
        }
      }
    }

    *indices = mid;
    *weights = weight;

    helper<T, Args...> &base = (*this);
    base.run(nd + 1, n, indices + 1, weights + 1);
  }
};

} // namespace

struct natord {
  template <typename Index, Index Dimension>
  static Index mux(const Index *nd, const Index *indices) {
    Index index = 0, product = 1, i = Dimension - 1;
    while (true) {
      index += indices[i] * product;
      if (i == 0) {
        break;
      }
      product *= nd[i--];
    }
    return index;
  }
};

struct rnatord {
  template <typename Index, Index Dimension>
  static Index mux(const Index *nd, const Index *indices) {
    Index index = 0, product = 1, i = 0;
    while (true) {
      index += indices[i] * product;
      if (i == Dimension - 1) {
        break;
      }
      product *= nd[i++];
    }
    return index;
  }
};

template <typename Order = natord, typename... Args, typename T, typename Index>
static void interp(const Index *nd, Index ni, const T *yd, T *yi,
                   Args... args) {
  // Cannot have a negative number of points
  assert(ni >= 0);

  // Infer dimension from arguments
  static_assert(sizeof...(Args) % 2 == 0, "needs 4+2*Dimension arguments");
  constexpr Index Dimension = sizeof...(Args) / 2;

  // Compute 2^Dimension
  constexpr Index Power = 1 << Dimension;

  // Unpack arguments
  helper<T, Args...> h(args...);

  // Perform interpolation for each point
  Index indices[Dimension];
  T weights[Dimension];
  Index buffer[Dimension];
  T factor;
  for (Index n = 0; n < ni; ++n) {
    yi[n] = 0.;
    h.run(nd, n, indices, weights);
    for (Index bitstr = 0; bitstr < Power; ++bitstr) {
      factor = 1.;
      for (Index i = 0; i < Dimension; ++i) {
        if (bitstr & (1 << i)) {
          buffer[i] = indices[i];
          factor *= weights[i];
        } else {
          buffer[i] = indices[i] + 1;
          factor *= 1 - weights[i];
        }
      }
      if (factor > std::numeric_limits<T>::epsilon()) {
        const Index k = Order::template mux<Index, Dimension>(nd, buffer);
        yi[n] += factor * yd[k];
      }
    }
  }
}

} // namespace mlinterp

#endif
