// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <iostream>
#include <functional>
#include <ze/common/types.hpp>
#include <ze/common/manifold.hpp>

namespace ze {

//! Template function to compute numerical derivatives. See unit tests for examples.
//! The traits used for this functions are defined in common/manifold.h
template<class Y, class X>
typename Eigen::Matrix<real_t, traits<Y>::dimension, traits<X>::dimension>
numericalDerivative(std::function<Y(const X&)> h, const X& x, real_t delta = 1e-5)
{
  typedef typename Eigen::Matrix<real_t, traits<Y>::dimension, traits<X>::dimension> Jacobian;
  typedef typename traits<Y>::TangentVector TangentY;
  typedef typename traits<X>::TangentVector TangentX;

  const int N_X = traits<X>::getDimension(x);

  // Get value at x.
  Y hx = h(x);

  const int N_Y = traits<Y>::getDimension(hx);

  // Prepare a tangent vector to perturb x.
  TangentX dx(N_X, 1);
  dx.setZero();

  // Compute numerical Jacobian column by column.
  Jacobian H(N_Y, N_X);
  H.setZero();

  real_t factor = 1.0 / (2.0 * delta);

  for(int i = 0; i < N_X; ++i)
  {
    dx(i) = delta;
    TangentY dy1 = traits<Y>::local(hx, h(traits<X>::retract(x, dx)));
    dx(i) = -delta;
    TangentY dy2 = traits<Y>::local(hx, h(traits<X>::retract(x, dx)));
    dx(i) = 0;
    H.col(i) << (dy1 - dy2) * factor;
  }
  return H;
}

} // namespace ze
