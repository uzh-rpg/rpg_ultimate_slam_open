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

#include <ze/common/logging.hpp>

#include <ze/common/manifold.hpp>
#include <ze/common/numerical_derivative.hpp>

namespace ze {

template<typename T>
void testManifoldInvariants(const T& a, const T& b, real_t tol = 1e-9)
{
  CHECK(traits<T>::equals(a, a));
  typename traits<T>::TangentVector v = traits<T>::local(a, b);
  T c = traits<T>::retract(a, v);
  CHECK(traits<T>::equals(b, c, tol));
}

template<typename T>
void testRetractJacobians(const T& a, const T& b, real_t tol = 1e-9)
{
  using namespace std::placeholders; // for _1
  typename traits<T>::Jacobian H1, H2;
  typename traits<T>::TangentVector v = traits<T>::local(a, b);
  T c = traits<T>::retract(a, v, &H1, &H2);
  typename traits<T>::Jacobian H1_numerical =
      numericalDerivative<T, T>(
        std::bind(traits<T>::retract, _1, v, nullptr, nullptr), a);
  CHECK(traits<typename traits<T>::Jacobian>::equals(H1, H1_numerical, tol));

  typename traits<T>::Jacobian H2_numerical =
      numericalDerivative<T, typename traits<T>::TangentVector>(
        std::bind(traits<T>::retract, a, _1, nullptr, nullptr), v);
  CHECK(traits<typename traits<T>::Jacobian>::equals(H2, H2_numerical, tol));
}

template<typename T>
void testLocalJacobians(const T& a, const T& b, real_t tol = 1e-9)
{
  using namespace std::placeholders; // for _1
  typename traits<T>::Jacobian H1, H2;
  typename traits<T>::TangentVector v = traits<T>::local(a, b, &H1, &H2);
  typename traits<T>::Jacobian H1_numerical =
      numericalDerivative<typename traits<T>::TangentVector, T>(
        std::bind(traits<T>::local, _1, b, nullptr, nullptr), a);
  CHECK(traits<typename traits<T>::Jacobian>::equals(H1, H1_numerical, tol));

  typename traits<T>::Jacobian H2_numerical =
      numericalDerivative<typename traits<T>::TangentVector, T>(
        std::bind(traits<T>::local, a, _1, nullptr, nullptr), b);
  CHECK(traits<typename traits<T>::Jacobian>::equals(H2, H2_numerical, tol));
}
} // namespace ze
