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

#include <utility>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <ze/common/logging.hpp>

#include <ze/common/stl_utils.hpp>

//! @file statistics.hpp
//! Various utilities, e.g. to compute statistical properties of vectors.

namespace ze {

//! Does not take const-ref because vector will be sorted.
template <typename Scalar>
std::pair<Scalar, bool> median(std::vector<Scalar>& v)
{
  if(v.size() == 0)
  {
    LOG(WARNING) << "Median computation of empty vector.";
    return std::make_pair(Scalar{0}, false);
  }
  const size_t center = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + center, v.end());
  Scalar median = v[center];
  return std::make_pair(median, true);
}

template <typename DerivedVec>
std::pair<typename DerivedVec::Scalar, bool> median(const Eigen::MatrixBase<DerivedVec>& v)
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedVec);
  auto w = eigenVectorToStlVector(v);
  return median<typename DerivedVec::Scalar>(w);
}

template<class T>
inline T normPdf(const T x, const T mean, const T sigma)
{
  T exponent = x - mean;
  exponent *= -exponent;
  exponent /= 2 * sigma * sigma;
  T result = std::exp(exponent);
  result /= sigma * std::sqrt(2 * M_PI);
  return result;
}

//! Calculate the covariance of a matrix of measurements. The measurements are
//! stacked column-wise in the matrix.
inline const MatrixX measurementCovariance(const Eigen::Ref<MatrixX>& values)
{
  MatrixX zero_mean = values.colwise() - values.rowwise().mean();

  return (zero_mean * zero_mean.adjoint()) / (zero_mean.cols() - 1);
}

} // namespace ze
