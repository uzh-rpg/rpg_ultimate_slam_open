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

#include <ze/common/random.hpp>
#include <ze/common/types.hpp>
#include <ze/common/macros.hpp>

//! @file random_matrix.hpp
//! Sample matrices and vectors from uniform or normal distributions.

namespace ze {

//------------------------------------------------------------------------------
//! A sampler for uncorrelated noise vectors.
template<size_t DIM>
class RandomVectorSampler
{
public:
  ZE_POINTER_TYPEDEFS(RandomVectorSampler);

  typedef Eigen::Matrix<real_t, DIM, DIM> covariance_matrix_t;
  typedef Eigen::Matrix<real_t, DIM, 1> covariance_vector_t;
  typedef Eigen::Matrix<real_t, DIM, 1> sigma_vector_t;
  typedef Eigen::Matrix<real_t, DIM, 1> noise_vector_t;

  //! Get a noise sample.
  noise_vector_t sample()
  {
    noise_vector_t noise;
    for (size_t i = 0; i < DIM; ++i)
    {
      // The gaussian takes a standard deviation as input.
      noise(i) = sampleNormalDistribution<real_t>(deterministic_, 0.0, sigma_(i));
    }
    return noise;
  }

  static Ptr sigmas(const sigma_vector_t& sigmas, bool deterministic = false)
  {
    Ptr noise(new RandomVectorSampler(deterministic));
    noise->sigma_ = sigmas;
    return noise;
  }

  static Ptr variances(const covariance_vector_t& variances, bool deterministic = false)
  {
    Ptr noise(new RandomVectorSampler(deterministic));
    noise->sigma_ = variances.cwiseSqrt();
    return noise;
  }

protected:
  RandomVectorSampler(bool deteterministic)
    : deterministic_(deteterministic)
  {}

private:
  const bool deterministic_;
  sigma_vector_t sigma_;
};

//------------------------------------------------------------------------------
inline MatrixX randomMatrixUniformDistributed(
    int rows,
    int cols,
    bool deterministic = false,
    real_t from  = 0.0,
    real_t to    = 1.0)
{
  DEBUG_CHECK_GT(rows, 0);
  DEBUG_CHECK_GT(cols, 0);
  MatrixX m(rows, cols);
  for (int x = 0; x < cols; ++x)
  {
    for (int y = 0; y < rows; ++y)
    {
      m(y,x) = sampleUniformRealDistribution(deterministic, from, to);
    }
  }
  return m;
}

template<int rows, int cols>
Eigen::Matrix<real_t, rows, cols>
randomMatrixUniformDistributed(
    bool deterministic = false,
    real_t from = 0.0,
    real_t to   = 1.0)
{
  return randomMatrixUniformDistributed(rows, cols, deterministic, from, to);
}

template<int size>
Eigen::Matrix<real_t, size, 1>
randomVectorUniformDistributed(
    bool deterministic = false,
    real_t from = 0.0,
    real_t to   = 1.0)
{
  return randomMatrixUniformDistributed<size, 1>(deterministic, from, to);
}

//------------------------------------------------------------------------------
inline MatrixX randomMatrixNormalDistributed(
    int rows,
    int cols,
    bool deterministic = false,
    real_t mean  = 0.0,
    real_t sigma = 1.0)
{
  DEBUG_CHECK_GT(rows, 0);
  DEBUG_CHECK_GT(cols, 0);
  MatrixX m(rows, cols);
  for (int x = 0; x < cols; ++x)
  {
    for (int y = 0; y < rows; ++y)
    {
      m(y,x) = sampleNormalDistribution(deterministic, mean, sigma);
    }
  }
  return m;
}

template<int rows, int cols>
Eigen::Matrix<real_t, rows, cols>
randomMatrixNormalDistributed(
    bool deterministic = false,
    real_t mean  = 0.0,
    real_t sigma = 1.0)
{
  return randomMatrixNormalDistributed(rows, cols, deterministic, mean, sigma);
}

template<int size>
Eigen::Matrix<real_t, size, 1>
randomVectorNormalDistributed(
    bool deterministic = false,
    real_t mean  = 0.0,
    real_t sigma = 1.0)
{
  return randomMatrixNormalDistributed<size, 1>(deterministic, mean, sigma);
}

} // namespace ze
