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

#include <cmath>
#include <utility>
#include <ze/common/types.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/statistics.hpp>
#include <ze/common/random_matrix.hpp>

TEST(StatisticsTest, testMedian)
{
  Eigen::VectorXd x(5);
  x << 1, 2, 3, 4, 5;
  auto m = ze::median(x);
  EXPECT_DOUBLE_EQ(m.first, 3);
}

TEST(StatisticsTest, testMeasurementCovariance)
{
  using namespace ze;

  // Generate a random distribution matrix of known covariance.
  Vector3 variances;
  variances << 2.0, 3.0, 4.0;
  RandomVectorSampler<3>::Ptr sampler(
        RandomVectorSampler<3>::variances(variances));

  MatrixX measurements(3, 100000);
  for (int i = 0; i < 100000; ++i)
  {
    measurements.col(i) = sampler->sample();
  }

  Matrix3 cov = measurementCovariance(measurements);
  Matrix3 ref = Vector3(2.0, 3.0, 4.0).asDiagonal();

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(cov, ref, 1e-1));
}

ZE_UNITTEST_ENTRYPOINT
