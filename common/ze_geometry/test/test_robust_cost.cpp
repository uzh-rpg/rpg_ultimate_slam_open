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
//
// Modified: Robotics and Perception Group

#include <cmath>
#include <random>
#include <utility>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/types.hpp>
#include <ze/geometry/robust_cost.hpp>

TEST(RobustCostTest, testScaleEstimators)
{
  using namespace ze;

  // Generate normally distributed errors with standard deviation 3.0
  std::ranlux24 gen;
  std::normal_distribution<real_t> noise(0.0, 3.0);
  constexpr int n = 1000;
  VectorX errors(n);
  errors.setZero();
  for(int i = 0; i < n; ++i)
    errors(i) = noise(gen);

  double s1 = UnitScaleEstimator<real_t>::compute(errors);
  EXPECT_FLOATTYPE_EQ(s1, 1.0);

  double s2 = NormalDistributionScaleEstimator<real_t>::compute(errors);
  EXPECT_TRUE(std::abs(s2 - 3.0) < 0.2);

  double s3 = MADScaleEstimator<real_t>::compute(errors);
  EXPECT_TRUE(std::abs(s3 - 3.0) < 0.2);
}

TEST(RobustCostTest, testWeightFunctions)
{
  using namespace ze;

  // Generate normally distributed errors with standard deviation 3.0
  std::ranlux24 gen;
  std::normal_distribution<real_t> noise(0.0, 3.0);
  constexpr int n = 10;
  VectorX errors(n);
  errors.setZero();
  for (int i = 0; i < n; ++i)
  {
    errors(i) = noise(gen);
  }

  VectorX errors_scaled = HuberWeightFunction<real_t>::weightVectorized(errors);

  //! @todo: add checks.
  VLOG(1) << errors.transpose();
  VLOG(1) << errors_scaled.transpose();
}

ZE_UNITTEST_ENTRYPOINT
