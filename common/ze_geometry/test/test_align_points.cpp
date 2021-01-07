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
#include <functional>
#include <utility>

#include <ze/common/numerical_derivative.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/transformation.hpp>
#include <ze/common/types.hpp>
#include <ze/common/random.hpp>
#include <ze/geometry/align_points.hpp>

#ifndef ZE_SINGLE_PRECISION_FLOAT
ze::real_t tol = 1e-10;
#else
ze::real_t tol = 1e-6;
#endif

TEST(AlignPointsTest, testJacobian)
{
#ifndef ZE_SINGLE_PRECISION_FLOAT
  using namespace ze;

  Vector3 p_A = Vector3::Random();
  Vector3 p_B = Vector3::Random();
  Transformation T_A_B;
  T_A_B.setRandom(1.0);

  auto residualLambda = [&](const Transformation& T_A_B) {
    return Vector3(p_A - T_A_B * p_B);
  };

  Matrix36 J_numeric = numericalDerivative<Vector3, Transformation>(residualLambda, T_A_B);
  Matrix36 J_analytic = dPointdistance_dRelpose(T_A_B, p_A, p_B);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(J_numeric, J_analytic));
#else
  LOG(WARNING) << "Numerical derivative test ignored for single precision float.";
#endif
}

TEST(AlignPosesTest, testOptimization)
{
  using namespace ze;

  const size_t n_points = 100;

  // Generate random points.
  Positions p_B(3, n_points);
  for (size_t i = 0; i < n_points; ++i)
  {
    p_B.col(i) = Vector3::Random();
  }

  // Random transformation between trajectories.
  Transformation T_A_B;
  T_A_B.setRandom();

  // Compute transformed points.
  Positions p_A = T_A_B.transformVectorized(p_B);

  // Align trajectories.
  PointAligner problem(p_A, p_B);
  Vector6 perturbation = Vector6::Ones() * 0.1;
  Transformation T_A_B_estimate =
      T_A_B * Transformation::exp(perturbation);
  problem.optimize(T_A_B_estimate);

  // Compute error.
  Transformation T_err = T_A_B.inverse() * T_A_B_estimate;
  EXPECT_LT(T_err.log().norm(), tol);
}

TEST(AlignPosesTest, testAlignSE3)
{
  using namespace ze;

  const size_t n_points = 100;

  // Generate random points.
  Positions p_B(3, n_points);
  for (size_t i = 0; i < n_points; ++i)
  {
    p_B.col(i) = Vector3::Random();
  }

  // Random transformation between trajectories.
  Transformation T_A_B;
  T_A_B.setRandom();

  // Compute transformed points.
  Positions p_A = T_A_B.transformVectorized(p_B);

  // Align trajectories
  Transformation T_A_B_estimate = alignSE3(p_B, p_A);

  // Compute error.
  Transformation T_err = T_A_B.inverse() * T_A_B_estimate;
  EXPECT_LT(T_err.log().norm(), tol);
}

TEST(AlignPosesTest, testAlignSim3)
{
  using namespace ze;

  const size_t n_points = 100;

  // Generate random points.
  Positions p_B(3, n_points);
  for (size_t i = 0; i < n_points; ++i)
  {
    p_B.col(i) = Vector3::Random();
  }
  real_t scale =
      sampleUniformRealDistribution<real_t>(false, 0.1, 10.0);

  // Random transformation between trajectories.
  Transformation T_A_B;
  T_A_B.setRandom();

  // Compute transformed points.
  Positions p_A =
      (scale * T_A_B.getRotation().rotateVectorized(p_B)).colwise()
      + T_A_B.getPosition();

  // Align trajectories
  std::pair<real_t, Transformation> sim3_estimate = alignSim3(p_B, p_A);
  Transformation T_A_B_estimate = sim3_estimate.second;
  real_t scale_estimate = sim3_estimate.first;

  // Compute error.
  Transformation T_err = T_A_B.inverse() * T_A_B_estimate;
  EXPECT_LT(T_err.log().norm(), tol);
  EXPECT_LT(std::abs(scale - scale_estimate), tol);
}

ZE_UNITTEST_ENTRYPOINT
