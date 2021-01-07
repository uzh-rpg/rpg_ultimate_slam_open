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
#include <random>
#include <utility>

#include <ze/common/numerical_derivative.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/transformation.hpp>
#include <ze/common/types.hpp>
#include <ze/geometry/align_poses.hpp>

TEST(AlignPosesTest, testJacobian)
{
#ifndef ZE_SINGLE_PRECISION_FLOAT
  using namespace ze;

  Transformation T_A0_B0, T_Ai_A0, T_B0_Bi;
  T_A0_B0.setRandom(1.0);
  T_Ai_A0.setRandom(1.0);
  T_B0_Bi.setRandom(1.0);

  auto residualLambda = [&](const Transformation& T_A0_B0) {
      return (T_Ai_A0 * T_A0_B0 * T_B0_Bi).log();
    };
  Matrix6 J_numeric = numericalDerivative<Vector6, Transformation>(residualLambda, T_A0_B0);
  Matrix6 J_analytic = dRelpose_dTransformation(T_A0_B0, T_Ai_A0, T_B0_Bi);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(J_numeric, J_analytic));
#else
  LOG(WARNING) << "Numerical derivative test ignored for single precision float.";
#endif
}

TEST(AlignPosesTest, testOptimization)
{
  using namespace ze;

  const size_t n_poses = 20;

  // Generate random trajectory.
  TransformationVector T_W_A(n_poses);
  for (Transformation& T : T_W_A)
  {
    T.setRandom(2.0);
  }

  // Random transformation between trajectories.
  Transformation T_A0_B0;
  T_A0_B0.setRandom();

  // Compute transformed trajectory.
  TransformationVector T_W_B(n_poses);
  for (size_t i = 0; i < n_poses; ++i)
  {
    Transformation T_A0_Ai = T_W_A[0].inverse() * T_W_A[i];
    T_W_B[i] = T_W_A[0] * T_A0_B0 * T_A0_Ai;
  }

  // Perturb estimated pose.
  real_t sigma_pos = 0.05;
  real_t sigma_rot = 5.0 / 180 * M_PI;
  Vector3 pos_pert = Vector3::Random();
  pos_pert = pos_pert.normalized() * sigma_pos;
  Vector3 rot_pert = Vector3::Random();
  rot_pert = rot_pert.normalized() * sigma_rot;
  Transformation T_pert = Transformation::exp((Vector6() << pos_pert, rot_pert).finished());
  Transformation T_A0_B0_estimate = T_A0_B0 * T_pert;

  // Optimize.
  PoseAligner problem(T_W_A, T_W_B, sigma_pos, sigma_rot);
  problem.optimize(T_A0_B0_estimate);

  // Compute error.
  Transformation T_err = T_A0_B0.inverse() * T_A0_B0_estimate;
  EXPECT_LT(T_err.log().norm(), 1.5e-5);
}

ZE_UNITTEST_ENTRYPOINT
