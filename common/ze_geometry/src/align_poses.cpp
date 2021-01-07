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

#include <ze/geometry/align_poses.hpp>

#include <cmath>
#include <numeric>
#include <algorithm>
#include <ze/common/logging.hpp>

#include <ze/common/matrix.hpp>
#include <ze/common/stl_utils.hpp>

namespace ze {

PoseAligner::PoseAligner(
    const TransformationVector& T_W_A,
    const TransformationVector& T_W_B,
    const real_t measurement_sigma_pos,
    const real_t measurement_sigma_rot)
  : T_W_A_(T_W_A)
  , T_W_B_(T_W_B)
  , measurement_sigma_pos_(measurement_sigma_pos)
  , measurement_sigma_rot_(measurement_sigma_rot)
{
  CHECK_EQ(T_W_A_.size(), T_W_B_.size());
}

double PoseAligner::evaluateError(
    const Transformation& T_A0_B0,
    HessianMatrix* H,
    GradientVector* g)
{
  double chi2 = 0.0;

  // Compute prediction error.
  Matrix6X residuals(6, T_W_A_.size());


  for (size_t i = 0; i < T_W_A_.size(); ++i)
  {
    Transformation T_A0_Ai = T_W_A_[0].inverse() * T_W_A_[i];
    Transformation T_Bi_A0 = T_W_B_[i].inverse() * T_W_A_[0];
    Transformation T_Bi_Ai = T_Bi_A0 * T_A0_B0 * T_A0_Ai;
    residuals.col(i) = T_Bi_Ai.log();
  }

  // Whiten the error.
  residuals.topRows<3>()    /= measurement_sigma_pos_;
  residuals.bottomRows<3>() /= measurement_sigma_rot_;

  // Robust cost function.
  VectorX residuals_norm = residuals.colwise().norm();
  VectorX weights = WeightFunction::weightVectorized(residuals_norm);

  if (H && g)
  {
    for (size_t i = 0; i < T_W_A_.size(); ++i)
    {
      // Compute Jacobian (if necessary, this can be optimized a lot).
      Transformation T_A0_Ai = T_W_A_[0].inverse() * T_W_A_[i];
      Transformation T_Bi_A0 = T_W_B_[i].inverse() * T_W_A_[0];
      Matrix6 J = dRelpose_dTransformation(T_A0_B0, T_Bi_A0, T_A0_Ai);

      // Compute square-root of inverse covariance:
      Matrix6 R =
          (Vector6() << Vector3::Ones() / measurement_sigma_pos_,
                        Vector3::Ones() / measurement_sigma_rot_).finished().asDiagonal();

      // Whiten Jacobian.
      J *= R;

      // Compute Hessian and Gradient Vector.
      H->noalias() += J.transpose() * J * weights(i);
      g->noalias() -= J.transpose() * residuals.col(i) * weights(i);
    }
  }

  // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
  chi2 += 0.5 * weights.dot(residuals.colwise().squaredNorm());

  return chi2;
}

} // namespace ze


