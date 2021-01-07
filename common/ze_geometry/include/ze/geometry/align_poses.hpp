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

#pragma once

#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>
#include <ze/geometry/robust_cost.hpp>
#include <ze/geometry/lsq_solver.hpp>

namespace ze {

//! Estimates relative transformation between two sets of associated pints
class PoseAligner :
    public LeastSquaresSolver<Transformation, PoseAligner>
{
public:
  using LeastSquaresSolver::HessianMatrix;
  using LeastSquaresSolver::GradientVector;
  using ScaleEstimator = UnitScaleEstimator<real_t>;
  using WeightFunction = UnitWeightFunction<real_t>;

  PoseAligner(
      const TransformationVector& T_W_A,
      const TransformationVector& T_W_B,
      const real_t measurement_sigma_pos,
      const real_t measurement_sigma_rot);

  double evaluateError(
      const Transformation& T_A_B,
      HessianMatrix* H,
      GradientVector *g);

private:
  const TransformationVector& T_W_A_;
  const TransformationVector& T_W_B_;
  real_t measurement_sigma_pos_;
  real_t measurement_sigma_rot_;
};

inline Matrix6 dRelpose_dTransformation(
    const Transformation& T_A0_B0,
    const Transformation& T_Ai_A0,
    const Transformation& T_B0_Bi)
{
  Quaternion R_error = T_Ai_A0.getRotation() * T_A0_B0.getRotation() * T_B0_Bi.getRotation();
  Matrix3 R_Ai_B0 = T_Ai_A0.getRotationMatrix() * T_A0_B0.getRotationMatrix();
  Matrix3 R_Bi_B0 = T_B0_Bi.getRotation().inverse().getRotationMatrix();
  Matrix6 J = Z_6x6;
  J.block<3,3>(0,0) = R_Ai_B0; // drt / dt
  J.block<3,3>(0,3) = - R_Ai_B0 * skewSymmetric(T_B0_Bi.getPosition()); // drt / dR
  J.block<3,3>(3,3) = logmapDerivativeSO3(R_error.log()) * R_Bi_B0; // drR / dR
  return J;
}

} // namespace ze
