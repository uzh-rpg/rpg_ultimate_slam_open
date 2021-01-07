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

//! Estimates relative transformation between two sets of associated points.
//! Iterative least squares solution.
class PointAligner : public LeastSquaresSolver<Transformation, PointAligner>
{
public:
  using LeastSquaresSolver::HessianMatrix;
  using LeastSquaresSolver::GradientVector;
  using ScaleEstimator = UnitScaleEstimator<real_t>;
  using WeightFunction = UnitWeightFunction<real_t>;

  PointAligner(
      const Positions& p_A,
      const Positions& p_B);

  double evaluateError(
      const Transformation& T_A_B,
      HessianMatrix* H,
      GradientVector* g);

private:
  real_t measurement_sigma_;
  const Positions& p_A_;
  const Positions& p_B_;
};

inline Matrix36 dPointdistance_dRelpose(
    const Transformation& T_A_B,
    const Eigen::Ref<const Vector3>& p_A,
    const Eigen::Ref<const Vector3>& p_B)
{
  Matrix3 R = T_A_B.getRotationMatrix();
  Matrix36 J;
  J.block<3,3>(0,0) = - R; // translation
  J.block<3,3>(0,3) = R * skewSymmetric(p_B); // orientation
  return J;
}

//! Compute LSQ alignment in SE3 (closed form solution by K. S. Arun et al.:
//! Least-Squares Fitting of Two 3-D Point Sets, IEEE Trans. Pattern Anal.
//! Mach. Intell., 9, NO. 5, SEPTEMBER 1987)
//! @param pts_A A vector of N points in the 'A' reference system (3xN)
//! @param pts_B A vector of N points in the 'B' reference system (3xN)
//! @return T_B_A such that ||T_B_A * pts_A - pts_B|| is minimized
Transformation alignSE3(
    const Positions& pts_A, const Positions& pts_B);

//! Compute LSQ alignment in Sim3 (close form solution by S. Umeyama:
//! Least-Squares Estimation of Transformation Parameters Between Two Point
//! Patterns, IEEE Trans. Pattern Anal. Mach. Intell., vol. 13, no. 4, 1991.)
//! @param pts_A A vector of N points in the 'A' reference system (3xN)
//! @param pts_B A vector of N points in the 'B' reference system (3xN)
//! @return <s, T_B_A> such that ||s*T_B_A * pts_A - pts_B|| is minimized.
std::pair<real_t, Transformation> alignSim3(
    const Positions& pts_A, const Positions& pts_B);

} // namespace ze
