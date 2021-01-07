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

#include <ze/geometry/align_points.hpp>

#include <cmath>
#include <numeric>
#include <algorithm>
#include <ze/common/logging.hpp>

#include <ze/common/matrix.hpp>
#include <ze/common/stl_utils.hpp>

namespace ze {

PointAligner::PointAligner(
    const Positions& p_A,
    const Positions& p_B)
  : p_A_(p_A)
  , p_B_(p_B)
{
  CHECK_EQ(p_A_.cols(), p_B_.cols());
}

// -----------------------------------------------------------------------------
double PointAligner::evaluateError(
    const Transformation& T_A_B,
    HessianMatrix* H,
    GradientVector* g)
{
  double chi2 = 0.0;

  // Compute prediction error.
  Positions residuals = p_A_ - T_A_B.transformVectorized(p_B_);
  VectorX residuals_norm = residuals.colwise().norm();

  // At the first iteration, compute the scale of the error.
  if(iter_ == 0)
  {
    measurement_sigma_ = ScaleEstimator::compute(residuals_norm);
    VLOG(1) << "measurement sigma = " << measurement_sigma_;
  }

  // Robust cost function.
  VectorX weights =
      WeightFunction::weightVectorized(residuals_norm / measurement_sigma_);

  // Whiten error.
  residuals_norm /= measurement_sigma_;

  if(H && g)
  {
    for(int i = 0; i < p_A_.cols(); ++i)
    {
      // Compute Jacobian (if necessary, this can be optimized a lot).
      Matrix36 J = dPointdistance_dRelpose(T_A_B, p_A_.col(i), p_B_.col(i));

      // Whiten Jacobian.
      J /= measurement_sigma_;

      // Compute Hessian and Gradient Vector.
      H->noalias() += J.transpose() * J * weights(i);
      g->noalias() -= J.transpose() * residuals.col(i) * weights(i);
    }
  }

  // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
  chi2 += 0.5 * weights.dot(residuals.colwise().squaredNorm());

  return chi2;
}

// -----------------------------------------------------------------------------
Transformation alignSE3(const Positions& pts_A, const Positions& pts_B)
{
  CHECK_GE(pts_A.cols(), 0);
  CHECK_EQ(pts_A.cols(), pts_B.cols());

  // Compute T_B_A using the method by K. S. Arun et al.:
  // Least-Squares Fitting of Two 3-D Point Sets
  // IEEE Trans. Pattern Anal. Mach. Intell., 9, NO. 5, SEPTEMBER 1987
  const Position mean_pts_A = pts_A.rowwise().mean();
  const Position mean_pts_B = pts_B.rowwise().mean();
  const Positions zero_mean_pts_A = pts_A.colwise() - mean_pts_A;
  const Positions zero_mean_pts_B = pts_B.colwise() - mean_pts_B;
  const Matrix3 Sigma_AB =
      zero_mean_pts_B * zero_mean_pts_A.transpose() / pts_A.cols();

  Eigen::JacobiSVD<Matrix3> svd(
        Sigma_AB, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const int rank_Sigma_AB = svd.rank();
  CHECK_GE(rank_Sigma_AB, 2);
  const Matrix3 svd_U = svd.matrixU();
  const Matrix3 svd_V = svd.matrixV();
  Matrix3 S = Matrix3::Identity();
  if ((rank_Sigma_AB == 3 &&
       Sigma_AB.determinant() < 0.0) ||
      (rank_Sigma_AB == 2 &&
       svd_U.determinant() * svd_V.determinant() < 0.0))
  {
    S(2, 2) = -1.0;
  }

  const Matrix3 R_B_A = svd_U * S * svd_V.transpose();
  const Position t_B_A = mean_pts_B - R_B_A * mean_pts_A;
  return Transformation(t_B_A, Quaternion(R_B_A));
}

// -----------------------------------------------------------------------------
std::pair<real_t, Transformation> alignSim3(
    const Positions& pts_A, const Positions& pts_B)
{
  CHECK_NE(pts_A.cols(), 0);
  CHECK_EQ(pts_A.cols(), pts_B.cols());

  // Compute (scale, T_B_A) using the method by
  // S. Umeyama: Least-Squares Estimation
  // of Transformation Parameters Between Two Point Patterns,
  // IEEE Trans. Pattern Anal. Mach. Intell., vol. 13, no. 4, 1991.
  const real_t n = static_cast<real_t>(pts_A.cols());
  const Position mean_pts_A = pts_A.rowwise().mean();
  const Position mean_pts_B = pts_B.rowwise().mean();
  const Positions zero_mean_pts_A = pts_A.colwise() - mean_pts_A;
  const Positions zero_mean_pts_B = pts_B.colwise() - mean_pts_B;
  const Matrix3 Sigma_AB =
      zero_mean_pts_B * zero_mean_pts_A.transpose() / n;
  const real_t sigma_sq_A
      = zero_mean_pts_A.rowwise().squaredNorm().sum() / n;

  Eigen::JacobiSVD<Matrix3> svd(
        Sigma_AB, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const int rank_Sigma_AB = svd.rank();
  CHECK_GE(rank_Sigma_AB, 2);
  const Matrix3 svd_U = svd.matrixU();
  const Matrix3 svd_V = svd.matrixV();
  const Matrix3 svd_D = svd.singularValues().asDiagonal();
  Matrix3 S = Matrix3::Identity();
  if ((rank_Sigma_AB == 3 &&
       Sigma_AB.determinant() < 0.0) ||
      (rank_Sigma_AB == 2 &&
       svd_U.determinant() * svd_V.determinant() < 0.0))
  {
    S(2, 2) = -1.0;
  }

  const Matrix3 R_B_A = svd_U * S * svd_V.transpose();
  const real_t c = (svd_D * S).trace() / sigma_sq_A;
  const Vector3 t_B_A = mean_pts_B - c * R_B_A * mean_pts_A;
  return std::pair<real_t, Transformation>(
        c, Transformation(t_B_A, Quaternion(R_B_A)));
}

} // namespace ze
