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

#include <ze/geometry/clam.hpp>
#include <ze/geometry/pose_optimizer.hpp>
#include <ze/geometry/pose_prior.hpp>

namespace ze {

Clam::Clam(
    const ClamLandmarks& landmarks,
    const std::vector<ClamFrameData>& data,
    const CameraRig& rig,
    const Transformation& T_Bc_Br_prior,
    const real_t prior_weight_pos,
    const real_t prior_weight_rot)
  : landmarks_(landmarks)
  , data_(data)
  , rig_(rig)
  , T_Bc_Br_prior_(T_Bc_Br_prior)
  , prior_weight_pos_(prior_weight_pos)
  , prior_weight_rot_(prior_weight_rot)
{
  measurement_sigma_localization_.resize(data.size());
  CHECK_EQ(landmarks_.f_Br.cols(), landmarks_.origin_Br.cols());
}

real_t Clam::evaluateError(
    const ClamState& state, HessianMatrix* H, GradientVector* g)
{
  CHECK_EQ(data_.size(), measurement_sigma_localization_.size());
  real_t chi2 = 0.0;

  const Transformation& T_Bc_Br = state.at<0>();
  const VectorX& inv_depth = state.at<1>();

  // ---------------------------------------------------------------------------
  // Localization

  // Loop over all cameras in rig.
  for (size_t i = 0; i < data_.size(); ++i)
  {
    const ClamFrameData& data = data_[i];
    real_t& measurement_sigma = measurement_sigma_localization_[i];

    // Continue if we have no landmarks to localize with.
    if(data.p_Br.cols() == 0)
    {
      VLOG(300) << "Cam " << i << " has no landmarks to localize.";
      continue;
    }

    // Transform points from reference coordinates to camera coordinates.
    const Transformation T_C_Br = data.T_C_B * T_Bc_Br; //! @todo(cfo): use inverse-depth coordinates!
    const Positions p_C = T_C_Br.transformVectorized(data.p_Br);

    // Normalize points to obtain estimated bearing vectors.
    Bearings f_est = p_C;
    normalizeBearings(f_est);

    // Compute difference between bearing vectors.
    Bearings f_err = f_est - data.f_C;
    const VectorX f_err_norm = f_err.colwise().norm();

    // At the first iteration, compute the scale of the error.
    if(iter_ == 0)
    {
      measurement_sigma = ScaleEstimator::compute(f_err_norm);
    }

    // Robust cost function.
    const VectorX weights =
        WeightFunction::weightVectorized(f_err_norm / measurement_sigma);

    // Whiten error.
    f_err /= measurement_sigma;

    if (H && g)
    {
      const Matrix3 R_C_Br = T_C_Br.getRotationMatrix();
      const int n = data.f_C.cols();
      Matrix36 G;
      G.block<3,3>(0,0) = I_3x3;
      for (int i = 0; i < n; ++i)
      {
        // Jacobian computation.
        G.block<3,3>(0,3) = - skewSymmetric(data.p_Br.col(i));
        Matrix3 J_normalization = dBearing_dLandmark(p_C.col(i));
        Matrix36 J = J_normalization * R_C_Br * G;

        // Whiten Jacobian.
        J /= measurement_sigma;

        // Compute Hessian and Gradient Vector.
        H->topLeftCorner<6,6>().noalias() += J.transpose() * J * weights(i);
        g->head<6>().noalias() -= J.transpose() * f_err.col(i) * weights(i);
      }
    }

    // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
    chi2 += 0.5 * weights.dot(f_err.colwise().squaredNorm());
  }

  // ---------------------------------------------------------------------------
  // Mapping

  //! @todo(cfo): This can be optimized a lot!
  for (size_t i = 0; i < data_.size(); ++i)
  {
    const ClamFrameData& data = data_[i];
    const Camera& cam = rig_.at(i);
    for (const std::pair<uint32_t, Keypoint>& m : data.landmark_measurements)
    {
      Matrix26 H1;
      Matrix21 H2;
      CHECK_LT(m.first, (uint32_t)landmarks_.f_Br.cols());
      CHECK_LT(m.first, (uint32_t)inv_depth.size());
      Vector2 err = reprojectionResidual(
            landmarks_.f_Br.col(m.first), landmarks_.origin_Br.col(m.first),
            cam, data.T_C_B, T_Bc_Br, inv_depth(m.first), m.second, &H1, &H2);

      // Robust cost function.
      const real_t weight = 1.0; //!< @todo(cfo)

      // Whiten error
      err /= measurement_sigma_mapping_;

      Matrix2X J(2, g->size());
      J.setZero();
      J.block<2,6>(0, 0) = H1;
      J.block<2,1>(0, 6 + m.first) = H2;

      // Whiten Jacobian.
      J /= measurement_sigma_mapping_;

      // Compute Hessian and Gradient Vector.
      H->noalias() += J.transpose() * J * weight;
      g->noalias() -= J.transpose() * err * weight;

      // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
      chi2 += 0.5 * weight * err.squaredNorm();
    }
  }

  // ---------------------------------------------------------------------------
  // Prior
  if (prior_weight_rot_ > 0.0f || prior_weight_pos_ > 0.0f)
  {
    applyPosePrior(
          T_Bc_Br, T_Bc_Br_prior_, prior_weight_rot_, prior_weight_pos_,
          H->block<6,6>(0,0), g->segment<6>(0));
  }

  return chi2;
}

} // namespace ze
