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

#include <ze/common/transformation.hpp>
#include <ze/cameras/camera_utils.hpp>
#include <ze/cameras/camera_impl.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <ze/geometry/robust_cost.hpp>
#include <ze/geometry/lsq_solver.hpp>
#include <ze/geometry/lsq_state.hpp>

namespace ze {

using ClamState = State<Transformation, VectorX>;

struct ClamLandmarks
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Positions origin_Br; //! Br is body-frame of the reverence view.
  Bearings f_Br;
};

//! Data required by Clam per frame.
struct ClamFrameData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! @name Localization data
  //! @{
  //! Measurements: Bearing vectors.
  Bearings f_C;

  //! Landmark positions expressed in body frame of reference view.
  //! Each column corresponds to a bearing measurement.
  Positions p_Br;
  //! @}

  //! @name Mapping data
  //! @{
  //! Landmark measurements {ClamLandmarks-Index, Keypoint}.
  std::vector<std::pair<uint32_t, Keypoint>> landmark_measurements;
  //! @}

  //! Extrinsic transformation between camera and body (i.e., IMU) frame.
  Transformation T_C_B;
};

//! Coupled localization and mapping (Clam) as descripted in:
//! Jonathan Balzer, Stefano Soatto, "CLAM: Coupled Localization and Mapping
//! with Efficient Outlier Handling".
class Clam : public LeastSquaresSolver<ClamState, Clam>
{
public:
  using LeastSquaresSolver::HessianMatrix;
  using LeastSquaresSolver::GradientVector;
  using ScaleEstimator = MADScaleEstimator<real_t>;
  using WeightFunction = TukeyWeightFunction<real_t>;

  Clam(
      const ClamLandmarks& landmarks,
      const std::vector<ClamFrameData>& data,
      const CameraRig& rig,
      const Transformation& T_Bc_Br_prior,
      const real_t prior_weight_pos,
      const real_t prior_weight_rot);

  real_t evaluateError(
      const ClamState& state,
      HessianMatrix* H,
      GradientVector* g);

private:
  const ClamLandmarks& landmarks_;
  const std::vector<ClamFrameData>& data_;
  const CameraRig& rig_;
  std::vector<real_t> measurement_sigma_localization_;
  real_t measurement_sigma_mapping_ = 2.0;

  // Prior:
  const Transformation& T_Bc_Br_prior_; //!< Body-frame of (c)urrent and (r)eference view.
  real_t prior_weight_pos_;
  real_t prior_weight_rot_;
};

inline Vector2 reprojectionResidual(
    const Eigen::Ref<const Bearing>& f_Br, //!< Bearing vector in reference body frame (Br).
    const Eigen::Ref<const Position>& p_Br, //!< Reference camera center pos in Br.
    const Camera& cam,
    const Transformation& T_C_B,
    const Transformation& T_Bc_Br,
    const real_t inv_depth,
    const Eigen::Ref<const Keypoint>& px_measured,
    Matrix26* H1 = nullptr, //!< Jacobian dreprojectionResidual() / dT_Bc_Br
    Matrix21* H2 = nullptr  //!< Jacobian dreprojectionResidual() / dinv_depth
    )
{
  HomPosition p_Br_h;
  p_Br_h.head<3>() = f_Br + p_Br * inv_depth;
  p_Br_h(3) = inv_depth;
  const Transformation T_C_Br = T_C_B * T_Bc_Br;
  const HomPosition p_C_h = T_C_Br.transform4(p_Br_h);
  const Keypoint px_est = cam.project(p_C_h.head<3>());
  const Vector2 px_err = px_est - px_measured;

  if (H1 || H2)
  {
    // H1 = dPx / dT_Bc_Br
    Matrix23 J_proj = cam.dProject_dLandmark(p_C_h.head<3>());
    if (inv_depth < 0.0)
    {
      J_proj *= -1.0;
    }
    Matrix36 G;
    G.block<3,3>(0,0) = I_3x3 * inv_depth; // translation
    G.block<3,3>(0,3) = -skewSymmetric(p_Br_h.head<3>()); // rotation
    *H1 = J_proj * T_C_Br.getRotationMatrix() * G;

    // H2 = dPx / dinv_depth
    *H2 = J_proj * (T_C_Br.getRotation().rotate(p_Br) + T_C_Br.getPosition());
  }

  return px_err;
}

} // namespace ze
