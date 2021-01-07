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
#include <ze/common/manifold.hpp>
#include <ze/cameras/camera_utils.hpp>
#include <ze/cameras/camera_impl.hpp>
#include <ze/geometry/line.hpp>
#include <ze/geometry/robust_cost.hpp>
#include <ze/geometry/lsq_solver.hpp>

namespace ze {

enum class PoseOptimizerResidualType
{
  Bearing,
  UnitPlane,
  UnitPlaneEdgelet,
  Line
};

//! Data required by the pose-optimizer per frame.
struct PoseOptimizerFrameData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PoseOptimizerResidualType type = PoseOptimizerResidualType::UnitPlane;

  uint32_t camera_idx = 0u;

  //! Measurements: Bearing vectors corresponding to keypoints.
  Bearings f;

  //! Gradients: Used only for edgelets.
  Gradients grad;

  //! At which level was the keypoint extracted.
  VectorX scale;

  //! Line measurements. The normalized plane normals through the camera and
  //! the measured endpoints.
  LineMeasurements line_measurements_C;

  //! Measurements bookkeeping: Corresponding indices. (Not used by the actual algorithm).
  KeypointIndices kp_idx;

  //! Landmark positions. Each column corresponds to a bearing measurement.
  //! @todo(cfo): Use inverse depth parametrization or homogeneous points.
  Positions p_W;

  //! Line. Each entry corresponds to a line measurement.
  Lines lines_W;

  //! Extrinsic transformation between camera and body (i.e., imu) frame.
  Transformation T_C_B;

  //! @name Internal buffer
  //! @{
  //! Measurements: Projected on unit-plane. (computed internally).
  Keypoints uv;

  real_t measurement_sigma;
  //! @}
};
using PoseOptimizerFrameDataVec = std::vector<PoseOptimizerFrameData>;

//! Optimizes body pose by minimizing difference between bearing vectors.
class PoseOptimizer :
    public LeastSquaresSolver<Transformation, PoseOptimizer>
{
public:
  using LeastSquaresSolver::HessianMatrix;
  using LeastSquaresSolver::GradientVector;
  using ScaleEstimator = MADScaleEstimator<real_t>;
  using WeightFunction = TukeyWeightFunction<real_t>;

  PoseOptimizer(
      const LeastSquaresSolverOptions& options,
      std::vector<PoseOptimizerFrameData>& data);

  PoseOptimizer(
      const LeastSquaresSolverOptions& options,
      std::vector<PoseOptimizerFrameData>& data,
      const Transformation& T_B_W_prior,
      const real_t prior_weight_pos,
      const real_t prior_weight_rot);

  static LeastSquaresSolverOptions getDefaultSolverOptions();

  void setPrior(
      const Transformation& T_B_W_prior,
      const real_t prior_weight_pos,
      const real_t prior_weight_rot);

  real_t evaluateError(
      const Transformation& T_B_W,
      HessianMatrix* H,
      GradientVector* g);

  void update(
      const Transformation& state,
      const UpdateVector& dx,
      Transformation& new_state)
  {
    new_state = traits<Transformation>::retract(state, dx);
    new_state.getRotation().normalize();
  }

private:
  //! Checks whether given data is valid. Throws if not.
  void checkData() const;

  std::vector<PoseOptimizerFrameData>& data_;

  //! @name Prior
  //! @{
  Transformation T_B_W_prior_;
  real_t prior_weight_pos_ {0.0};
  real_t prior_weight_rot_ {0.0};
  //! @}
};

//! Returns sum of chi2 errors (weighted and whitened errors) and
//! a vector of withened errors for each error term (used for outlier removal).
std::pair<real_t, VectorX> evaluateBearingErrors(
    const Transformation& T_B_W,
    const bool compute_measurement_sigma,
    PoseOptimizerFrameData& data,
    PoseOptimizer::HessianMatrix* H,
    PoseOptimizer::GradientVector* g);

//! Returns sum of chi2 errors (weighted and whitened errors) and
//! a vector of withened errors for each error term (used for outlier removal).
std::pair<real_t, VectorX> evaluateUnitPlaneErrors(
    const Transformation& T_B_W,
    const bool compute_measurement_sigma,
    PoseOptimizerFrameData& data,
    PoseOptimizer::HessianMatrix* H,
    PoseOptimizer::GradientVector* g);

std::pair<real_t, VectorX> evaluateLineErrors(
    const Transformation& T_B_W,
    const bool compute_measurement_sigma,
    PoseOptimizerFrameData& data,
    PoseOptimizer::HessianMatrix* H,
    PoseOptimizer::GradientVector* g);

std::vector<KeypointIndex> getOutlierIndices(
    PoseOptimizerFrameData& data,
    const Camera& cam,
    const Transformation& T_B_W,
    const real_t pixel_threshold);

/*!
 * @brief Jacobian of bearing vector w.r.t. landmark in camera coordinates.
 *
 * f = p / norm(p) = p / sqrt(x^2 + y^2 + z^2), with p = [x, y, z].
 *
 *                                       | y^2 + z^2, -xy, -xz |
 * df/dp = 1 / (x^2 + y^2 + z^2)^(3/2) * | -xy, x^2 + z^2, -yz |
 *                                       | -xz, -yz, x^2 + z^2 |.
 */
inline Matrix3 dBearing_dLandmark(const Eigen::Ref<const Position>& p_C)
{
  const real_t x2 = p_C(0) * p_C(0);
  const real_t y2 = p_C(1) * p_C(1);
  const real_t z2 = p_C(2) * p_C(2);
  const real_t xy = p_C(0) * p_C(1);
  const real_t xz = p_C(0) * p_C(2);
  const real_t yz = p_C(1) * p_C(2);
  const real_t x2_y2_z2 = x2 + y2 + z2;
  Matrix3 J;
  J << y2 + z2, -xy, -xz,
       -xy, x2 + z2, -yz,
       -xz, -yz, x2 + y2;
  J /= (x2_y2_z2 * std::sqrt(x2_y2_z2));
  return J;
}

/*!
 * @brief Jacobian of unit-plane coordinates uv w.r.t. landmark in camera coordinates.
 */
inline Matrix23 dUv_dLandmark(const Eigen::Ref<const Position>& p_C)
{
  const real_t z_sq = p_C(2) * p_C(2);
  const real_t z_inv = real_t{1.0} / p_C(2);
  Matrix23 J;
  J << z_inv, 0.0, -p_C(0) / z_sq,
       0.0, z_inv, -p_C(1) / z_sq;
  return J;
}

} // namespace ze
