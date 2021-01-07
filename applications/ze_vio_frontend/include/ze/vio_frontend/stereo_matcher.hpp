// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <ze/common/transformation.hpp>

namespace ze {

// fwd
class CameraRig;
class NFrame;
class LandmarkTable;

class StereoMatcher
{
public:
  StereoMatcher() = delete;
  StereoMatcher(
      const CameraRig& rig,
      const real_t keypoint_margin,
      const real_t reprojection_threshold_px,
      LandmarkTable& landmarks);

  //! @return returns inlier indices: { { ref_frame_idx, {inliner indices} }
  std::vector<std::pair<uint32_t, std::vector<uint32_t>>>
  matchStereoAndRejectOutliers(
      NFrame& nframe,
      const Transformation& T_B_W);

  uint32_t matchStereo(
      NFrame& nframe,
      const Transformation& T_B_W,
      const uint32_t ref_frame_idx,
      const uint32_t cur_frame_idx,
      const real_t inv_depth_ref_estimate,
      const real_t inv_depth_ref_min,
      const real_t inv_depth_ref_max);

  //! @return pair (inlier indices in reference frame, num outliers)
  std::pair<std::vector<uint32_t>, uint32_t> stereoOutlierRejection(
      NFrame& nframe,
      const uint32_t ref_idx = 0u,
      const uint32_t cur_idx = 1u);

  void updateSeedDepthAfterStereo(
      const Transformation& T_B_W,
      const NFrame& nframe);

private:
  LandmarkTable& landmarks_;
  const CameraRig& rig_;
  const TransformationVector T_C_B_;

  // System state
  uint32_t stereo_count_ = 0u; //!< Number of stereo matches, used to alternate reference frame.

  // Configs
  real_t keypoint_margin_;
  real_t reprojection_threshold_px_;
};

} // namespace ze
