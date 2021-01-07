// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <ze/common/thread_pool.hpp>
#include <ze/common/transformation.hpp>
#include <ze/vio_common/landmark_handle.hpp>

namespace ze {

// fwd
class CameraRig;
class NFrame;
class LandmarkTable;
class StereoMatcher;

using IndexMatches = std::vector<std::pair<uint32_t, uint32_t>>;

//! Tracks features from frame to frame using Lucas-Kanade style tracking.
class FeatureTracker
{
public:
  struct ProjectionResult
  {
    LandmarkHandles successful_projections;
    LandmarkHandles failed_projections;
  };

  FeatureTracker() = delete;
  FeatureTracker(
      const CameraRig& rig,
      const real_t keypoint_margin,
      const real_t reprojection_error_threshold_px,
      const bool use_5pt_ransac,
      StereoMatcher& stereo_matcher,
      LandmarkTable& landmarks);

  //! @name Interface
  //! @{
  size_t trackFeaturesInNFrame(
      const Transformation& T_Bref_W,
      const Transformation& T_Bcur_W,
      NFrame& ref_nframe,
      NFrame& cur_nframe);

  std::pair<std::vector<real_t>, uint32_t> outlierRemoval(
      NFrame& nframe_ref,
      NFrame& nframe_cur,
      const Transformation& T_Bcur_Bref);

  real_t getMedianDisparity(
      NFrame& nframe_ref,
      NFrame& nframe_cur);
  //! @}

  void postprocessTrackingResults(
      const size_t frame_idx,
      const ProjectionResult& res);

  std::pair<uint32_t, IndexMatches> ransacRelativePoseOutlierRejection(
      uint32_t ref_idx,
      uint32_t cur_idx,
      NFrame& nframe_ref,
      NFrame& nframe_cur,
      std::vector<real_t>& disparities_sq,
      Transformation& T_cur_ref);

  void setUse5PtRansac(bool use_5pt_ransac);

private:
  LandmarkTable& landmarks_;
  const CameraRig& rig_;
  const TransformationVector T_C_B_;
  ThreadPool thread_pool_;
  StereoMatcher& stereo_matcher_;

  // Configs
  bool async_;
  bool use_5pt_ransac_;
  real_t reprojection_error_threshold_px_;
  real_t keypoint_margin_;
};

} // namespace ze
