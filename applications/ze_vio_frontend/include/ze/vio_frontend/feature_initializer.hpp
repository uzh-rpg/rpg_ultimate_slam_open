// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <memory>

#include <imp/features/occupancy_grid_2d.hpp>
#include <ze/common/thread_pool.hpp>
#include <ze/common/transformation.hpp>

namespace ze {

// fwd
class AbstractDescriptorExtractor;
class CameraRig;
class FeatureDetectionHandler;
class LandmarkTable;
class NFrame;

//! Tracks features from frame to frame using Lucas-Kanade style tracking.
class FeatureInitializer
{
public:
  FeatureInitializer() = delete;
  FeatureInitializer(
      const CameraRig& rig,
      const real_t keypoint_margin,
      LandmarkTable& landmarks);

  //! @return Returns num detected features
  uint32_t detectAndInitializeNewFeatures(
      NFrame& nframe,
      const std::vector<uint32_t> frame_idx_vec //!< Frames for which to extract features.
      );

  void extractFeatureDescriptors(NFrame& nframe);

  inline void setMaskExistingFeatures(bool mask_existing_features)
  {
     mask_existing_features_ = mask_existing_features;
  }

  void setOccupancyGrid(const OccupancyGrid2dVector& grid);

private:
  LandmarkTable& landmarks_;
  const CameraRig& rig_;
  const TransformationVector T_C_B_;
  real_t keypoint_margin_;
  ThreadPool thread_pool_;
  std::vector<std::shared_ptr<FeatureDetectionHandler>> detectors_;
  std::vector<std::shared_ptr<AbstractDescriptorExtractor>> descriptors_;

  // Configs
  bool async_ = false; //!< Determined whether rig has more than one camera.
  bool mask_existing_features_ = true;
};

} // namespace ze
