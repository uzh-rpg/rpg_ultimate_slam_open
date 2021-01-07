// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_frontend/feature_initializer.hpp>

#include <imp/correspondence/sparse_stereo.hpp>
#include <imp/features/feature_descriptor.hpp>
#include <imp/features/feature_detector.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <ze/common/combinatorics.hpp>
#include <ze/common/logging.hpp>
#include <ze/common/statistics.hpp>
#include <ze/common/types.hpp>
#include <ze/geometry/ransac_relative_pose.hpp>
#include <ze/geometry/triangulation.hpp>
#include <ze/vio_common/frame.hpp>
#include <ze/vio_common/landmark_table.hpp>
#include <ze/vio_common/nframe.hpp>

namespace ze {

FeatureInitializer::FeatureInitializer(
    const CameraRig& rig,
    const real_t keypoint_margin,
    LandmarkTable& landmarks)
  : landmarks_(landmarks)
  , rig_(rig)
  , T_C_B_(rig_.T_C_B_vec())
  , keypoint_margin_(keypoint_margin)
  , detectors_(loadVioDetectorsFromGflags(rig_))
  , descriptors_(loadVioDescriptorExtractorFromGflags(rig_))
  , async_(rig_.size() > 1u)
{
  if (async_)
  {
    thread_pool_.startThreads(rig_.size());
  }
}

// -----------------------------------------------------------------------------
uint32_t detectFeatures(
    Frame& frame,
    FeatureDetectionHandler& detector,
    const Camera& cam,
    bool mask_existing_features)
{
  KeypointsWrapper features = frame.getKeypointsWrapper();
  const uint32_t num_before = features.num_detected;
  VLOG(3) << "Feature detection: Have already " << num_before
          << " features in camera";
  detector.detect(*frame.pyr_, features, mask_existing_features);
  uint32_t num_detected = features.num_detected - num_before;

  // Compute normalized bearing vectors corresponding to detected features.
  Bearings f = cam.backProjectVectorized(
        features.px.middleCols(num_before, num_detected));
  frame.f_vec_.middleCols(num_before, num_detected) = f;
  return num_detected;
}

// -----------------------------------------------------------------------------
uint32_t FeatureInitializer::detectAndInitializeNewFeatures(
    NFrame& nframe, const std::vector<uint32_t> frame_idx_vec)
{
  std::vector<uint32_t> num_features(rig_.size(), 0u);
  if (frame_idx_vec.size() > 1u && async_)
  {
    // Assign tasks to threads in pool for feature detection.
    std::vector<std::future<uint32_t>> results;
    for (uint32_t frame_idx : frame_idx_vec)
    {
      results.emplace_back(thread_pool_.enqueue(
                             detectFeatures,
                             std::ref(nframe.at(frame_idx)),
                             std::ref(*detectors_[frame_idx]),
                             std::cref(rig_.at(frame_idx)),
                             mask_existing_features_));
    }

    // Make sure both threads are finished.
    uint32_t i = 0u;
    for (uint32_t frame_idx : frame_idx_vec)
    {
      DEBUG_CHECK_LT(frame_idx, num_features.size());
      DEBUG_CHECK_LT(i, results.size());

      num_features[frame_idx] = results[i].get();
      VLOG(3) << "Detected " << num_features[frame_idx]
              << " new features in camera " << frame_idx;
      ++i;
    }
  }
  else
  {
    for (uint32_t frame_idx : frame_idx_vec)
    {
      DEBUG_CHECK_LT(frame_idx, num_features.size());
      DEBUG_CHECK_LT(frame_idx, detectors_.size());

      num_features[frame_idx] =
          detectFeatures(nframe.at(frame_idx), *detectors_[frame_idx],
                         rig_.at(frame_idx), mask_existing_features_);
      VLOG(3) << "Detected " << num_features[frame_idx]
              << " new features in camera " << frame_idx;
    }
  }
  uint32_t num_features_total =
      std::accumulate(num_features.begin(), num_features.end(), 0);

  // Assign new landmark-id for each feature and initialize seeds.
  LandmarkHandles handles =
      landmarks_.getNewLandmarkHandles(num_features_total, nframe.seq());
  uint32_t n = 0u;
  for (uint32_t frame_idx : frame_idx_vec)
  {
    Frame& frame = nframe.at(frame_idx);
    DEBUG_CHECK_GT(frame.min_depth_, 0.0);
    DEBUG_CHECK_GT(frame.max_depth_, 0.0);
    DEBUG_CHECK_GT(frame.median_depth_, 0.0);

    frame.seeds_index_from_ = frame.num_features_ - num_features.at(frame_idx);
    frame.seeds_index_to_ = frame.num_features_;
    frame.seeds_mean_range_ = real_t{2} / frame.min_depth_;
    Seed seed_prototype;
    seed_prototype << real_t{1} / frame.median_depth_,
                      frame.seeds_mean_range_, 10, 10;
    for (uint32_t i = frame.seeds_index_from_; i < frame.seeds_index_to_; ++i)
    {
      DEBUG_CHECK_LT(n, handles.size());
      frame.landmark_handles_[i] = handles[n];
      landmarks_.seed(handles[n]) = seed_prototype;
      ++n;
    }
  }

  return num_features_total;
}

// -----------------------------------------------------------------------------
void FeatureInitializer::extractFeatureDescriptors(NFrame& nframe)
{
  CHECK(descriptors_.size() == nframe.size()) << "There should be one"
                                                 "descriptor for each frame.";
  if (async_)
  {
    // Assign tasks to threads in pool for descriptor extraction.
    std::vector<std::future<bool>> results;
    for (uint32_t frame_idx = 0u; frame_idx < nframe.size(); ++frame_idx)
    {
      results.emplace_back(thread_pool_.enqueue([frame_idx, &nframe, this]()
      {
        VLOG(40) << "Extract descriptors for frame " << frame_idx;
        Frame& frame = nframe.at(frame_idx);
        KeypointsWrapper features = frame.getKeypointsWrapper();
        descriptors_.at(frame_idx)->extract(*frame.pyr_, features);
        return true;
      }));
    }

    // Make sure both threads are finished.
    for (uint32_t frame_idx = 0u; frame_idx < nframe.size(); ++frame_idx)
    {
      CHECK(results[frame_idx].get());
    }
  }
  else
  {
    for (uint32_t frame_idx = 0u; frame_idx < nframe.size(); ++frame_idx)
    {
      VLOG(40) << "Extract descriptors for frame " << frame_idx;
      Frame& frame = nframe.at(frame_idx);
      KeypointsWrapper features = frame.getKeypointsWrapper();
      descriptors_.at(frame_idx)->extract(*frame.pyr_, features);
    }
  }
}

// -----------------------------------------------------------------------------
void FeatureInitializer::setOccupancyGrid(const OccupancyGrid2dVector& grid)
{
  DEBUG_CHECK_EQ(grid.size(), detectors_.size());

  for (size_t i = 0u; i < grid.size(); ++i)
  {
    detectors_[i]->setGrid(grid[i]);
  }
}

} // namespace ze
