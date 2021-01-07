// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_common/landmark_utils.hpp>

#include <vector>

#include <ze/common/statistics.hpp>
#include <ze/vio_common/nframe.hpp>
#include <ze/vio_common/landmark_table.hpp>

namespace ze {

//------------------------------------------------------------------------------
int removeOldLandmarks(
    const uint32_t max_num_landmarks_to_keep,
    const uint32_t current_frame_index,
    LandmarkTable& landmarks)
{
  // Remove all landmarks that have not been seen in the last 3 frames.
  int num_active_landmarks = 0;
  int num_seeds = 0;
  int num_bad_points = 0;
  int num_terminated_persistent = 0;

  std::vector<std::pair<int, uint32_t>> count_last_seen_index;
  count_last_seen_index.reserve(landmarks.numLandmarks());

  for (uint32_t i = 0u, imax = landmarks.numLandmarks(); i < imax; ++i)
  {
    if (isLandmarkInactive(landmarks.typeAtSlot(i)))
    {
      continue;
    }

    if (landmarks.lastIterVisibleAtSlot(i) != current_frame_index
        && landmarks.projectionQualityAtSlot(i) < -10)
    {
      landmarks.typeAtSlot(i) = LandmarkType::Unknown;
      ++num_bad_points;
      continue;
    }

    ++num_active_landmarks;
    if (landmarks.typeAtSlot(i) == LandmarkType::Seed)
    {
      ++num_seeds;
    }

    count_last_seen_index.push_back(
          std::make_pair(current_frame_index - static_cast<int>(landmarks.lastIterVisibleAtSlot(i)),
                         i));
  }

  int num_landmarks_to_remove =
      std::max(0, num_active_landmarks - static_cast<int>(max_num_landmarks_to_keep));
  if (num_landmarks_to_remove > 0)
  {
    std::nth_element(
          count_last_seen_index.begin(),
          count_last_seen_index.begin() + num_landmarks_to_remove,
          count_last_seen_index.end(),
          [](const std::pair<int, uint32_t>& lhs, const std::pair<int, uint32_t>& rhs)
          { return lhs.first > rhs.first; });

    for (int i = 0; i < num_landmarks_to_remove; ++i)
    {
      landmarks.typeAtSlot(count_last_seen_index[i].second) = LandmarkType::Unknown;
    }
  }

  VLOG_IF(3, num_active_landmarks) << "Landmarks: " << num_active_landmarks << " active";
  VLOG_IF(3, num_seeds) << "Landmarks: " << num_seeds << " seeds";
  VLOG_IF(3, num_bad_points) << "Landmarks: " << num_bad_points << " removed bad ones";
  VLOG_IF(3, num_landmarks_to_remove) << "Landmarks: " << num_landmarks_to_remove << " removed old ones";
  VLOG_IF(3, num_terminated_persistent) << "Landmarks: " << num_terminated_persistent << " removed old ones";

  return num_landmarks_to_remove;
}

// -----------------------------------------------------------------------------
uint32_t addLandmarkObservations(const NFrame& nframe, LandmarkTable& landmarks)
{
  // For seeds it's important that the first frame is also the reference view.
  // A seed is identified if it's index in the frame is between seed_index_from
  // and seed_index_to. Therefore, we first loop over the nframe and add the
  // seed's observations and subsequently all the others.

  // Add seed observations.
  uint32_t num_lm_tot = 0;
  for (size_t frame_idx = 0; frame_idx < nframe.size(); ++frame_idx)
  {
    uint32_t num_lm = 0;
    const Frame& frame = nframe.at(frame_idx);
    for (uint16_t i = frame.seeds_index_from_; i < frame.seeds_index_to_; ++i)
    {
      const LandmarkHandle lm_h = frame.landmark_handles_[i];
      if (isValidLandmarkHandle(lm_h)
          && isLandmarkActive(landmarks.type(lm_h))) // can be inactive when multiple stereo configurations in rig.
      {
        DEBUG_CHECK_LT(frame.level_vec_(i), 10); // make sure it's initialized.
        DEBUG_CHECK(landmarks.isStored(lm_h));
        DEBUG_CHECK(landmarks.obs(lm_h).empty());
        landmarks.addObservation(
              lm_h, LandmarkObs(nframe.handle(), frame_idx, i, frame.f_vec_.col(i)));
        ++num_lm;
      }
    }
    VLOG(3) << "Added " << num_lm << " seed landmark observations to camera " << frame_idx;
    num_lm_tot += num_lm;
  }

  // Add all the remaining observations.
  for (size_t frame_idx = 0; frame_idx < nframe.size(); ++frame_idx)
  {
    uint32_t num_lm = 0;
    const Frame& frame = nframe.at(frame_idx);
    for (uint16_t i = 0; i < frame.num_features_; ++i)
    {
      const LandmarkHandle lm_h = frame.landmark_handles_[i];
      if (isValidLandmarkHandle(lm_h)
          && (i >= frame.seeds_index_to_        // For these indices, we have added
              || i < frame.seeds_index_from_))  // the observation above.
      {
        if (!isLandmarkActive(landmarks.type(lm_h)))
        {
          LOG(ERROR) << "Trying to add observation Landmark of type "
                     << landmarkTypeAsString(landmarks.type(lm_h));
          continue;
        }
        if (!landmarks.isStored(lm_h, true))
        {
          LOG(ERROR) << "Trying to add observation to Landmark that is not stored "
                     << "at slot " << lm_h.slot;
          continue;
        }
        DEBUG_CHECK_LT(frame.level_vec_(i), 10);  // make sure it's initialized.
        DEBUG_CHECK(landmarks.isStored(lm_h));
        DEBUG_CHECK(isLandmarkActive(landmarks.type(lm_h)));
        DEBUG_CHECK(!landmarks.obs(lm_h).empty());
        landmarks.addObservation(
              lm_h, LandmarkObs(nframe.handle(), frame_idx, i, frame.f_vec_.col(i)));
        ++num_lm;
      }
    }
    VLOG(3) << "Added " << num_lm << " landmark observations to camera " << frame_idx;
    num_lm_tot += num_lm;
  }
  return num_lm_tot;
}

// -----------------------------------------------------------------------------
uint32_t setTypeOfConvergedSeedsInNFrame(
    const NFrame& nframe,
    const LandmarkType type,
    LandmarkTable& landmarks)
{
  uint32_t num_upgraded_tot = 0u;
  for (const Frame::Ptr& frame : nframe)
  {
    uint32_t num_upgraded = 0u;
    for (uint32_t i = 0u; i < frame->num_features_; ++i)
    {
      LandmarkHandle lm_h = frame->landmark_handles_[i];
      if (isValidLandmarkHandle(lm_h)
          && landmarks.type(lm_h) == LandmarkType::SeedConverged)
      {
        landmarks.type(lm_h) = type;
        ++num_upgraded;
      }
    }
    num_upgraded_tot += num_upgraded;
    VLOG(5) << "Upgraded " << num_upgraded << " converged seeds to opportunistic";
  }
  return num_upgraded_tot;
}

// -----------------------------------------------------------------------------
void setLandmarksLastObservationInNFrame(
    const NFrame& nframe,
    LandmarkTable& landmarks)
{
  for (const Frame::Ptr& frame : nframe)
  {
    for (uint32_t i = 0u; i < frame->num_features_; ++i)
    {
      LandmarkHandle lm_h = frame->landmark_handles_[i];
      if (isValidLandmarkHandle(lm_h))
      {
        landmarks.lastIterVisible(lm_h) = nframe.seq();
        landmarks.track(lm_h).push_back(frame->px_vec_.col(i));
      }
    }
  }
}

} // namespace ze
