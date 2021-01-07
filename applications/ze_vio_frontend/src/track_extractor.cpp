// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_frontend/track_extractor.hpp>

#include <ze/vio_common/frame.hpp>
#include <ze/vio_common/nframe.hpp>
#include <ze/vio_common/nframe_table.hpp>
#include <ze/vio_common/landmark_table.hpp>
#include <ze/vio_common/landmark_triangulation.hpp>
#include <ze/vio_common/motion_type.hpp>
#include <ze/vio_frontend/frontend_gflags.hpp>

DEFINE_int32(vio_max_num_tracks_per_update, 50,
             "Max number of landmarks to add to back-end optimization per keyframe.");
DEFINE_int32(vio_num_obs_for_persistent, 10,
             "Number of observations in feature-track to classify landmark as persistent.");
DEFINE_double(vio_min_parallax_deg, 0.5,
              "Minimum parallax threshold to use a landmark in the back-end optimization.");
DEFINE_bool(vio_favor_equal_number_of_tracks_per_frame, false,
            "Favor a n equal number of new persistent tracks in every frame.");

namespace ze {

void selectLandmarksForBackend(
    const VioMotionType motion_type,
    const TransformationVector& T_C_B,
    const NFrameTable& states,
    NFrame& nframe_kp1,
    LandmarkTable& landmarks,
    std::vector<LandmarkHandle>& lm_opportunistic,
    std::vector<LandmarkHandle>& lm_persistent_new,
    std::vector<LandmarkHandle>& lm_persistent_continued)
{
  const NFrameHandle nframe_handle_kp1 = nframe_kp1.handle();
  const size_t max_num_obs = FLAGS_vio_num_obs_for_persistent + 1; // num keyframes in SWE.
  std::vector<std::pair<LandmarkHandle, int>> opportunistic;
  std::vector<LandmarkHandle> lm_opportunistic_from_aborted;
  std::vector<std::pair<LandmarkHandle, int>> persistent_new;
  std::vector<std::pair<LandmarkHandle, int>> persistent_continued;

  // ---------------------------------------------------------------------------
  // Loop over all landmarks and decide what to do with them.
  int num_opportunistic_candidates = 0;
  int num_opportunistic_candidates2 = 0;
  int num_opportunistic_candidates3 = 0;
  int num_persistent_candidates = 0;
  int num_triangulation_fails = 0;
  int num_opportunistic_too_few_obs = 0;
  for (uint32_t i = 0u; i < landmarks.numLandmarks(); ++i)
  {
    const LandmarkType lm_type = landmarks.typeAtSlot(i);
    if (!isLandmarkActive(lm_type))
    {
      continue;
    }

    const NFrameHandle obs_last_nframe = landmarks.lastObsNFrameHandleAtSlot(i);
    switch (lm_type)
    {
      case LandmarkType::Persistent:
      {
        ++num_persistent_candidates;
        if (obs_last_nframe == nframe_handle_kp1)
        {
          LandmarkHandle lm_h = landmarks.getHandleAtSlot(i);
          persistent_continued.push_back(
                std::make_pair(lm_h, landmarks.projectionQuality(lm_h)));
        }
        else
        {
          // This landmark has not been observed anymore in the last frame.
          landmarks.typeAtSlot(i) = LandmarkType::PersistentTerminated;
        }
        break;
      }
      case LandmarkType::Seed:
      case LandmarkType::SeedConverged:
      case LandmarkType::Opportunistic:
      {
        ++num_opportunistic_candidates;
        uint32_t num_nframe_obs = landmarks.numNFrameObsAtSlot(i);

        if (obs_last_nframe != nframe_handle_kp1)
        {
          // This landmark has not been observed anymore in the last frame.
          landmarks.typeAtSlot(i) = LandmarkType::OpportunisticTerminated;
        }

        if (FLAGS_vio_delayed_nframe_processing)
        {
          if (obs_last_nframe != nframe_handle_kp1
             && num_nframe_obs >= 3u) //! @todo: can reduce to two, in theory.
          {
            ++num_opportunistic_candidates2;
            // If this landmark has not been observed anymore in last frame, add
            // landmark to opportunistic landmark observations. We need to ensure
            // that frame has been observed by at least two nframes.
            if (!insertIfGoodQualityLandmark(landmarks, i, opportunistic,
                                             T_C_B, states, &nframe_handle_kp1))
            {
              ++num_triangulation_fails;
            }
          }
          else if (num_nframe_obs >= max_num_obs)
          {
            // This landmark is still being observed and it has already more than
            // 10 observations. Therefore, convert track to a persistent track.
            if (!insertIfGoodQualityLandmark(landmarks, i, persistent_new,
                                             T_C_B, states, &nframe_handle_kp1))
            {
              ++num_triangulation_fails;
            }
          }
          else if (num_nframe_obs < 3u)
          {
            ++num_opportunistic_too_few_obs;
          }
          else
          {
            ++num_opportunistic_candidates3;
          }
        }
        else if (obs_last_nframe == nframe_handle_kp1
                 && num_nframe_obs >= 3u) //! @todo: can reduce to two, in theory.
        {
          // If this landmark has not been observed anymore in last frame, add
          // landmark to opportunistic landmark observations. We need to ensure
          // that frame has been observed by at least two nframes.
          if (!insertIfGoodQualityLandmark(landmarks, i, persistent_new, T_C_B, states))
          {
            ++num_triangulation_fails;
          }
        }
        else if (obs_last_nframe == nframe_handle_kp1
                 && num_nframe_obs < 3u) //! @todo: can reduce to two, in theory.
        {
          ++num_opportunistic_too_few_obs;
        }
        break;
      }
      default:
        LOG(FATAL) << "Landmark Type unknown.";
        break;
    }
  }
  VLOG(3) << "Num opportunistic candidates = " << num_opportunistic_candidates;
  VLOG(3) << "Num opportunistic candidates2 = " << num_opportunistic_candidates2;
  VLOG(3) << "Num opportunistic candidates3 = " << num_opportunistic_candidates3;
  VLOG(3) << "Num persistent candidates = " << num_persistent_candidates;
  VLOG(3) << "Num opportunistic with too few obs = " << num_opportunistic_too_few_obs;

  // ---------------------------------------------------------------------------
  // Abort active opportunistic tracks if there are not enough naturally
  // terminated tracks. Don't do this if motion is rotation-only because in this
  // case, we will add zero-velocity contraints and landmarks are not very important.
  const size_t min_number_of_tracks = FLAGS_vio_max_num_tracks_per_update / 2;
  const size_t num_new_tracks =
      opportunistic.size() + persistent_new.size() + persistent_continued.size();
  VLOG(10) << "Num available tracks total: "<< num_new_tracks << " ("
           << "opportunistic = " << opportunistic.size() << ", "
           << "persistent_new = " << persistent_new.size() << ", "
           << "persistent_cont = " << persistent_continued.size() << ")";
  if (FLAGS_vio_delayed_nframe_processing
      && num_new_tracks < min_number_of_tracks
      && motion_type == VioMotionType::GeneralMotion)
  {
    // Loop through all opportunistic tracks and abort those that were not aborted
    // above because they are still being tracked.
    const int num_tracks_to_abort = min_number_of_tracks - num_new_tracks;
    int num_abort_candidates = 0;
    VLOG(10) << "Num tracks to abort = " << num_tracks_to_abort;
    std::vector<std::pair<LandmarkHandle, int>> aborted;
    for (uint32_t i = 0u; i < landmarks.numLandmarks(); ++i)
    {
      if ((landmarks.typeAtSlot(i) == LandmarkType::Opportunistic
           || landmarks.typeAtSlot(i) == LandmarkType::Seed
           || landmarks.typeAtSlot(i) == LandmarkType::SeedConverged)
         && landmarks.lastObsNFrameHandleAtSlot(i) == nframe_handle_kp1 // already in opportunistic
         && landmarks.numNFrameObsAtSlot(i) < max_num_obs // already in persistent_new
         && landmarks.numNFrameObsAtSlot(i) >= 3u)
      {
        ++num_abort_candidates;
        if (!insertIfGoodQualityLandmark(landmarks, i, aborted,
                                         T_C_B, states, &nframe_handle_kp1))
        {
          ++num_triangulation_fails;
        }
      }
    }
    VLOG(10) << "Num abort candidates = " << num_abort_candidates;

    //! @todo: Why don't we make persistent tracks out of them?
    //!
    // Select best tracks to abort:
    std::tie(lm_opportunistic_from_aborted, std::ignore) =
        splitNBestLandmarks(aborted, num_tracks_to_abort);
    LOG(ERROR) << "Aborted " << lm_opportunistic_from_aborted.size()
               << " opportunistic tracks because not enough were available.";

    // We will split the aborted tracks and restart them with the current nframe.
    splitAndInitializeNewLandmarks(lm_opportunistic_from_aborted, landmarks, nframe_kp1);
  }
  VLOG(10) << "Triangulation fails = " << num_triangulation_fails;

  // ---------------------------------------------------------------------------
  // Initialize new landmark observations for opportunistic and new persistent tracks

  // Limit the number of persistent continued tracks.
  size_t max_opportunistic =
      std::min(static_cast<size_t>(0.5 * FLAGS_vio_max_num_tracks_per_update),
               opportunistic.size() + lm_opportunistic_from_aborted.size());
  size_t max_persistent = FLAGS_vio_max_num_tracks_per_update - max_opportunistic;
  std::vector<LandmarkHandle> lm_persistent_continued_aborted;
  std::tie(lm_persistent_continued, lm_persistent_continued_aborted) =
      splitNBestLandmarks(persistent_continued, max_persistent);
  VLOG(10) << "Max opportunistic = " << max_opportunistic << ", "
           << "max persistent = " << max_persistent;
  for (const LandmarkHandle h : lm_persistent_continued_aborted)
  {
    landmarks.type(h) = LandmarkType::PersistentTerminated;
  }
  LOG_IF(WARNING, lm_persistent_continued_aborted.size() > 0u)
      << "Aborted " << lm_persistent_continued_aborted.size()
      << " persistent continued tracks. Because too many.";

  //! @todo: It might not be such a good idea to reinitiaize these because they
  //!        create clusters of 3d points that are actually the same feature...?
  /*
  lm_split_and_reinitialize.insert(lm_split_and_reinitialize.end(),
                                   lm_persistent_continued_aborted.begin(),
                                   lm_persistent_continued_aborted.end());
  */
  max_persistent -= lm_persistent_continued.size();
  VLOG(10) << "Continued persistent landmarks: " << lm_persistent_continued.size();

  // If we have too many new persistent tracks, sort them and keep the best ones.
  // The ones that are not selected remain Opportunistic and may be selected in
  // the next iteration.

  // Enforce equal choice from all frames.
  if (FLAGS_vio_favor_equal_number_of_tracks_per_frame)
  {
    // Favors an equal number of new persistent tracks in every frame. The total
    // number of initialized persistent tracks might be lower than the total
    // number of candidates.
    std::vector<std::vector<std::pair<LandmarkHandle, int>>> persistent_new_split(nframe_kp1.size());
    for (std::vector<std::pair<LandmarkHandle, int>>::iterator lm = persistent_new.begin();
         lm != persistent_new.end(); ++lm)
    {
      int cam_idx = static_cast<int>(landmarks.lastObs(lm->first).frame_idx_);
      persistent_new_split[cam_idx].push_back(*lm);
    }

    std::vector<LandmarkHandle> lm_persistent_new_aborted_set;
    for (std::vector<std::pair<LandmarkHandle, int>>& persistent_new_set : persistent_new_split)
    {
      std::vector<LandmarkHandle> lm_persistent_new_aborted;
      std::vector<LandmarkHandle> lm_persistent_new_set;

      std::tie(lm_persistent_new_set, lm_persistent_new_aborted_set) =
          splitNBestLandmarks(persistent_new_set, max_persistent / persistent_new_split.size());

      lm_persistent_new_aborted.insert(lm_persistent_new_aborted.end(),
                                       lm_persistent_new_aborted_set.begin(),
                                       lm_persistent_new_aborted_set.end());
      lm_persistent_new.insert(lm_persistent_new.end(),
                               lm_persistent_new_set.begin(),
                               lm_persistent_new_set.end());
    }
  }
  else
  {
    std::vector<LandmarkHandle> lm_persistent_new_aborted;
    std::tie(lm_persistent_new, lm_persistent_new_aborted) =
        splitNBestLandmarks(persistent_new, max_persistent);
    VLOG(20) << "New persistent landmarks aborted: " << lm_persistent_new_aborted.size();
  }

  for (const LandmarkHandle h : lm_persistent_new)
  {
    landmarks.type(h) = LandmarkType::Persistent;
  }
  max_persistent -= lm_persistent_new.size();
  VLOG(10) << "New persistent landmarks: " << lm_persistent_new.size();

  // Now, fill the remaining track budget with opportunistic tracks.
  max_opportunistic += max_persistent;
  std::tie(lm_opportunistic, std::ignore) = splitNBestLandmarks(opportunistic, max_opportunistic);
  lm_opportunistic.insert(lm_opportunistic.end(), lm_opportunistic_from_aborted.begin(),
                          lm_opportunistic_from_aborted.end());
  for (const LandmarkHandle h : lm_opportunistic)
  {
    landmarks.type(h) = LandmarkType::OpportunisticAbsorbed;
  }
  VLOG(10) << "New opportunistic landmarks: " << lm_opportunistic.size();
  LOG_IF(WARNING, lm_persistent_continued_aborted.size() > 0u)
      << "Aborted " << lm_persistent_continued_aborted.size()
      << " persistent continued because have too many.";

  // For the forcefully aborted track, we split the tracks and initialize new ones:
  //splitAndInitializeNewLandmarks(lm_split_and_reinitialize, landmarks, nframe_kp1);
}

//------------------------------------------------------------------------------
std::pair<std::vector<LandmarkHandle>, std::vector<LandmarkHandle>>
splitNBestLandmarks(std::vector<std::pair<LandmarkHandle, int>>& weighted_ids, const size_t N)
{
  std::vector<LandmarkHandle> best_ids;
  std::vector<LandmarkHandle> worst_ids;
  if (weighted_ids.size() > N)
  {
    std::nth_element(
          weighted_ids.begin(), weighted_ids.begin() + N, weighted_ids.end(),
          [](const std::pair<LandmarkHandle, int>& lhs, const std::pair<LandmarkHandle, int>& rhs)
          { return lhs.second > rhs.second; });
    std::transform(
          weighted_ids.begin(), weighted_ids.begin() + N,
          std::back_inserter(best_ids),
          [](const std::pair<LandmarkHandle, int>& it) { return it.first; });
    std::transform(
          weighted_ids.begin() + N, weighted_ids.end(),
          std::back_inserter(worst_ids),
          [](const std::pair<LandmarkHandle, int>& it) { return it.first; });
  }
  else if (!weighted_ids.empty())
  {
    std::transform(
          weighted_ids.begin(), weighted_ids.end(),
          std::back_inserter(best_ids),
          [](const std::pair<LandmarkHandle, int>& it) { return it.first; });
  }
  return std::make_pair(best_ids, worst_ids);
}

//------------------------------------------------------------------------------
bool insertIfGoodQualityLandmark(
    LandmarkTable& landmarks,
    const uint32_t slot,
    std::vector<std::pair<LandmarkHandle, int>>& container,
    const TransformationVector& T_C_B,
    const NFrameTable& states,
    const NFrameHandle* ignore_nframe)
{
  //! @todo: Do these checks afterwards and only on the most promising ones.
  //! @todo: Should we check the reprojection error after triangulation?
  //!
  Position p_W;
  TriangulationResult res;
  std::tie(p_W, res) = triangulateLandmark(
        states, T_C_B, landmarks.obsAtSlot(slot), FLAGS_vio_min_depth,
        FLAGS_vio_max_depth, ignore_nframe);
  if (res != TriangulationResult::Success)
  {
    return false;
  }

  landmarks.p_W_atSlot(slot) = p_W;
  if (getLandmarkParallaxDegrees(
        states, T_C_B, landmarks.obsAtSlot(slot), p_W,
        ignore_nframe,
        &landmarks.anchorObsAtSlot(slot).frame_idx_)
     < FLAGS_vio_min_parallax_deg)
  {
    return false;
  }
  LandmarkHandle lm_h = landmarks.getHandleAtSlot(slot);
  container.push_back(std::make_pair(lm_h, landmarks.projectionQuality(lm_h)));
  return true;
}

//------------------------------------------------------------------------------
void splitAndInitializeNewLandmarks(
    const std::vector<LandmarkHandle>& lm_handles,
    LandmarkTable& landmarks,
    NFrame& nframe)
{
  LandmarkHandles lm_handles_new =
      landmarks.getNewLandmarkHandles(lm_handles.size(), 0u); //! @todo: *** set iter count ***
  const NFrameHandle nframe_handle_kp1 = nframe.handle();
  for (size_t i = 0u; i < lm_handles.size(); ++i)
  {
    LandmarkHandle lm_old_h = lm_handles[i];
    CHECK_EQ(landmarks.lastObsNFrameHandle(lm_old_h), nframe_handle_kp1);

    LandmarkObsVec obs = landmarks.obs(lm_old_h);
    LandmarkObs last_obs = obs.back();
    LandmarkHandle lm_new_h = lm_handles_new[i];
    landmarks.addObservation(lm_new_h, last_obs);
    landmarks.p_W(lm_new_h) = landmarks.p_W(lm_old_h);

    nframe.at(last_obs.frame_idx_).landmark_handles_[last_obs.keypoint_idx_] = lm_new_h;

    // Second observation?
    //! @todo: deal with rigs that have more than two cameras.
    auto obs2 = ++obs.rbegin();
    if (obs2->nframe_handle_ == nframe_handle_kp1)
    {
      landmarks.addObservation(lm_new_h, *obs2);
      nframe.at(obs2->frame_idx_).landmark_handles_[obs2->keypoint_idx_] = lm_new_h;
    }
  }
}

} // namespace ze
