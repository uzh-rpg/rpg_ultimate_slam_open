// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_frontend/landmarks_reprojector.hpp>

#include <gflags/gflags.h>

#include <imp/correspondence/epipolar_matcher.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <ze/cameras/camera_utils.hpp>
#include <ze/common/combinatorics.hpp>
#include <ze/common/logging.hpp>
#include <ze/common/statistics.hpp>
#include <ze/common/types.hpp>
#include <ze/geometry/ransac_relative_pose.hpp>
#include <ze/geometry/triangulation.hpp>
#include <ze/vio_common/frame.hpp>
#include <ze/vio_common/landmark_table.hpp>
#include <ze/vio_common/nframe.hpp>
#include <ze/vio_common/nframe_table.hpp>
#include <ze/vio_frontend/frontend_gflags.hpp>

DEFINE_bool(vio_reprojector_pyr_alignment, true,
            "Use a pyramidal alignment in the reprojector if the alignment"
            " of a small patch fails to converge.");
DEFINE_int32(vio_reprojector_min_quality_to_project, 2,
             "Minimum quality of a seed to track it in the reprojector. "
             "Quality is defined as the number of successful projections minus "
             "the number of failed projections.");
DEFINE_bool(vio_reprojector_limit_num_features, true,
            "Limit number of projected points in reprojector.");

namespace ze {

LandmarksReprojector::LandmarksReprojector(
    const CameraRig& rig,
    const real_t keypoint_margin,
    const uint32_t grid_size,
    LandmarkTable& landmarks,
    NFrameTable& states)
  : matcher_vec_(rig.size())
  , landmarks_(landmarks)
  , states_(states)
  , rig_(rig)
  , T_C_B_(rig.T_C_B_vec())
  , keypoint_margin_(keypoint_margin)
  , async_(true)
{
  if (async_)
  {
    thread_pool_.startThreads(rig_.size());
  }

  for (size_t i = 0u; i < rig.size(); ++i)
  {
    grid_vec_.push_back(OccupancyGrid2D(grid_size, rig.at(i).size()));
    matcher_vec_.at(i).options_.margin_cur_level0 = keypoint_margin;
  }
}

void LandmarksReprojector::reset()
{
  for (OccupancyGrid2D& grid : grid_vec_)
  {
    grid.reset();
  }
}

//------------------------------------------------------------------------------
uint32_t LandmarksReprojector::projectLandmarksInNFrame(
    NFrame& nframe, const Transformation& T_B_W)
{
  reset();

  uint32_t num_matches = 0u;
  if (async_)
  {
    // Assign tasks to threads in pool.
    std::vector<std::future<MatchResultVector>> results;
    for (uint32_t i = 0u; i < nframe.size(); ++i)
    {
      results.emplace_back(thread_pool_.enqueue(
                             projectAllLandmarksInFrame,
                             std::cref(rig_), std::cref(landmarks_),
                             std::cref(states_), std::ref(matcher_vec_[i]),
                             std::ref(grid_vec_[i]), std::ref(nframe), i,
                             std::cref(T_B_W), keypoint_margin_));
    }

    // Make sure both threads are finished.
    for (uint32_t i = 0u; i < nframe.size(); ++i)
    {
      const MatchResultVector& match_results = results[i].get();

      // Increase num successful / fail projection counter in landmarks.
      num_matches += updateLandmarkStatistics(match_results, landmarks_);
    }
  }
  else
  {
    for (uint32_t i = 0u; i < nframe.size(); ++i)
    {
      MatchResultVector match_results =
          projectAllLandmarksInFrame(rig_, landmarks_, states_, matcher_vec_[i],
                                     grid_vec_[i], nframe, i, T_B_W, keypoint_margin_);

      // Increase num successful / fail projection counter in landmarks.
      num_matches += updateLandmarkStatistics(match_results, landmarks_);
    }
  }
  return num_matches;
}

//------------------------------------------------------------------------------
MatchResultVector projectAllLandmarksInFrame(
    const CameraRig& rig,
    const LandmarkTable& landmarks,
    const NFrameTable& states,
    EpipolarMatcher& matcher,
    OccupancyGrid2D& grid,
    NFrame& cur_nframe,
    const uint32_t cur_frame_idx,
    const Transformation& T_B_W,
    const real_t keypoint_margin)
{
  MatchResultVector match_results(landmarks.numLandmarks(), MatchResult::Unmatched);

  // Project all triangulated landmarks.
  auto stableLandmarksPredicate = [&](uint32_t i) -> bool
  {
    return isLandmarkTriangulated(landmarks.typeAtSlot(i));
  };

  projectAndMatchLandmarks(rig, landmarks, states, matcher, grid, cur_nframe,
                           match_results, cur_frame_idx, T_B_W, keypoint_margin,
                           true, stableLandmarksPredicate);


  // If we don't have enough features, also reproject unconverged seeds.
  auto seedsPredicate = [&](uint32_t i) -> bool
  {
    return landmarks.typeAtSlot(i) == LandmarkType::Seed
        && landmarks.projectionQualityAtSlot(i) > FLAGS_vio_reprojector_min_quality_to_project;
  };
  projectAndMatchLandmarks(rig, landmarks, states, matcher, grid, cur_nframe,
                           match_results, cur_frame_idx, T_B_W, keypoint_margin,
                           true, seedsPredicate);

  // Complete frame.
  cur_nframe.at(cur_frame_idx).f_vec_.leftCols(cur_nframe.at(cur_frame_idx).num_features_) =
      rig.at(cur_frame_idx).backProjectVectorized(
        cur_nframe.at(cur_frame_idx).px_vec_.leftCols(cur_nframe.at(cur_frame_idx).num_features_));

  return match_results;
}

// -----------------------------------------------------------------------------
void projectAndMatchLandmarks(
    const CameraRig& rig,
    const LandmarkTable& landmarks,
    const NFrameTable& states,
    EpipolarMatcher& matcher,
    OccupancyGrid2D& grid,
    NFrame& nframe,
    MatchResultVector& match_results,
    const uint32_t frame_idx,
    const Transformation& T_B_W,
    const real_t keypoint_margin,
    const bool set_remaining_occupied,
    const std::function<bool (uint32_t)>& landmarkSelectionPredicate)
{
  const Frame& frame = nframe.at(frame_idx);
  const Camera& cam = rig.at(frame_idx);
  const Transformation T_C_W = rig.T_C_B(frame_idx) * T_B_W;
  const Size2u img_size = cam.size();
  uint32_t num_projected = 0u;
  MatchCandidates candidates;
  candidates.reserve(landmarks.numLandmarks());

  for (uint32_t i = 0u, imax = landmarks.numLandmarks(); i < imax; ++i)
  {
    if (!landmarkSelectionPredicate(i))
    {
      continue;
    }

    // Check if point in front of camera.
    Position xyz_cur = T_C_W * landmarks.p_W_atSlot(i);
    if (xyz_cur.z() < 0.0)
    {
      continue;
    }

    // Check if point is visible in image.
    Keypoint px_cur = cam.project(xyz_cur);
    if (isVisibleWithMargin(img_size, px_cur, keypoint_margin))
    {
      ++num_projected;
      candidates.emplace_back(
            MatchCandidate(px_cur, i, landmarks.typeAtSlot(i),
                           landmarks.projectionQualityAtSlot(i)));
    }
  }
  const uint32_t num_matches_before = frame.num_features_;
  matchAllCandidates(rig, landmarks, states, T_C_W, frame_idx, nframe, candidates,
                     grid, match_results, matcher, set_remaining_occupied);

  VLOG(3) << "Cam " << frame_idx << " - matched successfully "
          << frame.num_features_ - num_matches_before
          << " of " << num_projected << " visible landmarks.";
}

// -----------------------------------------------------------------------------
void matchAllCandidates(
    const CameraRig& rig,
    const LandmarkTable& landmarks,
    const NFrameTable& states,
    const Transformation& T_Ccur_W,
    const uint32_t cur_frame_idx,
    NFrame& cur_nframe,
    MatchCandidates& candidates,
    OccupancyGrid2D& grid,
    MatchResultVector& match_results,
    EpipolarMatcher& matcher,
    bool project_remaining_in_grid)
{
  Frame& cur_frame = cur_nframe.at(cur_frame_idx);

  sortCandidates(cur_frame.num_features_, candidates);

  uint32_t i = 0u;
  for ( ; i < candidates.size(); ++i)
  {
    if (cur_frame.num_features_ >= FLAGS_vio_max_tracked_features_per_frame)
    {
      break;
    }

    MatchCandidate& candidate = candidates[i];
    size_t grid_index = grid.getCellIndex(candidate.cur_px.x(), candidate.cur_px.y());
    if (FLAGS_vio_reprojector_limit_num_features && grid.isOccupied(grid_index))
    {
      continue;
    }

    // Get the reference view to match this keyframe with.
    const LandmarkObsVec& ref_obs_vec = landmarks.obsAtSlot(candidate.lm_slot);
    if (ref_obs_vec.empty())
    {
      LOG(ERROR) << "Landmark at slot " << candidate.lm_slot << " has no observations:"
                 << " Type = " << landmarkTypeAsString(candidate.lm_type)
                 << ", Num Reproj = " << candidate.lm_score;
      continue;
    }

    //! @todo pick the best one!
    //! For seeds, the first observation is also the reference observation!
    const LandmarkObs& ref_obs =
        (candidate.lm_type == LandmarkType::Seed)
        ? ref_obs_vec.front() : ref_obs_vec.back();

    if (!states.isStored(ref_obs.nframe_handle_))
    {
      continue;
    }

    //! @todo: here we may take another frame for every projection -> costly!
    const Transformation& T_Bref_W = states.T_B_W(ref_obs.nframe_handle_);
    const Transformation T_Cref_W = rig.T_C_B(ref_obs.frame_idx_) * T_Bref_W;
    const Transformation T_Ccur_Cref = T_Ccur_W * T_Cref_W.inverse();
    const NFrame& ref_nframe =  *states.nframe(ref_obs.nframe_handle_);

    if (ref_nframe.at(ref_obs.frame_idx_).level_vec_(ref_obs.keypoint_idx_) > 10)
    {
      LOG(FATAL)
          << "Can happen if version of nframe slot wraps around"
          << "NFrame" << ref_nframe << "\n"
          << "Frame-ID = " << (int) ref_obs.frame_idx_ << "\n"
          << "Level not in range: \n"
          << "Frame: \n" << ref_nframe.at(ref_obs.frame_idx_) << "\n"
          << "Keypoint Index = " << ref_obs.keypoint_idx_ << "\n"
          << "Level " << (int) ref_nframe.at(ref_obs.frame_idx_).level_vec_(ref_obs.keypoint_idx_) << "\n"
          << "Landmark type = " << landmarkTypeAsString(candidate.lm_type)
          << "Obs: \n" << ref_obs_vec;
    }

    EpipolarMatchResult res =
        matchCandidate(rig, ref_nframe, cur_nframe, ref_obs.frame_idx_,
                       cur_frame_idx, ref_obs.keypoint_idx_, ref_obs.f_,
                       T_Cref_W, T_Ccur_Cref, landmarks, matcher, candidate);

    if (res != EpipolarMatchResult::Success)
    {
      VLOG(40) << cur_frame_idx << ": " << landmarkTypeAsString(candidate.lm_type)
               << " " << candidate.lm_slot << " with " << candidate.lm_score
               << " score, level = "
               << (int) ref_nframe.at(ref_obs.frame_idx_).level_vec_[ref_obs.keypoint_idx_]
               << " failed: " << stringFromEpipolarMatchResult(res);

      DEBUG_CHECK_LT(candidate.lm_slot, match_results.size());
      match_results[candidate.lm_slot] = MatchResult::Fail;
    }
    else
    {
      grid.setOccupied(grid_index);

      DEBUG_CHECK_LT(candidate.lm_slot, match_results.size());
      match_results[candidate.lm_slot] = MatchResult::Success;
    }
  }

  if (project_remaining_in_grid && i < candidates.size())
  {
    setGridCellsOccupied(grid, candidates.cbegin() + i, candidates.cend());
  }
}

// -----------------------------------------------------------------------------
uint32_t updateLandmarkStatistics(
    const MatchResultVector& match_results,
    LandmarkTable& landmarks)
{
  uint32_t num_success = 0u;
  for (size_t i = 0u; i < match_results.size(); ++i)
  {
    if (match_results[i] == MatchResult::Success)
    {
      landmarks.incrementSuccessfulProjectionsAtSlot(i);
      ++num_success;
    }
    else if (match_results[i] == MatchResult::Fail)
    {
      landmarks.incrementFailedProjectionsAtSlot(i);
    }
  }
  return num_success;
}

// -----------------------------------------------------------------------------
void setGridCellsOccupied(
    OccupancyGrid2D& grid,
    const MatchCandidates::const_iterator begin,
    const MatchCandidates::const_iterator end)
{
  // Set remaining grid cells as occupied if we stopped early.
  VLOG(40) << "Set " << (end - begin) << " remaining grid cells as occupied.";

  for (MatchCandidates::const_iterator it = begin; it != end; ++it)
  {
    size_t grid_index = grid.getCellIndex(it->cur_px.x(), it->cur_px.y());
    grid.setOccupied(grid_index);
  }
}

// -----------------------------------------------------------------------------
void sortCandidates(
    const int num_existing_matches,
    MatchCandidates& candidates)
{
  // If we don't have sufficient candidates, we don't need to sort them.
  size_t num_desired_matches =
      std::max(static_cast<int>(FLAGS_vio_max_tracked_features_per_frame) - num_existing_matches, 0);
  if (candidates.size() < num_desired_matches)
  {
    VLOG(40) << "Sorting not necessary since we need to project all landmarks anyway";
    return;
  }

  // Need to sort all, in case matching fails.
  std::sort(candidates.begin(), candidates.end(),
            [](const MatchCandidate& lhs, const MatchCandidate& rhs)
  {
    if(lhs.lm_type > rhs.lm_type
       || (lhs.lm_type == rhs.lm_type && lhs.lm_score > rhs.lm_score))
       //! @todo || (lhs.type == rhs.type && lhs.n_reproj == rhs.n_reproj && lhs.score > rhs.score))
    {
      return true;
    }
    return false;
  });
}

// -----------------------------------------------------------------------------
EpipolarMatchResult matchCandidate(
    const CameraRig& rig,
    const NFrame& ref_nframe,
    NFrame& cur_nframe,
    const uint32_t ref_frame_idx,
    const uint32_t cur_frame_idx,
    const uint32_t ref_obs_keypoint_idx,
    const Eigen::Ref<const Bearing>& ref_f,
    const Transformation& T_Cref_W,
    const Transformation& T_Ccur_Cref,
    const LandmarkTable& landmarks,
    EpipolarMatcher& matcher,
    MatchCandidate& candidate)
{
  const Frame& ref_frame = ref_nframe.at(ref_frame_idx);
  Frame& cur_frame = cur_nframe.at(cur_frame_idx);

  EpipolarMatchResult res;
  if (candidate.lm_type == LandmarkType::Seed)
  {
    // For seeds, we search a match along the epipolar line.
    const Eigen::Ref<const Seed> seed = landmarks.seedAtSlot(candidate.lm_slot);
    std::tie(candidate.cur_px, res) =
        matcher.findEpipolarMatchDirect(
        T_Ccur_Cref,
        *ref_frame.pyr_,
        *cur_frame.pyr_,
        rig.at(ref_frame_idx),
        rig.at(cur_frame_idx),
        ref_frame.px_vec_.col(ref_obs_keypoint_idx),
        ref_f,
        ref_frame.level_vec_[ref_obs_keypoint_idx],
        seed(0),
        seed(0) + std::sqrt(seed(1)),
        std::max(seed(0) - std::sqrt(seed(1)), real_t{0.00000001}));
  }
  else
  {
    // For non-seed (default), we just refine the patch position.
    //! @todo: memory-wise, this is reeeally inefficient.
    real_t inv_depth_ref =
        real_t{1.0} / (T_Cref_W * landmarks.p_W_atSlot(candidate.lm_slot)).norm();
    res = matcher.refineMatchDirect(
          T_Ccur_Cref,
          *ref_frame.pyr_,
          *cur_frame.pyr_,
          rig.at(ref_frame_idx),
          rig.at(cur_frame_idx),
          ref_frame.px_vec_.col(ref_obs_keypoint_idx),
          ref_f,
          ref_frame.level_vec_[ref_obs_keypoint_idx],
          inv_depth_ref,
          candidate.cur_px);

    if (res == EpipolarMatchResult::FailAlignment
        && FLAGS_vio_reprojector_pyr_alignment)
    {
      // If the alignment fails, we repeat the alignment with a pyramidal approach.
      res = matcher.refineMatchDirect(
            T_Ccur_Cref,
            *ref_frame.pyr_,
            *cur_frame.pyr_,
            rig.at(ref_frame_idx),
            rig.at(cur_frame_idx),
            ref_frame.px_vec_.col(ref_obs_keypoint_idx),
            ref_f,
            ref_frame.level_vec_[ref_obs_keypoint_idx],
            inv_depth_ref,
            candidate.cur_px,
            2);
    }
  }

  if (res != EpipolarMatchResult::Success)
  {
    return res;
  }

  CHECK(static_cast<uint32_t>(cur_frame.px_vec_.cols()) > cur_frame.num_features_);

  cur_frame.px_vec_.col(cur_frame.num_features_) =
      candidate.cur_px;
  cur_frame.level_vec_(cur_frame.num_features_) =
      matcher.level_cur_;
  cur_frame.type_vec_(cur_frame.num_features_) =
      ref_frame.type_vec_(ref_obs_keypoint_idx);
  cur_frame.angle_vec_(cur_frame.num_features_) =
      ref_frame.angle_vec_(ref_obs_keypoint_idx);
  cur_frame.score_vec_(cur_frame.num_features_) =
      ref_frame.score_vec_(ref_obs_keypoint_idx);
  cur_frame.landmark_handles_[cur_frame.num_features_] =
      ref_frame.landmark_handles_[ref_obs_keypoint_idx];
  ++cur_frame.num_features_;
  return res;
}

} // namespace ze
