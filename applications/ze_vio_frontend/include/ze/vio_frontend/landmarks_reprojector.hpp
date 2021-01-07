// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <imp/correspondence/epipolar_matcher.hpp>
#include <imp/features/occupancy_grid_2d.hpp>
#include <ze/common/transformation.hpp>
#include <ze/common/thread_pool.hpp>
#include <ze/vio_common/landmark_types.hpp>

namespace ze {

// fwd
class CameraRig;
class Frame;
class NFrame;
class NFrameTable;
class LandmarkTable;

//! A candidate is a landmark that projects into the image and for which we will
//! search a match after sorting the candidates to start with the most promising.
struct MatchCandidate
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Keypoint cur_px;
  uint32_t lm_slot;
  LandmarkType lm_type;
  int32_t lm_score;

  MatchCandidate() = default;

  MatchCandidate(
      const Keypoint& cur_px,
      uint32_t lm_slot,
      LandmarkType lm_type,
      int32_t num_proj)
    : cur_px(cur_px) , lm_slot(lm_slot), lm_type(lm_type), lm_score(num_proj)
  { ; }
};
using MatchCandidates = std::vector<MatchCandidate>;

enum class MatchResult : int8_t
{
  Unmatched,
  Success,
  Fail
};
using MatchResultVector = std::vector<MatchResult>;

class LandmarksReprojector
{
public:
  LandmarksReprojector() = delete;
  LandmarksReprojector(
      const CameraRig& rig,
      const real_t keypoint_margin,
      const uint32_t grid_size,
      LandmarkTable& landmarks,
      NFrameTable& states);

  void reset();

  //! @return Returns number of matched features.
  uint32_t projectLandmarksInNFrame(
      NFrame& nframe,
      const Transformation& T_B_W);

  inline const OccupancyGrid2dVector& gridVec() const
  {
    return grid_vec_;
  }

private:
  EpipolarMatcherVector matcher_vec_;
  LandmarkTable& landmarks_;
  const NFrameTable& states_;
  const CameraRig& rig_;
  const TransformationVector T_C_B_;
  OccupancyGrid2dVector grid_vec_;
  ThreadPool thread_pool_;

  // Configs
  real_t keypoint_margin_;
  bool async_;
};

MatchResultVector projectAllLandmarksInFrame(
    const CameraRig& rig,
    const LandmarkTable& landmarks,
    const NFrameTable& states,
    EpipolarMatcher& matcher,
    OccupancyGrid2D& grid,
    NFrame& nframe,
    const uint32_t frame_idx,
    const Transformation& T_B_W,
    const real_t keypoint_margin);

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
    const std::function<bool (uint32_t)>& landmarkSelectionPredicate);

void matchAllCandidates(
    const CameraRig& rig,
    const LandmarkTable& landmarks,
    const NFrameTable& states,
    const Transformation& T_Ccur_W,
    const uint32_t frame_idx,
    NFrame& cur_nframe,
    MatchCandidates& candidates,
    OccupancyGrid2D& grid,
    MatchResultVector& match_results,
    EpipolarMatcher& matcher,
    bool project_remaining_in_grid);

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
    MatchCandidate& candidate);

//! @return Returns number of successful matches.
uint32_t updateLandmarkStatistics(
    const MatchResultVector& match_results,
    LandmarkTable& landmarks);

void setGridCellsOccupied(
    OccupancyGrid2D& grid,
    const MatchCandidates::const_iterator begin,
    const MatchCandidates::const_iterator end);

void sortCandidates(
    const int num_existing_matches,
    MatchCandidates& candidates);

} // namespace ze
