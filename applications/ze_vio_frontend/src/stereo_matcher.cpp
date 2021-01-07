// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_frontend/stereo_matcher.hpp>

#include <imp/correspondence/sparse_stereo.hpp>
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

StereoMatcher::StereoMatcher(
    const CameraRig& rig,
    const real_t keypoint_margin,
    const real_t reprojection_threshold_px,
    LandmarkTable& landmarks)
  : landmarks_(landmarks)
  , rig_(rig)
  , T_C_B_(rig.T_C_B_vec())
  , keypoint_margin_(keypoint_margin)
  , reprojection_threshold_px_(reprojection_threshold_px)
{}

// -----------------------------------------------------------------------------
std::vector<std::pair<uint32_t, std::vector<uint32_t>>>
StereoMatcher::matchStereoAndRejectOutliers(
    NFrame& nframe,
    const Transformation& T_B_W)
{
  DEBUG_CHECK_GE(nframe.size(), 2u);

  std::vector<std::pair<uint32_t, std::vector<uint32_t>>> inliers;
  for (const StereoIndexPair pair : rig_.stereoPairs())
  {
    uint32_t cur_frame_idx = pair.first;
    uint32_t ref_frame_idx = pair.second;
    if (stereo_count_ % 2 == 0)
    {
      cur_frame_idx = pair.second;
      ref_frame_idx = pair.first;
    }

    Frame& ref_frame = nframe.at(ref_frame_idx);
    Frame& cur_frame = nframe.at(cur_frame_idx);
    DEBUG_CHECK_GT(ref_frame.median_depth_, 0.0);
    DEBUG_CHECK_GT(ref_frame.min_depth_, 0.0);
    DEBUG_CHECK_GT(ref_frame.max_depth_, 0.0);
    DEBUG_CHECK_GT(cur_frame.median_depth_, 0.0);
    DEBUG_CHECK_GT(cur_frame.min_depth_, 0.0);
    DEBUG_CHECK_GT(cur_frame.max_depth_, 0.0);

    // Find matching features in right image.
    matchStereo(nframe, T_B_W, ref_frame_idx, cur_frame_idx,
                1.0 / ref_frame.median_depth_,
                1.0 / ref_frame.min_depth_,
                1.0 / ref_frame.max_depth_);

    // Find matching features in left image.
    matchStereo(nframe, T_B_W, cur_frame_idx, ref_frame_idx,
                1.0 / ref_frame.median_depth_,
                1.0 / ref_frame.min_depth_,
                1.0 / ref_frame.max_depth_);

    // Outlier rejection.
    inliers.push_back(
          std::make_pair(
            ref_frame_idx,
            stereoOutlierRejection(nframe, ref_frame_idx, cur_frame_idx).first));
  }

  return inliers;
}

// -----------------------------------------------------------------------------
uint32_t StereoMatcher::matchStereo(
    NFrame& nframe,
    const Transformation& T_B_W,
    const uint32_t ref_frame_idx,
    const uint32_t cur_frame_idx,
    const real_t inv_depth_ref_estimate,
    const real_t inv_depth_ref_min,
    const real_t inv_depth_ref_max)
{
  VLOG(3) << "Match stereo between frames " << ref_frame_idx << " and " << cur_frame_idx;
  const Transformation T_Ccur_Cref = T_C_B_[cur_frame_idx] * T_C_B_[ref_frame_idx].inverse();
  const Transformation T_W_B = T_B_W.inverse();
  const Transformation T_W_Ccur = T_W_B * T_C_B_[cur_frame_idx].inverse();
  const Frame& frame_ref = nframe.at(ref_frame_idx);
  Frame& frame_cur = nframe.at(cur_frame_idx);

  // First, find those features that have not a match yet in the other frame.
  std::vector<uint32_t> ref_indices = getUnmatchedIndices<LandmarkHandle::value_t>(
        frame_ref.getLandmarkHandlesAsVector(),
        frame_cur.getLandmarkHandlesAsVector(),
        std::bind(&isValidLandmarkHandleType, std::placeholders::_1));
  const uint32_t num_features_to_match = ref_indices.size();

  // Extract reference features we need to match:
  Keypoints px_ref(2, num_features_to_match);
  Bearings f_ref(3, num_features_to_match);
  KeypointLevels levels_ref(num_features_to_match);
  for(uint32_t i = 0; i < num_features_to_match; ++i)
  {
    uint32_t i_ref = ref_indices[i];
    px_ref.col(i) = frame_ref.px_vec_.col(i_ref);
    f_ref.col(i) = frame_ref.f_vec_.col(i_ref);
    levels_ref(i) = frame_ref.level_vec_(i_ref);
  }

  // Match stereo.
  SparseStereoMatcherOptions options;
  options.algorithm = SparseStereoAlgorithm::EpipolarSampling;
  options.margin_at_level = keypoint_margin_;
  SparseStereoMatcher matcher(
        rig_.at(ref_frame_idx), rig_.at(cur_frame_idx),
        *frame_ref.pyr_, *frame_cur.pyr_, T_Ccur_Cref, px_ref, f_ref, levels_ref,
        options);
  const uint32_t num_matches = matcher.computeMatches(
        inv_depth_ref_estimate, inv_depth_ref_min, inv_depth_ref_max);
  const uint32_t num_inliers = matcher.triangulateMatches(
                                 1.0, (1.0 / inv_depth_ref_min) * 0.2,
                                 (1.0 / inv_depth_ref_max) * 5.0);

  // Allocate feature storage in current frame.
  if(num_inliers > frame_cur.featureCapacity())
  {
    LOG(WARNING) << "Feature storage capacity is not enough. Need "
                 << (num_inliers - frame_cur.featureCapacity()) << " more.";
    frame_cur.resizeFeatureStorage(frame_cur.num_features_ + num_inliers);
  }

  // Save matches in current frame.
  const uint32_t num_before = frame_cur.num_features_;
  frame_cur.px_vec_.middleCols(num_before, num_inliers) = matcher.px_cur_;
  frame_cur.f_vec_.middleCols(num_before, num_inliers)  = matcher.f_cur_;
  frame_cur.level_vec_.segment(num_before, num_inliers) = matcher.levels_cur_;
  frame_cur.num_features_ += num_inliers;

  // Add reference to landmark in frame.
  int num_updated_seeds = 0;
  const Transformation T_Cref_Ccur = T_Ccur_Cref.inverse();
  for (uint32_t i = 0; i < num_inliers; ++i)
  {
    uint32_t i_ref = ref_indices.at(matcher.inliers_.at(i));
    uint32_t i_cur = num_before + i;

    const LandmarkHandle lm_h = frame_ref.landmark_handles_[i_ref];
    DEBUG_CHECK(isValidLandmarkHandle(lm_h));
    frame_cur.landmark_handles_[i_cur] = lm_h;
    frame_cur.type_vec_(i_cur) = frame_ref.type_vec_(i_ref);
    frame_cur.angle_vec_(i_cur) = frame_ref.angle_vec_(i_ref); //! @todo check.
    frame_cur.score_vec_(i_cur) = frame_ref.score_vec_(i_ref);
    landmarks_.p_W(lm_h) = T_W_Ccur * matcher.xyz_cur_.col(i);
    landmarks_.incrementSuccessfulProjections(lm_h);

    // Update seed in reference frame.
    if (i_ref <  frame_ref.seeds_index_to_
        && i_ref >= frame_ref.seeds_index_from_)
    {
      landmarks_.seed(lm_h)(0) = real_t{1.0} / (T_Cref_Ccur * matcher.xyz_cur_.col(i)).norm();
      landmarks_.seed(lm_h)(1) *= real_t{0.5}; // Lower depth variance.
      ++num_updated_seeds;
    }
  }

  VLOG_IF(10, num_features_to_match - num_matches)
      << "Stereo matching fails: " << num_features_to_match - num_matches;
  VLOG_IF(10, num_matches - num_inliers)
      << "Stereo triangulation fails: " << num_matches - num_inliers;
  VLOG(10) << "Stereo mached " << frame_cur.num_features_ - num_before
           << " of " << num_features_to_match;
  VLOG(10) << "Stereo updated " << num_updated_seeds << " seeds.";
  return num_matches;
}

// -----------------------------------------------------------------------------
std::pair<std::vector<uint32_t>, uint32_t> StereoMatcher::stereoOutlierRejection(
    NFrame& nframe,
    const uint32_t ref_idx,
    const uint32_t cur_idx)
{
  const Transformation T_cur_ref = T_C_B_[cur_idx] * T_C_B_[ref_idx].inverse();
  Frame& frame_ref = nframe.at(ref_idx);
  Frame& frame_cur = nframe.at(cur_idx);

  // Find matching indices.
  auto indices_cur_ref = getMatchIndices<LandmarkHandle::value_t>(
        frame_cur.getLandmarkHandlesAsVector(),
        frame_ref.getLandmarkHandlesAsVector(),
        std::bind(&isValidLandmarkHandleType, std::placeholders::_1));

  // Collect all matching bearing vectors.
  const size_t n = indices_cur_ref.size();
  if (n == 0u)
  {
    LOG(WARNING) << "Have no stereo correspondences.";
    return std::make_pair(std::vector<uint32_t>(), 0u);
  }
  Bearings f_cur(3, n);
  Bearings f_ref(3, n);
  for (size_t i = 0; i < n; ++i)
  {
    f_cur.col(i) = frame_cur.f_vec_.col(indices_cur_ref[i].first);
    f_ref.col(i) = frame_ref.f_vec_.col(indices_cur_ref[i].second);
  }

  // Compute reprojection errors.
  Positions p_cur(3, n);
  VectorX e_vec(n);
  triangulateManyAndComputeAngularErrors(T_cur_ref, f_cur, f_ref, p_cur, e_vec);

  // Remove outliers.
  real_t thresh = real_t{1.0} - std::cos(rig_.at(0).getApproxAnglePerPixel()
                                               * reprojection_threshold_px_);
  std::vector<uint32_t> inlier_indices_ref;
  inlier_indices_ref.reserve(n);
  uint32_t num_outliers = 0;
  for (size_t i = 0; i < n; ++i)
  {
    auto cr = indices_cur_ref[i];
    LandmarkHandle& h_ref = frame_ref.landmark_handles_[cr.second];
    LandmarkHandle& h_cur = frame_cur.landmark_handles_[cr.first];
    DEBUG_CHECK_EQ(h_ref, h_cur);
    DEBUG_CHECK(isValidLandmarkHandle(h_ref));
    if(e_vec(i) > thresh)
    {
      landmarks_.type(h_ref) = LandmarkType::Outlier;
      markLandmarkHandleAsOutlier(h_ref);
      markLandmarkHandleAsOutlier(h_cur);
      ++num_outliers;
    }
    else
    {
      inlier_indices_ref.push_back(cr.second);
    }
  }
  VLOG(3) << "Stereo consistency: have " << inlier_indices_ref.size()
          << " inliers and " << num_outliers << " outliers";
  return std::make_pair(inlier_indices_ref, num_outliers);
}

// -----------------------------------------------------------------------------
void StereoMatcher::updateSeedDepthAfterStereo(
    const Transformation& T_B_W,
    const NFrame& nframe)
{
  //! @todo: not currently used because all stereo triangulations are directly
  //!        set as opportunistic.

  for (uint32_t frame_idx = 0u; frame_idx < nframe.size(); ++frame_idx)
  {
    const Frame& frame = nframe.at(frame_idx);
    const Transformation T_C_W = T_C_B_[frame_idx] * T_B_W;

    for (uint32_t i = frame.seeds_index_from_; i < frame.seeds_index_to_; ++i)
    {
      const LandmarkHandle lm_h = frame.landmark_handles_[i];
      if (isValidLandmarkHandle(lm_h)
          && landmarks_.numObs(lm_h) > 1u) //! @todo accessing numObs is not efficient.
      {
        real_t inv_depth = real_t{1.0} / (T_C_W * landmarks_.p_W(lm_h)).norm();
        Eigen::Ref<Seed> seed = landmarks_.seed(lm_h);
        seed(0) = inv_depth;       // set mean.
        seed(1) *= real_t{0.5}; // reduce covariance.
      }
    }
  }
}


} // namespace ze
