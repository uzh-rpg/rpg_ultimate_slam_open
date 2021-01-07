// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_frontend/feature_tracker.hpp>

#include <gflags/gflags.h>

#include <imp/correspondence/klt.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <ze/common/combinatorics.hpp>
#include <ze/common/logging.hpp>
#include <ze/common/statistics.hpp>
#include <ze/common/types.hpp>
#include <ze/common/string_utils.hpp>
#include <ze/geometry/ransac_relative_pose.hpp>
#include <ze/geometry/triangulation.hpp>
#include <ze/vio_common/frame.hpp>
#include <ze/vio_common/landmark_table.hpp>
#include <ze/vio_common/nframe.hpp>
#include <ze/vio_frontend/stereo_matcher.hpp>


DEFINE_string(vio_feature_tracker_patch_size_by8, "4,4",
            "Patch size for each pyramid in feature tracker (in multiples of 8!)");
DEFINE_double(vio_feature_tracker_termcrit_min_update_squared, 0.03,
            "Minimum parameter update in KLT to consider tracking converged.");

namespace ze {

FeatureTracker::FeatureTracker(
    const CameraRig& rig,
    const real_t keypoint_margin,
    const real_t reprojection_error_threshold_px,
    const bool use_5pt_ransac,
    StereoMatcher& stereo_matcher,
    LandmarkTable& landmarks)
  : landmarks_(landmarks)
  , rig_(rig)
  , T_C_B_(rig_.T_C_B_vec())
  , stereo_matcher_(stereo_matcher)
  , async_(rig_.size() > 1u)
  , use_5pt_ransac_(use_5pt_ransac)
  , reprojection_error_threshold_px_(reprojection_error_threshold_px)
  , keypoint_margin_(keypoint_margin)
{
  if (async_)
  {
    thread_pool_.startThreads(rig_.size());
  }
}

// -----------------------------------------------------------------------------
FeatureTracker::ProjectionResult projectLandmarksAndLKTracking(
    const uint32_t ref_frame_idx,
    const uint32_t cur_frame_idx,
    const CameraRig& rig,
    const Transformation T_Bref_W,
    const Transformation T_Bcur_W,
    const LandmarkTable& landmarks,
    const real_t keypoint_margin,
    NFrame& ref_nframe,
    NFrame& cur_nframe)
{
  const Camera& cam_ref = rig.at(ref_frame_idx);
  const Camera& cam_cur = rig.at(cur_frame_idx);
  const Transformation T_Ccur_W = rig.T_C_B(cur_frame_idx) * T_Bcur_W;
  const Transformation T_Cref_W = rig.T_C_B(ref_frame_idx) * T_Bref_W;
  const Transformation T_cur_ref = T_Ccur_W * T_Cref_W.inverse();
  Frame& ref_frame = ref_nframe.at(ref_frame_idx);
  Frame& cur_frame = cur_nframe.at(cur_frame_idx);
  FeatureTracker::ProjectionResult proj_res;

  // First, select which landmarks we want to reproject:
  std::vector<uint32_t> feature_indices;
  feature_indices.reserve(ref_frame.num_features_);
  for(uint32_t i = 0; i < ref_frame.num_features_; ++i)
  {
    const LandmarkHandle lm_h = ref_frame.landmark_handles_[i];
    if (landmarks.isStored(lm_h) && landmarks.isActive(lm_h))
    {
      feature_indices.push_back(i);
    }
  }

  // Prepare data that we need for KLT.
  const uint32_t num_features = feature_indices.size();
  if (num_features == 0u)
  {
    VLOG(1) << "Have no landmarks to project.";
    return proj_res;
  }
  Keypoints px_ref_vec(2, num_features);
  Bearings f_ref_vec(3, num_features);
  VectorX inv_depth_ref_vec(num_features);
  LandmarkHandles lm_handles(num_features);
  KeypointLevels levels_ref(num_features);
  for (uint32_t i = 0u; i < num_features; ++i)
  {
    const uint32_t i_orig = feature_indices[i];
    const LandmarkHandle lm_h = ref_frame.landmark_handles_[i_orig];
    lm_handles[i] = lm_h;
    px_ref_vec.col(i) = ref_frame.px_vec_.col(i_orig);
    f_ref_vec.col(i) = ref_frame.f_vec_.col(i_orig);
    levels_ref(i) = ref_frame.level_vec_(i_orig);
    inv_depth_ref_vec(i) = 1.0 / (T_Cref_W * landmarks.p_W(lm_h)).norm();
  }

  // Pyramidal KLT over four levels, patch sizes vary on each level.
  Keypoints px_cur_vec(2, num_features);
  KltParameters klt_params;
  klt_params.border_margin = keypoint_margin;
  klt_params.termcrit_min_update_squared =
      std::pow(FLAGS_vio_feature_tracker_termcrit_min_update_squared, 2.);
  std::vector<int> patch_sizes_by8 = readIntsFromString(FLAGS_vio_feature_tracker_patch_size_by8);
  KltResults res = alignFeaturesGuidedPyr(
                     *ref_frame.pyr_, *cur_frame.pyr_, cam_ref, cam_cur,
                     patch_sizes_by8, klt_params, true, T_cur_ref, px_ref_vec,
                     f_ref_vec, inv_depth_ref_vec, px_cur_vec);

  // Count number of matches and allocate memory if necessary!
  const uint32_t num_matches = std::count(res.begin(), res.end(), KltResult::Converged);
  cur_frame.ensureFeatureCapacity(num_matches);

  const uint32_t num_before = cur_frame.num_features_;
  uint32_t num_failed = 0u;
  for (size_t n = 0; n < res.size(); ++n)
  {
    const LandmarkHandle lm_h = lm_handles[n];
    if (res[n] != KltResult::Converged)
    {
      ++num_failed;
      proj_res.failed_projections.push_back(lm_h);
      continue;
    }
    cur_frame.landmark_handles_[cur_frame.num_features_] = lm_h;
    cur_frame.px_vec_.col(cur_frame.num_features_) = px_cur_vec.col(n);
    cur_frame.level_vec_(cur_frame.num_features_) = 0u; //! @todo: pyramid level!
    cur_frame.num_features_++;
    proj_res.successful_projections.push_back(lm_h);
  }

  // Compute bearing vectors, corresponding to matched features
  Bearings f = cam_cur.backProjectVectorized(
        cur_frame.px_vec_.middleCols(num_before, cur_frame.num_features_ - num_before));
  cur_frame.f_vec_.middleCols(num_before, cur_frame.num_features_ - num_before) = f;

  return proj_res;
}

// -----------------------------------------------------------------------------
void FeatureTracker::postprocessTrackingResults(
    const size_t frame_idx,
    const FeatureTracker::ProjectionResult& res)
{
  for (const auto lm_h : res.failed_projections)
  {
    landmarks_.incrementFailedProjections(lm_h);
  }
  for (const auto lm_h : res.successful_projections)
  {
    landmarks_.incrementSuccessfulProjections(lm_h);
  }
  size_t num_failed = res.failed_projections.size();
  size_t num_succ = res.successful_projections.size();

  /*
  stats_["klt_num_features_per_frame"].addSample(num_failed + num_succ);
  stats_["projection_num_failed"].addSample(num_succ);
  stats_["projection_num_succeeded"].addSample(num_succ);
  */

  VLOG(3)
      << "Tracked successfully " << num_succ << " of " << num_succ + num_failed
      << " features from ref-cam " << frame_idx << " to cur-cam "
      << frame_idx << ", num failures = " << num_failed;
}

// -----------------------------------------------------------------------------
size_t FeatureTracker::trackFeaturesInNFrame(
    const Transformation& T_Bref_W,
    const Transformation& T_Bcur_W,
    NFrame& ref_nframe,
    NFrame& cur_nframe)
{
  size_t num_successful_tracked = 0u;
  if (async_)
  {
    // Project Landmarks concurrently in N frames.
    std::vector<std::future<ProjectionResult>> results;
    for (size_t frame_idx = 0u; frame_idx < ref_nframe.size(); ++frame_idx)
    {
      auto f = std::bind(projectLandmarksAndLKTracking,
                         frame_idx, frame_idx, std::cref(rig_),
                         T_Bref_W, T_Bcur_W, std::cref(landmarks_),
                         keypoint_margin_,
                         std::ref(ref_nframe), std::ref(cur_nframe));
      results.emplace_back(thread_pool_.enqueue(f));
    }

    // Process results of both threads.
    for (size_t frame_idx = 0u; frame_idx < ref_nframe.size(); ++frame_idx)
    {
      const ProjectionResult& res = results[frame_idx].get();
      postprocessTrackingResults(frame_idx, res);
      num_successful_tracked += res.successful_projections.size();
    }
  }
  else
  {
    for (size_t frame_idx = 0u; frame_idx < ref_nframe.size(); ++frame_idx)
    {
      ProjectionResult res = projectLandmarksAndLKTracking(
                               frame_idx, frame_idx, rig_,
                               T_Bref_W, T_Bcur_W, landmarks_, keypoint_margin_,
                               ref_nframe, cur_nframe);
      postprocessTrackingResults(frame_idx, res);
      num_successful_tracked += res.successful_projections.size();
    }
  }
  return num_successful_tracked;
}

// -----------------------------------------------------------------------------
real_t FeatureTracker::getMedianDisparity(
    NFrame& nframe_ref,
    NFrame& nframe_cur)
{
  std::vector<real_t> disparities;
  for (size_t frame_idx = 0u; frame_idx < nframe_ref.size(); ++frame_idx)
  {
    Frame& frame_ref = nframe_ref.at(frame_idx);
    Frame& frame_cur = nframe_cur.at(frame_idx);

    // Find matching indices.
    const std::vector<std::pair<uint32_t, uint32_t>> indices_cur_ref =
        getMatchIndices<LandmarkHandle::value_t>(
          frame_cur.getLandmarkHandlesAsVector(),
          frame_ref.getLandmarkHandlesAsVector(),
          std::bind(&isValidLandmarkHandleType, std::placeholders::_1));

    // Compute disparities.
    disparities.reserve(disparities.size() + indices_cur_ref.size());
    for (const std::pair<uint32_t, uint32_t>& match : indices_cur_ref)
    {
      disparities.push_back(
            (frame_ref.px_vec_.col(match.second)
             - frame_cur.px_vec_.col(match.first)).norm());
    }
  }

  // Compute median.
  std::pair<real_t, bool> res = median(disparities);
  if (!res.second)
  {
    LOG(ERROR) << "Have no feature matches to compute disparity.";
    return 0.0;
  }
  return res.first;
}

// -----------------------------------------------------------------------------
std::pair<std::vector<real_t>, uint32_t> FeatureTracker::outlierRemoval(
    NFrame& nframe_ref,
    NFrame& nframe_cur,
    const Transformation& T_Bcur_Bref)
{
  std::vector<real_t> disparities_sq;
  uint32_t num_outliers = 0u;
  //if (rig_.size() == 2u) //! @todo: Implement for more than two cameras.
  //{
   // num_outliers += stereo_matcher_.stereoOutlierRejection(nframe_cur).second;
  //}
  for (size_t frame_idx = 0u; frame_idx < nframe_ref.size(); ++frame_idx)
  {
    Transformation T_cur_ref = T_Bcur_Bref;
    num_outliers += ransacRelativePoseOutlierRejection(
                      frame_idx, frame_idx, nframe_ref, nframe_cur,
                      disparities_sq, T_cur_ref).first;
  }
  return std::make_pair(disparities_sq, num_outliers);
}

// -----------------------------------------------------------------------------
std::pair<uint32_t, IndexMatches> FeatureTracker::ransacRelativePoseOutlierRejection(
    uint32_t ref_idx, uint32_t cur_idx,
    NFrame& nframe_ref, NFrame& nframe_cur,
    std::vector<real_t>& disparities_sq,
    Transformation& T_Bcur_Bref)
{
  Frame& frame_ref = nframe_ref.at(ref_idx);
  Frame& frame_cur = nframe_cur.at(cur_idx);

  // Find matching indices.
  auto indices_cur_ref = getMatchIndices<LandmarkHandle::value_t>(
        frame_cur.getLandmarkHandlesAsVector(),
        frame_ref.getLandmarkHandlesAsVector(),
        std::bind(&isValidLandmarkHandleType, std::placeholders::_1));

  // Collect all matching bearing vectors.
  BearingsVector f_cur, f_ref;
  f_cur.reserve(indices_cur_ref.size());
  f_ref.reserve(indices_cur_ref.size());
  for (const std::pair<uint32_t, uint32_t>& it : indices_cur_ref)
  {
    f_cur.push_back(frame_cur.f_vec_.col(it.first).cast<double>());
    f_ref.push_back(frame_ref.f_vec_.col(it.second).cast<double>());
  }

  // Compute transformation in camera coordinates
  Transformation T_Ccur_Cref = T_C_B_[cur_idx] * T_Bcur_Bref * T_C_B_[ref_idx].inverse();

  // Run RANSAC.
  RansacRelativePose ransac(rig_.at(ref_idx), reprojection_error_threshold_px_);
  auto algorithm = use_5pt_ransac_ ? RelativePoseAlgorithm::FivePoint
                                   : RelativePoseAlgorithm::TwoPointTranslationOnly;
  bool success = ransac.solve(f_ref, f_cur, algorithm, T_Ccur_Cref);
  const std::vector<int>& inliers = ransac.inliers();
  VLOG(3) << "RANSAC: success = " << success
          << ", #inliers = " << inliers.size();

  // Update in body-coordinate frame:
  T_Bcur_Bref = T_C_B_[cur_idx].inverse() * T_Ccur_Cref * T_C_B_[ref_idx];

  // Compute disparities.
  disparities_sq.reserve(disparities_sq.size() + inliers.size());
  for (int i : inliers)
  {
    uint32_t i_ref, i_cur;
    std::tie(i_cur, i_ref) = indices_cur_ref.at(i);
    disparities_sq.push_back(
          (frame_ref.px_vec_.col(i_ref) - frame_cur.px_vec_.col(i_cur)).squaredNorm());
  }

  // Remove outliers.
  std::vector<int> outliers = ransac.outliers();
  for (int i : outliers)
  {
    auto cr = indices_cur_ref.at(i);
    LandmarkHandle& h_ref = frame_ref.landmark_handles_[cr.second];
    LandmarkHandle& h_cur = frame_cur.landmark_handles_[cr.first];
    DEBUG_CHECK_EQ(h_ref, h_cur);
    landmarks_.type(h_ref) = LandmarkType::Outlier;
    markLandmarkHandleAsOutlier(h_ref);
    markLandmarkHandleAsOutlier(h_cur);
  }

  return std::make_pair(outliers.size(), indices_cur_ref);
}

void FeatureTracker::setUse5PtRansac(bool use_5pt_ransac)
{
  use_5pt_ransac_ = use_5pt_ransac;
}

} // namespace ze
