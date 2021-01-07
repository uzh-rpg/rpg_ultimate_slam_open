#include <imp/correspondence/sparse_stereo.hpp>

#include <imp/correspondence/epipolar_matcher.hpp>
#include <ze/cameras/camera.hpp>
#include <ze/cameras/camera_utils.hpp>
#include <ze/geometry/triangulation.hpp>

namespace ze {

SparseStereoMatcher::SparseStereoMatcher(
    const Camera& cam_ref,
    const Camera& cam_cur,
    const ImagePyramid8uC1& pyr_ref,
    const ImagePyramid8uC1& pyr_cur,
    const Transformation& T_cur_ref,
    const Eigen::Ref<const Keypoints>& px_ref,
    const Eigen::Ref<const Bearings>& f_ref,
    const Eigen::Ref<const KeypointLevels>& levels_ref,
    const SparseStereoMatcherOptions options)
  : options_(options)
  , cam_ref_(cam_ref)
  , cam_cur_(cam_cur)
  , pyr_ref_(pyr_ref)
  , pyr_cur_(pyr_cur)
  , T_cur_ref_(T_cur_ref)
  , px_ref_(px_ref)
  , f_ref_(f_ref)
  , levels_ref_(levels_ref)
  , px_cur_(2, px_ref_.cols())
  , levels_cur_(px_ref_.cols())
{
  CHECK_EQ(px_ref_.cols(), f_ref_.cols());
  CHECK_EQ(px_ref_.cols(), levels_ref_.size());
}

uint32_t SparseStereoMatcher::computeMatches(
    const real_t inv_depth_ref_estimate,
    const real_t inv_depth_ref_min,
    const real_t inv_depth_ref_max)
{
  const real_t img_width = cam_cur_.width();
  const real_t img_height = cam_cur_.height();
  uint32_t n_matches = 0u;
  uint32_t n_not_visible = 0u;
  inliers_.reserve(px_ref_.cols());

  switch (options_.algorithm)
  {
    case SparseStereoAlgorithm::EpipolarSampling:
    {
      EpipolarMatcherOptions matcher_options;
      matcher_options.subpix_refinement = false;
      matcher_options.max_epi_search_steps = 200;
      EpipolarMatcher matcher;
      for (int i = 0; i < px_ref_.cols(); ++i)
      {
        Keypoint px_cur;
        EpipolarMatchResult res;
        //! @todo: should directly use options_.margin_at_level in epipolar sampler.
        std::tie(px_cur, res) = matcher.findEpipolarMatchDirect(
              T_cur_ref_, pyr_ref_, pyr_cur_, cam_ref_, cam_cur_,
              px_ref_.col(i), f_ref_.col(i), levels_ref_(i),
              inv_depth_ref_estimate, inv_depth_ref_min, inv_depth_ref_max);

        if (res == EpipolarMatchResult::Success)
        {
          if (!isVisibleWithMargin(img_width, img_height, px_cur,
                                  options_.margin_at_level * (matcher.level_cur_ + 1)))
          {
            ++n_not_visible;
            continue;
          }
          inliers_.push_back(i);
          px_cur_.col(n_matches) = px_cur;
          levels_cur_(n_matches) = matcher.level_cur_;
          ++n_matches;
        }
      }
      break;
    }

    case SparseStereoAlgorithm::PyramidalKLT:
    {
      KltParameters klt_params;
      klt_params.termcrit_n_iter = 20;
      klt_params.termcrit_min_update_squared = 0.01 * 0.01;
      klt_params.border_margin = options_.margin_at_level;
      bool affine_warp = false;
      std::vector<int> patch_sizes_by8 = {2, 2, 3, 3, 3};
      VectorX inv_depth_estimate(px_ref_.cols());
      Keypoints px_cur(2, px_ref_.cols());
      inv_depth_estimate.setConstant(inv_depth_ref_estimate);
      KltResults res = alignFeaturesGuidedPyr(
            pyr_ref_, pyr_cur_, cam_ref_, cam_cur_, patch_sizes_by8, klt_params,
            affine_warp, T_cur_ref_, px_ref_, f_ref_, inv_depth_estimate, px_cur);
      for (int i = 0; i < px_ref_.cols(); ++i)
      {
        if (res[i] == KltResult::Converged)
        {
          inliers_.push_back(i);
          px_cur_.col(n_matches) = px_cur.col(i);
          levels_cur_(n_matches) = levels_ref_(i);
          ++n_matches;
        }
      }
      break;
    }
    default:
      LOG(FATAL) << "Algorithm not implemented.";
  }

  CHECK_EQ(inliers_.size(), n_matches);
  px_cur_.conservativeResize(2, n_matches);
  levels_cur_.conservativeResize(n_matches);
  VLOG_IF(20, n_not_visible) << "Features too close to border = " << n_not_visible;
  return n_matches;
}

uint32_t SparseStereoMatcher::triangulateMatches(
    const real_t max_reprojection_error_px,
    const real_t min_distance,
    const real_t max_distance)
{
  f_cur_ = cam_cur_.backProjectVectorized(px_cur_);

  // Compute reprojection errors:
  uint32_t n_matches = inliers_.size();
  Bearings f_ref_matched(3, f_cur_.cols());
  for (uint32_t i = 0; i < n_matches; ++i)
  {
    f_ref_matched.col(i) = f_ref_.col(inliers_.at(i));
  }
  VectorX reprojection_errors(f_cur_.cols());
  xyz_cur_.resize(3, f_cur_.cols());
  triangulateManyAndComputeAngularErrors(T_cur_ref_, f_cur_, f_ref_matched,
                                         xyz_cur_, reprojection_errors);

  // Compute triangulation inliers:
  real_t thresh = 1.0 - std::cos(cam_cur_.getApproxAnglePerPixel()
                                    * max_reprojection_error_px);
  std::vector<uint32_t> triangulation_inliers;
  triangulation_inliers.reserve(n_matches);
  std::vector<uint32_t> new_inliers;
  new_inliers.reserve(n_matches);
  for (uint32_t i = 0; i < n_matches; ++i)
  {
    auto p = xyz_cur_.col(i);
    if (reprojection_errors(i) < thresh
        && p.z() > min_distance //! @todo: deal with omnidirectional cameras.
        && p.norm() < max_distance)
    {
      triangulation_inliers.push_back(i);
      new_inliers.push_back(inliers_.at(i));
    }
  }

  px_cur_ = getMatrixCols(px_cur_, triangulation_inliers);
  f_cur_ = getMatrixCols(f_cur_, triangulation_inliers);
  xyz_cur_ = getMatrixCols(xyz_cur_, triangulation_inliers);
  levels_cur_ = getVectorElements(levels_cur_, triangulation_inliers);
  inliers_ = new_inliers;

  return inliers_.size();
}

} // namespace ze
