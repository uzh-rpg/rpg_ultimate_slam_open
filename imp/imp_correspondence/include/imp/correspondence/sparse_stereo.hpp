#pragma once

#include <imp/imgproc/image_pyramid.hpp>
#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>

namespace ze {

// fwd
class Camera;

enum class SparseStereoAlgorithm
{
  EpipolarSampling,
  PyramidalKLT
};

struct SparseStereoMatcherOptions
{
  //! Matching algorithm that should be used.
  SparseStereoAlgorithm algorithm = SparseStereoAlgorithm::EpipolarSampling;

  //! Reject matches that are too close to border.
  real_t margin_at_level = 0.0;
};

class SparseStereoMatcher
{
public:
  SparseStereoMatcher(
      const Camera& cam_ref,
      const Camera& cam_cur,
      const ImagePyramid8uC1& pyr_ref,
      const ImagePyramid8uC1& pyr_cur,
      const Transformation& T_cur_ref,
      const Eigen::Ref<const Keypoints>& px_ref,
      const Eigen::Ref<const Bearings>& f_ref,
      const Eigen::Ref<const KeypointLevels>& levels_ref,
      const SparseStereoMatcherOptions options = SparseStereoMatcherOptions());

  //! Populates px_cur_, levels_cur and inliers_
  uint32_t computeMatches(
      const real_t inv_depth_ref_estimate,
      const real_t inv_depth_ref_min,
      const real_t inv_depth_ref_max);

  //! Populates xyz_cur_, f_cur_ and updates inliers_
  uint32_t triangulateMatches(
      const real_t max_reprojection_error_px,
      const real_t min_distance,
      const real_t max_distance);

private:
  // Settings
  SparseStereoMatcherOptions options_;

  // Data
  const Camera& cam_ref_;
  const Camera& cam_cur_;
  const ImagePyramid8uC1& pyr_ref_;
  const ImagePyramid8uC1& pyr_cur_;
  const Transformation& T_cur_ref_;

  // Reference Keypoints
  const Eigen::Ref<const Keypoints>& px_ref_;
  const Eigen::Ref<const Bearings>& f_ref_;
  const Eigen::Ref<const KeypointLevels>& levels_ref_;

public:
  //! Maps input indices (px_ref) to inlier-output indices
  std::vector<uint32_t> inliers_;
  Keypoints px_cur_;
  Bearings f_cur_;              //!< Only available after triangulation.
  //! the landmark positions in cur frame
  Positions xyz_cur_;           //!< Dito. @todo: make proper interface.
  KeypointLevels levels_cur_;
};

} // namespace ze
