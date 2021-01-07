#pragma once

#include <array>
#include <vector>

#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/imgproc/image_pyramid.hpp>
#include <imp/correspondence/klt.hpp>

namespace ze {

// fwd
template<int HalfPatchSize> class ZMSSD;
class Camera;

enum class EpipolarMatchResult : uint8_t {
  Success,
  FailScore,
  FailWarp,
  FailAlignment,
  FailVisibilityReference,
  FailVisibilityCurrent
};

std::string stringFromEpipolarMatchResult(const EpipolarMatchResult res);

struct EpipolarMatcherOptions
{
  //! Number of iterations for aligning the feature patches in Gauss Newton.
  KltParameters subpix_parameters {10, 0.03 * 0.03};

  //! Max length of epipolar line to skip epipolar search and do supix refinement.
  real_t max_epi_length_optim = 2.0;

  //! Perform Lucas-Kanade patch alignment at the end to get subpix precision.
  bool subpix_refinement = true;

  //! Max number of evaluations along epipolar line.
  int max_epi_search_steps = 100;

  //! Margin of the result.
  int margin_cur_level0 = 4;
};

//! Patch-matcher for reprojection-matching and epipolar search in triangulation.
class EpipolarMatcher
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int c_patch_size_by_8_ = 1;
  static constexpr int c_patch_size_ = c_patch_size_by_8_ * 8;
  static constexpr int c_halfpatch_size_ = c_patch_size_by_8_ * 4;
  static constexpr int c_max_patch_size_by_8_ = 3;

  using PatchScore = ZMSSD<c_halfpatch_size_>;

  EpipolarMatcherOptions options_;
  Matrix2 A_cur_ref_;
  int level_cur_ = 0;

  //! Constructor allocates patch data.
  EpipolarMatcher(EpipolarMatcherOptions options = EpipolarMatcherOptions());

  //! Find a matching patch by searching along the epipolar line.
  //! Returns estimated depth in reference frame and matching result.
  std::pair<Keypoint, EpipolarMatchResult> findEpipolarMatchDirect(
      const Transformation& T_cur_ref,
      const ImagePyramid8uC1& pyr_ref,
      const ImagePyramid8uC1& pyr_cur,
      const Camera& cam_ref,
      const Camera& cam_cur,
      const Eigen::Ref<const Keypoint> px_ref,
      const Eigen::Ref<const Bearing> f_ref,
      const uint32_t level_ref,
      const real_t inv_depth_ref_estimate,
      const real_t inv_depth_ref_min,
      const real_t inv_depth_ref_max);

  //! Run KLT alignment of small patch to obtain subpixel-precise correspondence.
  EpipolarMatchResult refineMatchDirect(
      const Transformation& T_cur_ref,
      const ImagePyramid8uC1& pyr_ref,
      const ImagePyramid8uC1& pyr_cur,
      const Camera& cam_ref,
      const Camera& cam_cur,
      const Eigen::Ref<const Keypoint> px_ref,
      const Eigen::Ref<const Bearing> f_ref,
      const uint32_t level_ref,
      const real_t inv_depth_ref_estimate,
      Keypoint& px_cur,
      const uint32_t num_levels = 1);

  //! Utility function to debug patch warping.
  void displayMatch(
      const ImagePyramid8uC1& pyr_cur,
      const Keypoint& px_cur,
      const uint32_t level_cur,
      const ImagePyramid8uC1& pyr_ref,
      const Keypoint& px_ref,
      const uint32_t level_ref,
      const Eigen::Ref<const Bearing> f_ref,
      const Transformation& T_cur_ref,
      const real_t inv_depth_ref_estimate,
      const Camera& cam_cur);

private:
  //! Patch-sizes used for pyramidal alignment.
  std::array<int, 4> patch_sizes_by8_ {{ 1, 2, 2, 2 }};

  //! Managed and 32-bit aligned patch data.
  ImageRaw8uC1  patch_data_;

  //! @name Pointers to rows in patch_data_.
  //! @{
  uint8_t* patch_with_border_;
  uint8_t* patch_;
  int16_t* patch_dx_;
  int16_t* patch_dy_;
  //! @}

  //! Run KLT on patch. Assumes patch is already extracted.
  bool subpixelPatchAlignment(
      const Image8uC1& img_cur,
      Eigen::Ref<Keypoint> px_cur_lev,
      int patch_size_by_8);

  //! Scan epipolar line with patch. Assumes patch is already extracted.
  std::pair<Vector2i, bool> findMaxScoreOnEpipolarLine(
      const Bearing& f_A,
      const Bearing& f_B,
      const Bearing& f_C,
      const Image8uC1& img_cur,
      const Camera& cam_cur,
      real_t scale_cur);
};

// convenience typedefs
using EpipolarMatcherVector = std::vector<EpipolarMatcher>;

} // namespace ze
