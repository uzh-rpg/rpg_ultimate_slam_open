#pragma once

#include <vector>
#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>
#include <imp/core/image.hpp>
#include <imp/imgproc/image_pyramid.hpp>

namespace ze {

// fwd
class Camera;

enum class KltResult : uint8_t
{
  Converged = 0,
  Stopped_MaxIter = 1,
  Stopped_NotWithinMargin = 2,
  Stopped_RefPatchNotWithinMargin = 3,
  Stopped_NaN = 4,
  Invalid_PointBehindCamera = 5
};
using KltResults = std::vector<KltResult>; // TODO: make Eigen Vector!

struct KltParameters
{
  int termcrit_n_iter = 10;
  int border_margin = 0;
  real_t termcrit_min_update_squared = 0.03 * 0.03;

  KltParameters() = default;
  KltParameters(int max_iter,
                real_t min_update_squared)
    : termcrit_n_iter(max_iter)
    , termcrit_min_update_squared(min_update_squared)
  {}
};

//! Alignment of a single feature.
KltResult alignFeature(
    const Image8uC1& img_ref,
    const Image8uC1& img_cur,
    const int patch_size_by8,
    const KltParameters& params,
    const Keypoint& px_ref,
    Keypoint& px_cur,
    bool use_simd = true);

//! Alignment of multiple features.
KltResults alignFeatures(
    const Image8uC1& img_ref,
    const Image8uC1& img_cur,
    const int patch_size_by8,
    const KltParameters& params,
    const Keypoints& px_ref_vec,
    Keypoints& px_cur_vec);

//! Pyramidal alignment of multiple features.
KltResults alignFeaturesPyr(
    const ImagePyramid8uC1& pyr_ref,
    const ImagePyramid8uC1& pyr_cur,
    const std::vector<int> patch_sizes_by8,
    const KltParameters& params,
    const Keypoints& px_ref_vec,
    Keypoints& px_cur_vec);

#if 0
//! For reference, the KLT implementation of OpenCv with the same interface as ours.
KltResults alignFeaturesGuidedPyrOpenCv(
    const ImagePyramid8uC1& pyr_ref,
    const ImagePyramid8uC1& pyr_cur,
    const Camera& /*cam_ref*/,
    const Camera& cam_cur,
    const std::vector<int> /*patch_sizes_by8*/,
    const KltParameters& /*params*/,
    const bool /*affine_warp*/,
    const Transformation& T_cur_ref_prior,
    const Keypoints& px_ref_vec,
    const Bearings& f_ref_vec,
    const VectorX& inv_depth_ref_vec,
    Keypoints& px_cur_vec);
#endif

//! Predicts pose of features using relative motion prior (e.g. from IMU).
//! Warps the patches according to the motion before pyramidal alignment.
KltResults alignFeaturesGuidedPyr(
    const ImagePyramid8uC1& pyr_ref,
    const ImagePyramid8uC1& pyr_cur,
    const Camera& cam_ref,
    const Camera& cam_cur,
    const std::vector<int> patch_sizes_by8,
    const KltParameters& params,
    const bool affine_warp,
    const Transformation& T_cur_ref_prior,
    const Keypoints& px_ref_vec,
    const Bearings& f_ref_vec,
    const VectorX& invdepth_vec,
    Keypoints& px_cur_vec);

#ifdef __ARM_NEON__
KltResult alignPatchNeon(
    const KltParameters& params,
    const uint8_t* __restrict__ img_data,
    const uint8_t* __restrict__ patch,
    const int16_t* __restrict__ patch_dx_x2,
    const int16_t* __restrict__ patch_dy_x2,
    const int img_width,
    const int img_height,
    const int img_stride,
    const int patch_size_by8,
    Eigen::Ref<Keypoint> cur_px_estimate);
#endif

KltResult alignPatchImpl(
    const KltParameters& params,
    const uint8_t* __restrict__ img_data,
    const uint8_t* __restrict__ patch,
    const int16_t* __restrict__ patch_dx_x2,
    const int16_t* __restrict__ patch_dy_x2,
    const int img_width,
    const int img_height,
    const int img_stride,
    const int patch_size_by8,
    Eigen::Ref<Keypoint> cur_px_estimate);

//! Lucas-Kanade alignment of a patch. Used by all the above functions.
//! Patch derivatives dx, dy must be provided with an extra factor of 2.0.
inline KltResult alignPatch(
    const KltParameters& params,
    const uint8_t* __restrict__ img_data,
    const uint8_t* __restrict__ patch,
    const int16_t* __restrict__ patch_dx_x2,
    const int16_t* __restrict__ patch_dy_x2,
    const int img_width,
    const int img_height,
    const int img_stride,
    const int patch_size_by_8,
    Eigen::Ref<Keypoint> cur_px_estimate,
    bool use_simd = true)
{
#ifdef __ARM_NEON__
  if (use_simd)
  {
    return alignPatchNeon(
          params, img_data, patch, patch_dx_x2, patch_dy_x2, img_width,
          img_height, img_stride, patch_size_by_8, cur_px_estimate);
  }
#endif
  (void)use_simd;
  return alignPatchImpl(
        params, img_data, patch, patch_dx_x2, patch_dy_x2, img_width,
        img_height, img_stride, patch_size_by_8, cur_px_estimate);
}

} // namespace ze
