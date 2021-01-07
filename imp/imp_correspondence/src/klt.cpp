#include <imp/correspondence/klt.hpp>

#include <algorithm>
#ifdef __SSE2__
# include <emmintrin.h>
#endif
#ifdef __ARM_NEON__
# include <arm_neon.h>
#endif
#include <ze/common/logging.hpp>
#include <opencv2/video/tracking.hpp>
#include <ze/cameras/camera_utils.hpp>
#include <imp/bridge/opencv/image_cv.hpp>
#include <imp/correspondence/patch_utils.hpp>
#include <imp/core/image_raw.hpp>

namespace ze {

//------------------------------------------------------------------------------
KltResult alignFeature(
    const Image8uC1& img_ref,
    const Image8uC1& img_cur,
    const int patch_size_by8,
    const KltParameters& params,
    const Keypoint& px_ref,
    Keypoint& px_cur,
    bool use_simd)
{
  const int patch_size = patch_size_by8 * 8;
  const int halfpatch_with_border_size = patch_size_by8 * 4 + 1;
  const int patch_with_border_size = 2 * halfpatch_with_border_size;

  // Allocate patch data.
  ImageRaw8uC1  patch(patch_with_border_size * patch_with_border_size, 2);
  ImageRaw16sC1 patch_dxdy(patch_size * patch_size, 2);

  // Extract patch with border.
  // We round the keypoint to the closest int, so we can avoid using bilinear
  // interpolation when extracting the reference patch. At the end, we add the
  // offset to the closest int to the result again.
  Vector2i px_ref_rounded = (px_ref + Keypoint(0.5, 0.5)).cast<int>();
  Vector2 px_ref_offset = px_ref - px_ref_rounded.cast<real_t>();

  // Check visibility of reference patch:
  if(!isVisibleWithMargin(
       img_ref.width(), img_ref.height(), px_ref_rounded, halfpatch_with_border_size))
  {
    return KltResult::Stopped_RefPatchNotWithinMargin;
  }

  extractPatch8uC1(
        img_ref, px_ref_rounded, halfpatch_with_border_size, patch.data(0, 0));


  // Compute patch derivatives.
  computePatchDerivative8uC1(patch_size_by8 * 2,
                             reinterpret_cast<uint8_t*>(patch.data(0, 0)),
                             reinterpret_cast<int16_t*>(patch_dxdy.data(0, 0)),
                             reinterpret_cast<int16_t*>(patch_dxdy.data(0, 1)));

  // Create patch without border.
  removeBorderFromPatch8uC1(patch_size,
                        reinterpret_cast<uint8_t*>(patch.data(0, 0)),
                        reinterpret_cast<uint8_t*>(patch.data(0, 1)));

  // Align patch.
  auto res = alignPatch(
               params,
               reinterpret_cast<const uint8_t*>(img_cur.data()),
               reinterpret_cast<const uint8_t*>(patch.data(0, 1)),
               reinterpret_cast<const int16_t*>(patch_dxdy.data(0, 0)),
               reinterpret_cast<const int16_t*>(patch_dxdy.data(0, 1)),
               img_cur.width(), img_cur.height(), img_cur.stride(),
               patch_size_by8,
               px_cur,
               use_simd);

  // Account for offset.
  px_cur += px_ref_offset;
  return res;
}

//------------------------------------------------------------------------------
KltResults alignFeatures(
    const Image8uC1& img_ref,
    const Image8uC1& img_cur,
    const int patch_size_by8,
    const KltParameters& params,
    const Keypoints& px_ref_vec,
    Keypoints& px_cur_vec)
{
  CHECK_EQ(px_ref_vec.cols(), px_cur_vec.cols());

  const int patch_size = patch_size_by8 * 8;
  const int halfpatch_with_border_size = patch_size_by8 * 4 + 1;
  const int patch_with_border_size = 2 * halfpatch_with_border_size;
  const int n = px_ref_vec.cols();

  // Allocate patch data.
  ImageRaw8uC1  patch(patch_with_border_size * patch_with_border_size, 2);
  ImageRaw16sC1 patch_dxdy(patch_size * patch_size, 2);

  // Allocate result vector:
  KltResults result(n);

  const uint8_t* img_data = reinterpret_cast<const uint8_t*>(img_cur.data());
  const int img_width = img_cur.width();
  const int img_height = img_cur.height();
  const int img_stride = img_cur.stride();
  for(int i = 0; i < n; ++i)
  {
    // Extract patch with border.
    // We round the keypoint to the closest int, so we can avoid using bilinear
    // interpolation when extracting the reference patch. At the end, we add the
    // offset to the closest int to the result again.
    Vector2i px_ref_rounded = (px_ref_vec.col(i) + Keypoint(0.5, 0.5)).cast<int>();
    Vector2  px_ref_offset = px_ref_vec.col(i) - px_ref_rounded.cast<real_t>();

    // Check visibility of reference patch:
    if(!isVisibleWithMargin(
         img_ref.width(), img_ref.height(), px_ref_rounded, halfpatch_with_border_size))
    {
      result[i] = KltResult::Stopped_RefPatchNotWithinMargin;
      continue;
    }

    extractPatch8uC1(
          img_ref, px_ref_rounded, halfpatch_with_border_size, patch.data(0, 0));


    // Compute patch derivatives.
    computePatchDerivative8uC1(patch_size_by8 * 2,
                               reinterpret_cast<uint8_t*>(patch.data(0, 0)),
                               reinterpret_cast<int16_t*>(patch_dxdy.data(0, 0)),
                               reinterpret_cast<int16_t*>(patch_dxdy.data(0, 1)));

    // Create patch without border.
    removeBorderFromPatch8uC1(patch_size,
                          reinterpret_cast<uint8_t*>(patch.data(0, 0)),
                          reinterpret_cast<uint8_t*>(patch.data(0, 1)));

    // Align patch.
    result[i] = alignPatch(
                  params,
                  img_data,
                  reinterpret_cast<const uint8_t*>(patch.data(0, 1)),
                  reinterpret_cast<const int16_t*>(patch_dxdy.data(0, 0)),
                  reinterpret_cast<const int16_t*>(patch_dxdy.data(0, 1)),
                  img_width, img_height, img_stride,
                  patch_size_by8,
                  px_cur_vec.col(i));

    // Account for offset.
    px_cur_vec.col(i) += px_ref_offset;
  }
  return result;
}

//------------------------------------------------------------------------------
KltResults alignFeaturesPyr(
    const ImagePyramid8uC1& pyr_ref,
    const ImagePyramid8uC1& pyr_cur,
    const std::vector<int> patch_sizes_by8,
    const KltParameters& params,
    const Keypoints& px_ref_vec,
    Keypoints& px_cur_vec)
{
  CHECK_EQ(px_ref_vec.cols(), px_cur_vec.cols());

  // Allocate patch data.
  const int max_patch_size =
      (*std::max_element(patch_sizes_by8.begin(), patch_sizes_by8.end())) * 8;
  ImageRaw8uC1  patch((max_patch_size + 2) * (max_patch_size + 2), 2);
  ImageRaw16sC1 patch_dxdy(max_patch_size * max_patch_size, 2);

  // Allocate result vector:
  const int n = px_ref_vec.cols();
  KltResults result(n, KltResult::Converged);

  // Run pyramidal lucas kanade. Start at highest level.
  for(int level = static_cast<int>(patch_sizes_by8.size()) - 1; level >= 0; --level)
  {
    const Image8uC1& img_ref = pyr_ref.at(level);
    const Image8uC1& img_cur = pyr_cur.at(level);
    const uint8_t* img_data = reinterpret_cast<const uint8_t*>(img_cur.data());
    const int img_width = img_cur.width();
    const int img_height = img_cur.height();
    const int img_stride = img_cur.stride();
    const int patch_size_by8 = patch_sizes_by8.at(level);
    const int patch_size = patch_size_by8 * 8;
    const int halfpatch_size = patch_size >> 1;
    real_t scale = pyr_ref.scaleFactor(level);

    for(int i = 0; i < n; ++i)
    {
      if(result[i] == KltResult::Stopped_NaN || result[i] == KltResult::Stopped_MaxIter)
      {
        // If the alignment did not converge or is NaN, when processing this patch
        // the last time, then there is no reason to go to the next level.
        continue;
      }

      // Extract patch with border.
      // We round the keypoint to the closest int, so we can avoid using bilinear
      // interpolation when extracting the reference patch. At the end, we add the
      // offset to the closest int to the result again.
      Keypoint px_ref = px_ref_vec.col(i) * scale;
      Vector2i px_ref_rounded = (px_ref + Keypoint(0.5, 0.5)).cast<int>();
      Vector2  px_ref_offset = px_ref - px_ref_rounded.cast<real_t>();

      // Check visibility of reference patch:
      if(!isVisibleWithMargin(
           img_ref.width(), img_ref.height(), px_ref_rounded, halfpatch_size + 1))
      {
        result[i] = KltResult::Stopped_RefPatchNotWithinMargin;
        continue;
      }

      extractPatch8uC1(
            img_ref, px_ref_rounded, halfpatch_size + 1, patch.data(0, 0));


      // Compute patch derivatives.
      computePatchDerivative8uC1(patch_size_by8 * 2,
                                 reinterpret_cast<const uint8_t*>(patch.data(0, 0)),
                                 reinterpret_cast<int16_t*>(patch_dxdy.data(0, 0)),
                                 reinterpret_cast<int16_t*>(patch_dxdy.data(0, 1)));

      // Create patch without border.
      removeBorderFromPatch8uC1(patch_size,
                            reinterpret_cast<uint8_t*>(patch.data(0, 0)),
                            reinterpret_cast<uint8_t*>(patch.data(0, 1)));

      // Align patch.
      Keypoint px_cur = px_cur_vec.col(i) * scale;
      result[i] = alignPatch(
                    params,
                    img_data,
                    reinterpret_cast<const uint8_t*>(patch.data(0, 1)),
                    reinterpret_cast<const int16_t*>(patch_dxdy.data(0, 0)),
                    reinterpret_cast<const int16_t*>(patch_dxdy.data(0, 1)),
                    img_width, img_height, img_stride,
                    patch_size_by8,
                    px_cur);

      // Account for offset and scale.
      px_cur_vec.col(i) = (px_cur + px_ref_offset) / scale;
    }
  }
  return result;
}

#if 0
//------------------------------------------------------------------------------
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
    Keypoints& px_cur_vec)
{
  Bearings f_cur_predicted =
      T_cur_ref_prior.getRotation().rotateVectorized(f_ref_vec)
      + T_cur_ref_prior.getPosition() * inv_depth_ref_vec.transpose();
  px_cur_vec = cam_cur.projectVectorized(f_cur_predicted);

  const int klt_win_size = 21;
  const int klt_max_iter = 30;
  const double klt_eps = 0.001;
  std::vector<uchar> status;
  std::vector<float> error;
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);
  std::vector<cv::Point2f> px_ref, px_cur;
  px_ref.reserve(px_ref_vec.cols());
  px_cur.reserve(px_cur_vec.cols());
  const uint32_t img_width = cam_cur.width();
  const uint32_t img_height = cam_cur.height();
  for (int i = 0; i < px_ref_vec.cols(); ++i)
  {
    px_ref.push_back(cv::Point2f(px_ref_vec(0,i), px_ref_vec(1,i)));
    px_cur.push_back(cv::Point2f(px_cur_vec(0,i), px_cur_vec(1,i)));
  }
  cv::calcOpticalFlowPyrLK(ImageCv8uC1(pyr_ref.at(0)).cvMat(),
                           ImageCv8uC1(pyr_cur.at(0)).cvMat(),
                           px_ref, px_cur, status, error,
                           cv::Size2i(klt_win_size, klt_win_size),
                           4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);
  KltResults res;
  res.reserve(px_ref_vec.cols());
  for(size_t i = 0; i < status.size(); ++i)
  {
    if(status[i] && isVisibleWithMargin(img_width, img_height, px_cur.at(i).x, px_cur.at(i).y, 5.0))
    {
      px_cur_vec.col(i) = Keypoint(px_cur.at(i).x, px_cur.at(i).y);
      res.push_back(KltResult::Converged);
    }
    else
    {
      res.push_back(KltResult::Stopped_MaxIter);
    }
  }
  return res;
}
#endif

//------------------------------------------------------------------------------
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
    const VectorX& inv_depth_ref_vec,
    Keypoints& px_cur_vec)
{
  CHECK_EQ(px_ref_vec.cols(), px_cur_vec.cols());
  CHECK_EQ(px_ref_vec.cols(), f_ref_vec.cols());
  CHECK_EQ(px_ref_vec.cols(), inv_depth_ref_vec.size());

  // Allocate patch data.
  const int max_patch_size =
      (*std::max_element(patch_sizes_by8.begin(), patch_sizes_by8.end())) * 8;
  ImageRaw8uC1  patch((max_patch_size + 2) * (max_patch_size + 2), 2);
  ImageRaw16sC1 patch_dxdy(max_patch_size * max_patch_size, 2);
  uint8_t* patch_data = reinterpret_cast<uint8_t*>(patch.data());

  // Allocate result vector:
  const int n = px_ref_vec.cols();
  KltResults result(n, KltResult::Converged);

  // Predict feature position: Using inverse depth coordinates we don't need to
  // explicitly compute the 3D point. Should numerically be more stable for far points.
  Bearings f_cur_predicted =
      T_cur_ref_prior.getRotation().rotateVectorized(f_ref_vec)
      + T_cur_ref_prior.getPosition() * inv_depth_ref_vec.transpose();
  px_cur_vec = cam_cur.projectVectorized(f_cur_predicted);

  // Run pyramidal lucas kanade. Start at highest level.
  for(int level = static_cast<int>(patch_sizes_by8.size()) - 1; level >= 0; --level)
  {
    const Image8uC1& img_ref = pyr_ref.at(level);
    const Image8uC1& img_cur = pyr_cur.at(level);
    const uint8_t* img_data = reinterpret_cast<const uint8_t*>(img_cur.data());
    const int img_width = img_cur.width();
    const int img_height = img_cur.height();
    const int img_stride = img_cur.stride();
    const int patch_size_by8 = patch_sizes_by8.at(level);
    const int patch_size = patch_size_by8 * 8;
    const int halfpatch_size = patch_size >> 1;
    real_t scale = pyr_ref.scaleFactor(level);
    const int img_ref_width = img_ref.width();
    const int img_ref_height = img_ref.height();
    const int img_ref_stride = img_ref.stride();
    const uint8_t* img_ref_data = reinterpret_cast<const uint8_t*>(img_ref.data());

    for(int i = 0; i < n; ++i)
    {
      if(result[i] == KltResult::Stopped_NaN
         || result[i] == KltResult::Stopped_MaxIter)
      {
        // If the alignment did not converge or is NaN, when processing this patch
        // the last time, then there is no reason to go to the next level.
        continue;
      }

      Vector2 px_ref_offset = Vector2::Zero();

      if(affine_warp)
      {
        //! @todo: Scale for ref and cur is the same.
        real_t inv_scale = 1.0 / scale;
        Matrix2 A_cur_ref =
            getAffineWarpMatrix(cam_ref, cam_cur, px_ref_vec.col(i), f_ref_vec.col(i),
                                inv_depth_ref_vec(i), T_cur_ref_prior, inv_scale);

       if(!warpAffine8uC1(
             A_cur_ref, img_ref_data, img_ref_width, img_ref_height, img_ref_stride,
             px_ref_vec.col(i), inv_scale, inv_scale, halfpatch_size + 1, patch_data))
        {
          result[i] = KltResult::Stopped_RefPatchNotWithinMargin;
          continue;
        }
      }
      else
      {
        // Extract patch with border.
        // We round the keypoint to the closest int, so we can avoid using bilinear
        // interpolation when extracting the reference patch. At the end, we add the
        // offset to the closest int to the result again.
        Keypoint px_ref = px_ref_vec.col(i) * scale;
        Vector2i px_ref_rounded = (px_ref + Keypoint(0.5, 0.5)).cast<int>();
        px_ref_offset = px_ref - px_ref_rounded.cast<real_t>();

        // Check visibility of reference patch:
        if(!isVisibleWithMargin(
             img_ref.width(), img_ref.height(), px_ref_rounded, halfpatch_size + 1))
        {
          result[i] = KltResult::Stopped_RefPatchNotWithinMargin;
          continue;
        }

        extractPatch8uC1(
              img_ref, px_ref_rounded, halfpatch_size + 1, patch.data(0, 0));
      }

      // Compute patch derivatives.
      computePatchDerivative8uC1(
            patch_size_by8 * 2,
            patch_data,
            reinterpret_cast<int16_t*>(patch_dxdy.data(0, 0)),
            reinterpret_cast<int16_t*>(patch_dxdy.data(0, 1)));

      // Create patch without border.
      removeBorderFromPatch8uC1(
            patch_size, patch_data,
            reinterpret_cast<uint8_t*>(patch.data(0, 1)));

      // Align patch.
      Keypoint px_cur = px_cur_vec.col(i) * scale;
      result[i] = alignPatch(
                    params,
                    img_data,
                    reinterpret_cast<const uint8_t*>(patch.data(0, 1)),
                    reinterpret_cast<int16_t*>(patch_dxdy.data(0, 0)),
                    reinterpret_cast<int16_t*>(patch_dxdy.data(0, 1)),
                    img_width, img_height, img_stride,
                    patch_size_by8,
                    px_cur);

      // Account for offset and scale.
      px_cur_vec.col(i) = (px_cur + px_ref_offset) / scale;
    }
  }

  return result;
}

//------------------------------------------------------------------------------
KltResult alignPatchImpl(
    const KltParameters& params,
    const uint8_t* __restrict__ img_data,
    const uint8_t* __restrict__ patch,
    const int16_t* __restrict__ patch_dx_x2,
    const int16_t* __restrict__ patch_dy_x2,
    const int img_width,
    const int img_height,
    const int img_stride,
    const int patch_size_by_8,
    Eigen::Ref<Keypoint> cur_px_estimate)
{
  // Pre-allocation of data:
  const int patch_size = patch_size_by_8 * 8;
  const int halfpatch_size = patch_size >> 1;
  const int margin = std::max(halfpatch_size, params.border_margin);
  const int patch_area_by_8 = patch_size_by_8 * patch_size;
  const Matrix3 H_inv = getPatchHessianInverse(patch_area_by_8, patch_dx_x2, patch_dy_x2);
  const int img_max_x = img_width  - margin;
  const int img_max_y = img_height - margin;
  KltResult result = KltResult::Stopped_MaxIter;
  real_t mean_diff = 0.0;
  Vector3 update = Vector3::Zero();

  // Compute pixel location in new image:
  real_t u = cur_px_estimate(0);
  real_t v = cur_px_estimate(1);

  // Optimize
  for(int iter = 0; iter < params.termcrit_n_iter; ++iter)
  {
    int u_r = static_cast<int>(u); // floor.
    int v_r = static_cast<int>(v); // floor.
    if(u_r < margin || v_r < margin || u_r >= img_max_x || v_r >= img_max_y)
    {
      result = KltResult::Stopped_NotWithinMargin;
      break;
    }

    if(std::isnan(u) || std::isnan(v))
    {
      result = KltResult::Stopped_NaN;
      break;
    }

    // Pre-compute bi-linear interpolation weights.
    real_t subpix_x = u - u_r;
    real_t subpix_y = v - v_r;
    real_t wTL = (1.0 - subpix_x) * (1.0 - subpix_y);
    real_t wTR = subpix_x * (1.0 - subpix_y);
    real_t wBL = (1.0 - subpix_x) * subpix_y;
    real_t wBR = subpix_x * subpix_y;

    // loop through search_patch, interpolate
    real_t chi2 = 0;
    Vector3 Jres = Vector3::Zero();
    const int16_t* dx_ptr = patch_dx_x2;
    const int16_t* dy_ptr = patch_dy_x2;
    const uint8_t* patch_ptr = patch;
    for(int y = 0; y < patch_size; ++y)
    {
      const uint8_t* img_ptr   = img_data + (v_r - halfpatch_size + y) * img_stride
                                          + (u_r - halfpatch_size);
      for(int x = 0; x < patch_size; ++x, ++img_ptr, ++dx_ptr, ++dy_ptr, ++patch_ptr)
      {
        real_t img_px = wTL * img_ptr[0]          + wTR * img_ptr[1]
                         + wBL * img_ptr[img_stride] + wBR * img_ptr[img_stride+1];
        real_t res = img_px - (*patch_ptr) + mean_diff;
        Jres[0] -= res * (*dx_ptr);
        Jres[1] -= res * (*dy_ptr);
        Jres[2] -= res;
        chi2 += res * res;
      }
    }

    // times 2 because dx/dy misses a factor /2
    Jres[0] /= 2.0;
    Jres[1] /= 2.0;
    update = H_inv  * Jres;
    u += update[0];
    v += update[1];
    mean_diff += update[2];

    VLOG(500)
        << "Chi2 error = " << chi2
        << ", update = " << update.transpose();

    if(update[0]*update[0]+update[1]*update[1] < params.termcrit_min_update_squared)
    {
      result = KltResult::Converged;
      break;
    }
  }

  // Result
  cur_px_estimate << u, v;
  return result;
}

} // namespace ze
