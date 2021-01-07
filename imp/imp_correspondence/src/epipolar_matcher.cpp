#include <imp/correspondence/epipolar_matcher.hpp>

#ifdef ZE_USE_OPENCV
#include <imp/bridge/opencv/cv_bridge.hpp>
#endif
#include <imp/correspondence/epipolar_sampling.hpp>
#include <imp/correspondence/patch_utils.hpp>
#include <imp/correspondence/patch_score.hpp>
#include <imp/correspondence/klt.hpp>

#include <ze/cameras/camera.hpp>
#include <ze/cameras/camera_utils.hpp>
#include <ze/geometry/triangulation.hpp>
#include <kindr/minimal/angle-axis.h>

namespace ze {

std::string stringFromEpipolarMatchResult(const EpipolarMatchResult res)
{
  std::string desc;
  switch (res)
  {
    case EpipolarMatchResult::Success:
      desc = "Success";
      break;
    case EpipolarMatchResult::FailScore:
      desc = "Fail Score";
      break;
    case EpipolarMatchResult::FailWarp:
      desc = "Fail Warp";
      break;
    case EpipolarMatchResult::FailAlignment:
      desc = "Fail Alignment";
      break;
    case EpipolarMatchResult::FailVisibilityReference:
      desc = "Fail Visibility Reference";
      break;
    case EpipolarMatchResult::FailVisibilityCurrent:
      desc = "Fail Visibility Current";
      break;
    default:
      LOG(FATAL) << "Match result unknown";
      break;
  }
  return desc;
}

//------------------------------------------------------------------------------
EpipolarMatcher::EpipolarMatcher(EpipolarMatcherOptions options)
  : options_(options)
  , patch_data_((c_max_patch_size_by_8_ * 8 + 2) * (c_max_patch_size_by_8_ * 8 + 2), 6)
  , patch_with_border_(reinterpret_cast<uint8_t*>(patch_data_.data(0, 0)))
  , patch_(reinterpret_cast<uint8_t*>(patch_data_.data(0, 1)))
  , patch_dx_(reinterpret_cast<int16_t*>(patch_data_.data(0, 2)))
  , patch_dy_(reinterpret_cast<int16_t*>(patch_data_.data(0, 4)))
{}

//------------------------------------------------------------------------------
std::pair<Keypoint, EpipolarMatchResult>
EpipolarMatcher::findEpipolarMatchDirect(
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
    const real_t inv_depth_ref_max)
{
  CHECK_GE(inv_depth_ref_estimate, 0.0);
  CHECK_GE(inv_depth_ref_min, 0.0);
  CHECK_GE(inv_depth_ref_max, 0.0);

  // Compute start and end of epipolar line in old_kf for match search, on image plane
  const Bearing f_ref_rotated = T_cur_ref.getRotation().rotate(f_ref);
  Bearing f_C = f_ref_rotated + T_cur_ref.getPosition() * inv_depth_ref_estimate;

  // Check visibility.
  Keypoint px = cam_cur.project(f_C);
  if (!isVisibleWithMargin(cam_cur.size(), px, c_halfpatch_size_ + 2.0))
  {
    return std::make_pair(Keypoint(), EpipolarMatchResult::FailVisibilityCurrent);
  }

  Bearing f_A = f_ref_rotated + T_cur_ref.getPosition() * inv_depth_ref_min;
  Bearing f_B = f_ref_rotated + T_cur_ref.getPosition() * inv_depth_ref_max;

  // Compute affine warp matrix between reference and current image.
  real_t scale_ref = real_t{1} / pyr_ref.scaleFactor(level_ref);
  A_cur_ref_ = getAffineWarpMatrix(
        cam_ref, cam_cur, px_ref, f_ref, inv_depth_ref_estimate,
        T_cur_ref, scale_ref);

  // Get optimal level in current image, given the warp matrix.
  level_cur_ =
      getBestSearchLevel(A_cur_ref_, pyr_cur.scaleFactor(), pyr_cur.numLevels() - 1);
  const Image8uC1& img_cur = pyr_cur.at(level_cur_);
  real_t scale_cur = real_t{1} / pyr_cur.scaleFactor(level_cur_);

  // Warp reference patch. We create a slightly larger patch so we can compute
  // the derivative later for subpixel refinement.
  const Image8uC1& img_ref = pyr_ref.at(level_ref);
  if (!warpAffine8uC1(
        A_cur_ref_, reinterpret_cast<const uint8_t*>(img_ref.data()),
        img_ref.width(), img_ref.height(), img_ref.stride(), px_ref, scale_ref, scale_cur,
        c_halfpatch_size_ + 1, patch_with_border_))
  {
    return std::make_pair(Keypoint(), EpipolarMatchResult::FailWarp);
  }

  // Remove patch from border
  removeBorderFromPatch8uC1(c_patch_size_, patch_with_border_, patch_);

  // Perform epipolar search if epipolar line segment is longer than 2 pixels.
  Keypoint px_A = cam_cur.project(f_A) / scale_cur;
  Keypoint px_B = cam_cur.project(f_B) / scale_cur;
  Keypoint px_cur_lev = cam_cur.project(f_C) / scale_cur;
  if ((px_A - px_B).norm() > options_.max_epi_length_optim
      || options_.subpix_refinement == false)
  {
    f_A.normalize();
    f_B.normalize();
    f_C.normalize();
    auto res = findMaxScoreOnEpipolarLine(f_A, f_B, f_C, img_cur, cam_cur, scale_cur);
    if(!res.second)
    {
      return std::make_pair(Keypoint(), EpipolarMatchResult::FailScore);
    }
    px_cur_lev = res.first.cast<real_t>();
  }

  // Subpixel refinement
  if(options_.subpix_refinement)
  {
    if (!subpixelPatchAlignment(img_cur, px_cur_lev, c_patch_size_by_8_))
    {
      return std::make_pair(Keypoint(), EpipolarMatchResult::FailAlignment);
    }
  }

  // Success
  Keypoint px_cur = px_cur_lev * scale_cur;
  if (!isVisibleWithMargin(cam_cur.size(), px_cur, options_.margin_cur_level0))
  {
    return std::make_pair(px_cur, EpipolarMatchResult::FailVisibilityCurrent);
  }
  return std::make_pair(px_cur, EpipolarMatchResult::Success);
}

//------------------------------------------------------------------------------
EpipolarMatchResult EpipolarMatcher::refineMatchDirect(
    const Transformation& T_cur_ref,
    const ImagePyramid8uC1& pyr_ref,
    const ImagePyramid8uC1& pyr_cur,
    const Camera& cam_ref,
    const Camera& cam_cur,
    const Eigen::Ref<const Keypoint> px_ref,
    const Eigen::Ref<const Bearing> f_ref,
    const uint32_t level_ref_target,
    const real_t inv_depth_ref_estimate,
    Keypoint& px_cur,
    const uint32_t num_levels)
{
  DEBUG_CHECK_GE(inv_depth_ref_estimate, 0.0);
  DEBUG_CHECK_DOUBLE_EQ(f_ref.norm(), 1.0);
  DEBUG_CHECK_DOUBLE_EQ((cam_ref.backProject(px_ref) - f_ref).norm(), 0.0);

  int max_level = std::min(static_cast<int>(pyr_ref.numLevels() - 1),
                           static_cast<int>(level_ref_target + num_levels - 1));
  for (int level_ref = max_level; level_ref >= static_cast<int>(level_ref_target);
       --level_ref)
  {
    DEBUG_CHECK_LT(static_cast<uint32_t>(level_ref - level_ref_target),
                   patch_sizes_by8_.size());

    int patch_size_by8 = patch_sizes_by8_[level_ref - level_ref_target];

    DEBUG_CHECK_LT(level_ref, static_cast<int>(pyr_ref.numLevels()));
    real_t scale_ref = real_t{1} / pyr_ref.scaleFactor(level_ref);

    const real_t margin = (patch_size_by8 * 4 + 2) * scale_ref;
    if (!isVisibleWithMargin(cam_ref.size(), px_ref, margin))
    {
      return EpipolarMatchResult::FailVisibilityReference;
    }

    // Compute affine warp matrix between reference and current image.
    A_cur_ref_ = getAffineWarpMatrix(
          cam_ref, cam_cur, px_ref, f_ref, inv_depth_ref_estimate,
          T_cur_ref, scale_ref);

    level_cur_ = getBestSearchLevel(
                   A_cur_ref_, pyr_cur.scaleFactor(), pyr_cur.numLevels() - 1);
    const Image8uC1& img_cur = pyr_cur.at(level_cur_);

    // Warp reference patch. We create a slightly larger patch so we can compute
    // the derivative later for subpixel refinement.
    const real_t scale_cur = real_t{1} / pyr_cur.scaleFactor(level_cur_);
    const Image8uC1& img_ref = pyr_ref.at(level_ref);
    if (!warpAffine8uC1(
          A_cur_ref_, reinterpret_cast<const uint8_t*>(img_ref.data()),
          img_ref.width(), img_ref.height(), img_ref.stride(), px_ref, scale_ref, scale_cur,
          patch_size_by8 * 4 + 1, patch_with_border_))
    {
      return EpipolarMatchResult::FailWarp;
    }

    // Remove patch from border
    removeBorderFromPatch8uC1(patch_size_by8 * 8, patch_with_border_, patch_);

    //! @todo: Implement 1D alignment for edgelets!

    // Run KLT.
    Keypoint px_cur_lev = px_cur / scale_cur;
    if (!subpixelPatchAlignment(img_cur, px_cur_lev, patch_size_by8))
    {
      return EpipolarMatchResult::FailAlignment;
    }
    px_cur = px_cur_lev * scale_cur;

    if (!isVisibleWithMargin(cam_cur.size(), px_cur, options_.margin_cur_level0))
    {
      return EpipolarMatchResult::FailVisibilityCurrent;
    }
  }

  return EpipolarMatchResult::Success;
}

//------------------------------------------------------------------------------
bool EpipolarMatcher::subpixelPatchAlignment(
    const Image8uC1& img_cur,
    Eigen::Ref<Keypoint> px_cur_lev,
    int patch_size_by_8)
{
  DEBUG_CHECK_LE(patch_size_by_8, c_max_patch_size_by_8_);

  computePatchDerivative8uC1(
        patch_size_by_8 * 2, patch_with_border_, patch_dx_, patch_dy_);
  if (alignPatch(
        options_.subpix_parameters,
        reinterpret_cast<const uint8_t*>(img_cur.data()),
        patch_, patch_dx_, patch_dy_,
        img_cur.width(), img_cur.height(), img_cur.stride(),
        patch_size_by_8,
        px_cur_lev) != KltResult::Converged)
  {
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
std::pair<Vector2i, bool> EpipolarMatcher::findMaxScoreOnEpipolarLine(
    const Bearing& f_A, const Bearing& f_B, const Bearing& f_C,
    const Image8uC1& img_cur, const Camera& cam_cur,
    real_t scale_cur) // (1 << level)
{

  PixelVector px_on_epipolar_line_lev = sampleGreatCircle(
        cam_cur, scale_cur, c_patch_size_, f_A, f_B, f_C, options_.max_epi_search_steps);

  // For each pixel on epipolar line, compute score.
  PatchScore patch_score(patch_);
  int zmssd_best = PatchScore::threshold();
  Vector2i px_best = Vector2i::Zero();
  for (const Vector2i px : px_on_epipolar_line_lev)
  {
    const uint8_t* cur_patch =
        reinterpret_cast<const uint8_t*>(img_cur.data(px(0) - c_halfpatch_size_,
                                                      px(1) - c_halfpatch_size_));
    int zmssd = patch_score.computeScore(cur_patch, img_cur.stride());
    if (zmssd < zmssd_best)
    {
      zmssd_best = zmssd;
      px_best = px;
    }
  }

  if (zmssd_best >= PatchScore::threshold())
  {
    return std::make_pair(px_best, false);
  }
  return std::make_pair(px_best, true);
}

//------------------------------------------------------------------------------
void EpipolarMatcher::displayMatch(
    const ImagePyramid8uC1& pyr_cur,
    const Keypoint& px_cur,
    const uint32_t level_cur,
    const ImagePyramid8uC1& pyr_ref,
    const Keypoint& px_ref,
    const uint32_t level_ref,
    const Eigen::Ref<const Bearing> f_ref,
    const Transformation& T_cur_ref,
    const real_t inv_depth_ref_estimate,
    const Camera& cam_cur)
{
#ifdef ZE_USE_OPENCV
  cv::Mat img_cur = ImageCv8uC1(pyr_cur.at(level_cur)).cvMat();
  cv::Mat img_cur_rgb = cv::Mat(img_cur.size(), CV_8UC3);
  cv::cvtColor(img_cur, img_cur_rgb, cv::COLOR_GRAY2RGB);
  Keypoint px_cur_lev = px_cur / (1 << level_cur);
  cv::circle(img_cur_rgb, cv::Point2f(px_cur_lev(0), px_cur_lev(1)), 10, cv::Scalar(0, 255, 0));

  cv::Mat img_ref = ImageCv8uC1(pyr_ref.at(level_ref)).cvMat();
  cv::Mat img_ref_rgb = cv::Mat(img_ref.size(), CV_8UC3);
  cv::cvtColor(img_ref, img_ref_rgb, cv::COLOR_GRAY2RGB);
  Keypoint px_ref_lev = px_ref / (1 << level_ref);
  cv::circle(img_ref_rgb, cv::Point2f(px_ref_lev(0), px_ref_lev(1)), 10, cv::Scalar(0, 255, 0));

  // Compute start and end of epipolar line in old_kf for match search, on image plane
  const Bearing f_ref_rotated = T_cur_ref.getRotation().rotate(f_ref);
  Bearing f_C = f_ref_rotated + T_cur_ref.getPosition() * inv_depth_ref_estimate;

  // Check visibility.
  Keypoint px = cam_cur.project(f_C) / (1 << level_cur);
  cv::circle(img_cur_rgb, cv::Point2f(px(0), px(1)), 12, cv::Scalar(255, 0, 255));

  // Draw affine warp.
  Matrix2 A_ref_cur = A_cur_ref_.inverse() / (1 << level_cur);
  cv::line(img_cur_rgb,
           cv::Point2f(px_cur_lev(0), px_cur_lev(1)),
           cv::Point2f(px_cur_lev(0)+20, px_cur_lev(1)), cv::Scalar(0,0,255), 2);
  cv::line(img_cur_rgb,
           cv::Point2f(px_cur_lev(0), px_cur_lev(1)),
           cv::Point2f(px_cur_lev(0), px_cur_lev(1)+20), cv::Scalar(0,255,0), 2);
  cv::line(img_ref_rgb,
           cv::Point2f(px_ref_lev(0),    px_ref_lev(1)),
           cv::Point2f(px_ref_lev(0) + A_ref_cur(0,0)*20, px_ref_lev(1) + A_ref_cur(1,0)*20), cv::Scalar(0,0,255), 2);
  cv::line(img_ref_rgb,
           cv::Point2f(px_ref_lev(0), px_ref_lev(1)),
           cv::Point2f(px_ref_lev(0) + A_ref_cur(0,1)*20, px_ref_lev(1) + A_ref_cur(1,1)*20), cv::Scalar(0,255,0), 2);

  uint32_t patch_size_by8 = 1;
  cv::Mat patch(patch_size_by8 * 8, patch_size_by8 * 8, CV_8UC1, reinterpret_cast<uint8_t*>(patch_data_.data(0, 1)));
  cv::Mat patch_large;
  cv::resize(patch, patch_large, cv::Size(), 4.0, 4.0, cv::INTER_LINEAR);
  cv::imshow("img_cur", img_cur_rgb);
  cv::imshow("img_ref", img_ref_rgb);
  cv::imshow("patch", patch_large);
  cv::waitKey(0);
#endif
}

} // namespace ze
