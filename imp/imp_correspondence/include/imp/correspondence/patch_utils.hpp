#pragma once

#include <ze/cameras/camera.hpp>
#include <ze/common/macros.hpp>
#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>
#include <imp/core/pixel.hpp>
#include <imp/core/image.hpp>

namespace ze {

// -----------------------------------------------------------------------------
inline Matrix2 getAffineWarpMatrix(
    const Camera& cam_ref,
    const Camera& cam_cur,
    const Eigen::Ref<const Keypoint>& px_ref,
    const Eigen::Ref<const Bearing>& f_ref,
    const real_t inv_depth_ref,
    const Transformation& T_cur_ref,
    const real_t ref_scale) // = (1<<level_ref)
{
  constexpr int halfpatch_size = 5;
  Matrix2 A_cur_ref;

  if (inv_depth_ref < real_t{0.0001})
  {
    // Using inverse depth.
    Bearing f_du_ref = cam_ref.backProject(px_ref + Vector2(halfpatch_size * ref_scale, 0));
    Bearing f_dv_ref = cam_ref.backProject(px_ref + Vector2(0, halfpatch_size * ref_scale));

    // Using inverse depth coordinates we don't need to explicitly compute the
    // 3D point. Should numerically be more stable for points far away.
    Vector3 t = T_cur_ref.getPosition() * inv_depth_ref;
    Keypoint px_cur    = cam_cur.project(T_cur_ref.getRotation().rotate(f_ref) + t);
    Keypoint px_du_cur = cam_cur.project(T_cur_ref.getRotation().rotate(f_du_ref) + t);
    Keypoint px_dv_cur = cam_cur.project(T_cur_ref.getRotation().rotate(f_dv_ref) + t);

    A_cur_ref.col(0) = (px_du_cur - px_cur) / halfpatch_size;
    A_cur_ref.col(1) = (px_dv_cur - px_cur) / halfpatch_size;
  }
  else
  {
    // Using direct depth
    const Position xyz_ref = f_ref / inv_depth_ref;
    Position xyz_dx_ref = cam_ref.backProject(px_ref + Vector2(halfpatch_size, 0) * ref_scale);
    Position xyz_dy_ref = cam_ref.backProject(px_ref + Vector2(0, halfpatch_size) * ref_scale);

    // This ensures that the z-value of all 3 points is the same.
    // Hence, this assumes that the patch is fronto-parallel to the reference camera.
    xyz_dx_ref *= xyz_ref(2) / xyz_dx_ref(2);
    xyz_dy_ref *= xyz_ref(2) / xyz_dy_ref(2);

    Keypoint px_cur    = cam_cur.project(T_cur_ref * xyz_ref);
    Keypoint px_dx_cur = cam_cur.project(T_cur_ref * xyz_dx_ref);
    Keypoint px_dy_cur = cam_cur.project(T_cur_ref * xyz_dy_ref);
    A_cur_ref.col(0) = (px_dx_cur - px_cur) / halfpatch_size;
    A_cur_ref.col(1) = (px_dy_cur - px_cur) / halfpatch_size;
  }
  return A_cur_ref;
}

// -----------------------------------------------------------------------------
inline int getBestSearchLevel(
    const Matrix2& A_cur_ref,
    const real_t pyr_scale, // typically 0.5
    const int max_level)
{
  // Compute patch level in other image
  int level_cur = 0;
  real_t D = A_cur_ref.determinant();
  real_t pyr_scale_sq = pyr_scale * pyr_scale;
  while (D > real_t{3.0} && level_cur < max_level)
  {
    level_cur += 1;
    D *= pyr_scale_sq;
  }
  return level_cur;
}

// -----------------------------------------------------------------------------
inline bool warpAffine8uC1(
    const Matrix2& A_cur_ref,
    const uint8_t* __restrict__ img_data,
    const int img_width,
    const int img_height,
    const int img_stride,
    const Keypoint& px_ref_level0,
    const real_t scale_ref, // (1 << level_ref)
    const real_t scale_cur, // (1 << search_level)
    const int halfpatch_size,
    uint8_t* __restrict__ patch)
{
  CHECK_NOTNULL(patch);
  Matrix2 A_ref_cur = A_cur_ref.inverse() * scale_cur;
  if(std::isnan(A_ref_cur(0,0)))
  {
    LOG(WARNING) << "Affine warp is NaN, probably camera has no translation";
    return false;
  }

  // Perform the warp on a larger patch.
  uint8_t* patch_ptr = patch;
  const Vector2 px_ref_pyr = px_ref_level0 / scale_ref;

  const int w = img_width - 1;
  const int h = img_height - 1;
  for (int y = -halfpatch_size; y < halfpatch_size; ++y)
  {
    for (int x = -halfpatch_size; x < halfpatch_size; ++x, ++patch_ptr)
    {
      const Vector2 px_patch(x, y);
      const Vector2 px = A_ref_cur * px_patch + px_ref_pyr;
      const int xi = static_cast<int>(px[0]);
      const int yi = static_cast<int>(px[1]);

      //! @todo: For increased efficiency: Do this check before with 4 corners.
      if (UNLIKELY(xi < 0 || yi < 0 || xi >= w || yi >= h)) // unsigned int is always positive.
      {
        return false;
      }
      else
      {
        const real_t subpix_x = px[0] - xi;
        const real_t subpix_y = px[1] - yi;
        const real_t wTL = (real_t{1.0} - subpix_x) * (real_t{1.0} - subpix_y);
        const real_t wTR = subpix_x * (real_t{1.0} - subpix_y);
        const real_t wBL = (real_t{1.0} - subpix_x) * subpix_y;
        const real_t wBR = real_t{1.0} - wTL - wBL - wTR;
        const uint8_t* const ptr = img_data + yi * img_stride + xi;
        *patch_ptr = wTL * ptr[0]          + wTR * ptr[1]
                   + wBL * ptr[img_stride] + wBR * ptr[img_stride+1];
      }
    }
  }
  return true;
}

// -----------------------------------------------------------------------------
inline void extractPatch8uC1(
    const Image8uC1& img,
    const Vector2i& px,
    const int halfpatch_size,
    Pixel8uC1* patch)
{
  CHECK_NOTNULL(patch);
  CHECK(px(0) >= halfpatch_size
        && px(1) >= halfpatch_size
        && px(0) < static_cast<int>(img.width()) - halfpatch_size
        && px(1) < static_cast<int>(img.height()) - halfpatch_size);

  const int patch_size = 2 * halfpatch_size;
  const int img_stride = img.stride();
  const Pixel8uC1* img_data = img.data();
  Pixel8uC1* patch_ptr = patch;
  for (int y = 0; y < patch_size; ++y)
  {
    const Pixel8uC1* img_ptr = img_data + (px[1] - halfpatch_size + y) * img_stride
                                        + (px[0] - halfpatch_size);
    for (int x = 0; x < patch_size; ++x, ++patch_ptr)
    {
      *patch_ptr = img_ptr[x];
    }
  }
}

// -----------------------------------------------------------------------------
// IMPORTANT: Check before calling that patch is inside image!
inline void extractPatchInterpolated8uC1(
    const uint8_t* __restrict__ img_data,
    const int img_stride,
    const Eigen::Ref<const Keypoint>& px,
    const int halfpatch_size,
    uint8_t* __restrict__ patch)
{
  //! @todo: This could be easily implemented using SIMD instructions.

  const real_t u = px(0);
  const real_t v = px(1);
  int u_r = static_cast<int>(u); // floor
  int v_r = static_cast<int>(v); // floor

  // compute interpolation weights
  const real_t subpix_x = u - u_r;
  const real_t subpix_y = v - v_r;
#if 1
  const real_t wTL = (real_t{1.0} - subpix_x) * (real_t{1.0} - subpix_y);
  const real_t wTR = subpix_x * (real_t{1.0} - subpix_y);
  const real_t wBL = (real_t{1.0} - subpix_x) * subpix_y;
  const real_t wBR = subpix_x * subpix_y;
  const int patch_size = 2 * halfpatch_size;
  for (int y = 0; y < patch_size; ++y)
  {
    const uint8_t* __restrict__ img_ptr =
        img_data + (v_r - halfpatch_size + y) * img_stride + (u_r - halfpatch_size);
    uint8_t* __restrict__ patch_ptr = patch + y * patch_size;
    for (int x = 0; x < patch_size; ++x)
    {
      // Bilinear interpolation:
      patch_ptr[x] = wTL * img_ptr[x]          + wTR * img_ptr[x+1]
                   + wBL * img_ptr[x+img_stride] + wBR * img_ptr[x+img_stride+1];
    }
  }
#else // use fixed-point arithmetics to speed-up.
  constexpr int SHIFT_BITS = 10;
  constexpr int FACTOR = (1 << SHIFT_BITS);
  const int wTL = FACTOR * ((real_t{1.0} - subpix_x) * (real_t{1.0} - subpix_y));
  const int wTR = FACTOR * (subpix_x * (real_t{1.0} - subpix_y));
  const int wBL = FACTOR * ((real_t{1.0} - subpix_x) * subpix_y);
  const int wBR = FACTOR - wTL - wBL - wTR;
  const int patch_size = 2 * halfpatch_size;
  v_r -= halfpatch_size;
  u_r -= halfpatch_size;
  for (int y = 0; y < patch_size; ++y)
  {
    const uint8_t* const img_ptr_up = img_data + (v_r + y) * img_stride + u_r;
    const uint8_t* const img_ptr_lo = img_ptr_up + img_stride;
    uint8_t* patch_ptr = patch + y * patch_size;
    for (int x = 0; x < patch_size; ++x)
    {
      // Bilinear interpolation:
      const int res = (wTL * img_ptr_up[x] + wTR * img_ptr_up[x+1]
                     + wBL * img_ptr_lo[x] + wBR * img_ptr_lo[x+1]);
      patch_ptr[x] = (res >> SHIFT_BITS);
    }
  }
#endif
}

// -----------------------------------------------------------------------------
inline void removeBorderFromPatch8uC1(
    const int patch_size,
    const uint8_t* patch_with_border,
    uint8_t* patch)
{
  for(int y = 0; y < patch_size; ++y)
  {
    const uint8_t* ref_ptr = patch_with_border + (y + 1) * (patch_size + 2) + 1;
    uint8_t* patch_ptr = patch + y * patch_size;
    for(int x = 0; x < patch_size; ++x)
    {
      patch_ptr[x] = ref_ptr[x];
    }
  }
}

// -----------------------------------------------------------------------------
inline void computePatchDerivative8uC1(
    const int patch_size_by4,
    const uint8_t* patch_with_border,
    int16_t* patch_dx_x2,
    int16_t* patch_dy_x2)
{
  //! @todo: Patch derivative computation should be done for many patches at the
  //!        same time. Then SSE, NEON should become useful.

  // patch_with_border has two pixels more width
  const int patch_stride = patch_size_by4 * 4 + 2;

  // In the derivative computation, we are missing a division by 2 on purpose.
  for(int y = 0; y < patch_size_by4 * 4; ++y)
  {
    const uint8_t* patch_ptr = patch_with_border + (y + 1) * patch_stride + 1;
    int16_t* dx = patch_dx_x2 + y * patch_size_by4 * 4;
    int16_t* dy = patch_dy_x2 + y * patch_size_by4 * 4;
    for(int x = 0; x < patch_size_by4 * 4; ++x, ++patch_ptr)
    {
      dx[x] = static_cast<int16_t>(patch_ptr[1]) - patch_ptr[-1];
      dy[x] = static_cast<int16_t>(patch_ptr[patch_stride]) - patch_ptr[-patch_stride];
    }
  }
}

// -----------------------------------------------------------------------------
//! Input derivatives dx and dy must be multiplied with a factor of 2.
inline Matrix3 getPatchHessianInverse(
    const int data_size_by_8,
    const int16_t* patch_dx_x2,
    const int16_t* patch_dy_x2)
{
  real_t sum_dx2 = 0, sum_dy2 = 0, sum_dxdy = 0;
  real_t sum_dx = 0, sum_dy = 0;
  for(int i = 0; i < data_size_by_8 * 8; ++i)
  {
    real_t dx = patch_dx_x2[i];
    real_t dy = patch_dy_x2[i];
    sum_dx   += dx;
    sum_dy   += dy;
    sum_dx2  += dx * dx;
    sum_dy2  += dy * dy;
    sum_dxdy += dx * dy;
  }
  // correct that dx and dy has not been divided by two.
  sum_dx   /= 2.0;
  sum_dy   /= 2.0;
  sum_dx2  /= 4.0;
  sum_dy2  /= 4.0;
  sum_dxdy /= 4.0;
  Matrix3 H;
  H << sum_dx2, sum_dxdy, sum_dx,
       sum_dxdy, sum_dy2, sum_dy,
       sum_dx,   sum_dy,  data_size_by_8 * 8;
  return H.inverse();
}

} // namespace ze
