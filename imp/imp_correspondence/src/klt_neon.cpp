#ifndef __ARM_NEON__
//# error "This file requires NEON support. Check your compiler flags."
#else
# include <arm_neon.h>
#endif

#include <imp/correspondence/patch_utils.hpp>
#include <imp/correspondence/klt.hpp>

namespace ze {

KltResult alignPatchNeon(
    const KltParameters& params,
    const uint8_t* __restrict__ img_data,
    const uint8_t* __restrict__ ref_patch,
    const int16_t* __restrict__ ref_patch_dx_x2,
    const int16_t* __restrict__ ref_patch_dy_x2,
    const int img_width,
    const int img_height,
    const int img_stride,
    const int patch_size_by8,
    Eigen::Ref<Keypoint> cur_px_estimate)
{
  // Pre-allocation of data:
  const int patch_size = patch_size_by8 * 8;
  const int halfpatch_size = patch_size >> 1;
  const int margin = std::max(halfpatch_size, params.border_margin);
  const int patch_area_by8 = patch_size_by8 * patch_size;
  const Matrix3 H_inv = getPatchHessianInverse(patch_area_by8, ref_patch_dx_x2, ref_patch_dy_x2);
  const int img_max_x = img_width  - margin;
  const int img_max_y = img_height - margin;
  KltResult result = KltResult::Stopped_MaxIter;

  // State:
  real_t u = cur_px_estimate(0);
  real_t v = cur_px_estimate(1);
  real_t mean_diff {0.0};

  // Optimize:
  for (int iter = 0; iter < params.termcrit_n_iter; ++iter)
  {
    int u_r = static_cast<int>(u); // floor.
    int v_r = static_cast<int>(v); // floor.
    if (u_r < margin || v_r < margin || u_r >= img_max_x || v_r >= img_max_y)
    {
      result = KltResult::Stopped_NotWithinMargin;
      break;
    }

    if (std::isnan(u) || std::isnan(v))
    {
      result = KltResult::Stopped_NaN;
      break;
    }

    // Pre-compute bi-linear interpolation weights.
    constexpr int SHIFT_BITS = 7;
    constexpr int FACTOR = (1 << SHIFT_BITS);
    const real_t subpix_x = u - u_r;
    const real_t subpix_y = v - v_r;
    const uint16_t wTL = static_cast<uint16_t>((real_t{1.0} - subpix_x) * (real_t{1.0} - subpix_y) * FACTOR);
    const uint16_t wTR = static_cast<uint16_t>(subpix_x * (real_t{1.0} - subpix_y) * FACTOR);
    const uint16_t wBL = static_cast<uint16_t>((real_t{1.0} - subpix_x) * subpix_y * FACTOR);
    //const uint16_t wBR = static_cast<uint16_t>(FACTOR - wTL - wTR - wBL);
    const uint16_t wBR = static_cast<uint16_t>(subpix_x * subpix_y * FACTOR);

    // initialize result to zero
    int32x4_t Jres0_int32x4 = vdupq_n_s32(int32_t{0});
    int32x4_t Jres1_int32x4 = vdupq_n_s32(int32_t{0});
    int16x8_t Jres2_int16x8 = vdupq_n_s16(int16_t{0});
    int16x8_t mean_diff_int16x8 = vdupq_n_s16(static_cast<int16_t>(mean_diff + real_t{0.5}));

    // loop through search_patch, interpolate
    const uint8_t* __restrict__ it_ref = ref_patch;
    for (int y = 0; y < patch_size; ++y)
    {
      const uint8_t* __restrict__ it_cur = img_data + (v_r - halfpatch_size + y) * img_stride
                                                    + (u_r - halfpatch_size);
      for (int x = 0; x < patch_size_by8 * 8; x += 8, it_cur += 8, it_ref += 8)
      {
        // ---------------------------------------------------------------------
        // Computing:
        // img_px = wTL * img_ptr[0]          + wTR * img_ptr[1]
        //        + wBL * img_ptr[img_stride] + wBR * img_ptr[img_stride+1];

        // load and convert from uint8 to uint16
        uint16x8_t v00 = vmovl_u8(vld1_u8(it_cur));
        uint16x8_t v01 = vmovl_u8(vld1_u8(it_cur + 1));
        uint16x8_t v10 = vmovl_u8(vld1_u8(it_cur + img_stride));
        uint16x8_t v11 = vmovl_u8(vld1_u8(it_cur + img_stride + 1));

        // vector multiply by scalar
        v00 = vmulq_n_u16( v00, wTL );
        v01 = vmulq_n_u16( v01, wTR );
        v10 = vmulq_n_u16( v10, wBL );
        v11 = vmulq_n_u16( v11, wBR );

        // add all results together
        v00 = vaddq_u16( v00, vaddq_u16( v01, vaddq_u16( v10, v11 ) ) );

        // descale: shift right by constant
        v00 = vrshrq_n_u16(v00, SHIFT_BITS);

        // ---------------------------------------------------------------------
        // compute difference between reference and interpolated patch,
        // use reinterpet-cast to make signed [-255,255]:
        // res = img_px - (*patch_ptr) + mean_diff;

        int16x8_t res_int16x8 = vsubq_s16(
                                  vreinterpretq_s16_u16(v00),
                                  vreinterpretq_s16_u16(vmovl_u8(vld1_u8(it_ref))));

        // correct res with mean difference
        res_int16x8 = vaddq_s16(res_int16x8, mean_diff_int16x8);    // int16x8_t

        // ---------------------------------------------------------------------
        // Compute jacobian:
        // Jres[0] -= res * (*dx_ptr);
        // Jres[1] -= res * (*dy_ptr);
        // Jres[2] -= res;

        // multiply accumulate long: vmla -> Vr[i] := Va[i] + Vb[i] * Vc[i]
        // int32x4_t  vmlal_s16(int32x4_t a, int16x4_t b, int16x4_t c);    // VMLAL.S16 q0,d0,d0
        int16x8_t dx = vld1q_s16(ref_patch_dx_x2 + y * patch_size + x);
        Jres0_int32x4 = vmlal_s16(Jres0_int32x4, vget_low_s16( dx), vget_low_s16( res_int16x8));
        Jres0_int32x4 = vmlal_s16(Jres0_int32x4, vget_high_s16(dx), vget_high_s16(res_int16x8));

        int16x8_t dy = vld1q_s16(ref_patch_dy_x2 + y * patch_size + x);
        Jres1_int32x4 = vmlal_s16(Jres1_int32x4, vget_low_s16( dy), vget_low_s16( res_int16x8));
        Jres1_int32x4 = vmlal_s16(Jres1_int32x4, vget_high_s16(dy), vget_high_s16(res_int16x8));

        // compute sum of the residual
        Jres2_int16x8 = vaddq_s16(Jres2_int16x8, res_int16x8);
      }
    }

    // Finally, sum the results: (* 0.5 because we did not compute the derivative correctly)
    Vector3 Jres;
    int32x2_t tmp1 = vpadd_s32(vget_low_s32(Jres0_int32x4), vget_high_s32(Jres0_int32x4));
    Jres[0] = - real_t{0.5} * (vget_lane_s32(tmp1, 0) + vget_lane_s32(tmp1, 1));

    int32x2_t tmp2 = vpadd_s32(vget_low_s32(Jres1_int32x4), vget_high_s32(Jres1_int32x4));
    Jres[1] = - real_t{0.5} * (vget_lane_s32(tmp2, 0) + vget_lane_s32(tmp2, 1));

    int32x4_t tmp3 = vpaddlq_s16(Jres2_int16x8);
    int32x2_t tmp4 = vpadd_s32(vget_low_s32(tmp3), vget_high_s32(tmp3));
    Jres[2] = - vget_lane_s32(tmp4, 0) - vget_lane_s32(tmp4, 1);

    // Compute update:
    Vector3 update = H_inv * Jres;
    u += update[0];
    v += update[1];
    mean_diff += update[2];

    VLOG(500) << "update = " << update.transpose();

    if (update[0]*update[0]+update[1]*update[1] < params.termcrit_min_update_squared)
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
