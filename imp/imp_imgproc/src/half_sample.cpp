#include <imp/imgproc/half_sample.hpp>

#if __SSE2__
# include <emmintrin.h>
#endif
#if __AVX2__
#include "immintrin.h"
#endif
#if __ARM_NEON__
# include <arm_neon.h>
#endif
#include <glog/logging.h>

#include <imp/core/image_raw.hpp>

namespace ze {

/*
ImgPyr8uC1 createImgPyramid(const Image8uC1& in, uint32_t n_levels)
{
  CHECK_GT(in.width(), 0u);
  CHECK_GT(in.height(), 0u);
  CHECK_GT(n_levels, 0u);

  ImgPyr8uC1 pyr(n_levels);
  pyr[0] = in;
  for(uint32_t i = 1; i < n_levels; ++i)
  {
    pyr[i] = ImageRaw8uC1(pyr[i-1].width() << 1, pyr[i-1].height() << 1);
    halfSample(pyr[i-1], pyr[i]);
  }
  return pyr;
}
*/

void halfSample(const Image8uC1& in, Image8uC1& out, bool use_simd)
{
  CHECK_EQ(std::floor(in.width() / 2), out.width());
  CHECK_EQ(std::floor(in.height() / 2), out.height());

  const uint8_t* in_data = reinterpret_cast<const uint8_t*>(in.data());
  uint8_t* out_data = reinterpret_cast<uint8_t*>(out.data());

//! @todo: There is a bug in the AVX2 version... just using SSE2 on Intel.
/*
#ifdef __AVX2__
  if(use_simd
     && is_aligned32(in_data)
     && is_aligned32(out_data)
     && ((in.width() % 32) == 0)
     && ((out.width() % 32) == 0))
  {
    internal::halfSample_AVX2(in_data, out_data, in.width(), in.height(),
                              in.stride(), out.stride());
    return;
  }
  else if(use_simd
          && is_aligned16(in_data)
          && is_aligned16(out_data)
          && ((in.width() % 16) == 0)
          && ((out.width() % 16) == 0))
  {
   internal::halfSample_SSE2(in_data, out_data, in.width(), in.height(),
                             in.stride(), out.stride());
   return;
  }
#endif
*/
#ifdef __SSE2__
  if(use_simd
     && isAligned16(in_data)
     && isAligned16(out_data)
     && ((in.width() % 16) == 0)
     && ((out.width() % 16) == 0))
  {
    internal::halfSample_SSE2(in_data, out_data, in.width(), in.height(),
                              in.stride(), out.stride());
    return;
  }
#endif

#ifdef __ARM_NEON__
  if(use_simd
     && (in.width() % 16) == 0)
  {
    internal::halfSample_NEON(in_data, out_data, in.width(), in.height(),
                              in.stride(), out.stride());
    return;
  }
#endif

  const int stride_in  = in.stride();
  const int stride_out = out.stride();
  const uint8_t* top = in_data;
  const uint8_t* bottom = top + stride_in;
  const uint8_t* end = top + stride_in * in.height();
  const int height = out.height();
  const int width = out.width();
  uint8_t* out_ptr = reinterpret_cast<uint8_t*>(out.data());
  for (int y=0; y < height && bottom < end;
       ++y, top += stride_in * 2, bottom += stride_in * 2, out_ptr += stride_out)
  {
    for (int x=0; x < width; x++)
    {
       int x2 = x*2;
       // divide each element by two, add them and then take care of rounding.
       // this avoids casting to float.
       uint8_t avg_top = (top[x2]>>1) + (top[x2+1]>>1) + (top[x2] & top[x2+1] & 1);
       uint8_t avg_bottom = (bottom[x2]>>1) + (bottom[x2+1]>>1) + (bottom[x2] & bottom[x2+1] & 1);
       out_ptr[x] = (avg_top>>1) + (avg_bottom>>1) + (avg_top & avg_bottom & 1);
    }
  }
}


// -----------------------------------------------------------------------------
namespace internal {

#ifdef __AVX2__
void halfSample_AVX2(const uint8_t* in, uint8_t* out, int width, int height,
                     int in_stride, int out_stride)
{
  CHECK_NOTNULL(out);
 // VLOG(300) << "halfSample AVX2: Image size = (" << width << ", " << height << "), "
 //           << "in_stride = " << in_stride << ", out_stride = " << out_stride;

  const uint64_t mask_data[4] = {0x00FF00FF00FF00FFull, 0x00FF00FF00FF00FFull,
                                 0x00FF00FF00FF00FFull, 0x00FF00FF00FF00FFull};
  const uint8_t* next_row = in + width;

  // Load 256-bits of integer data from memory into dst. mem_addr must be aligned
  // on a 32-byte boundary or a general-protection exception may be generated.
  __m256i mask = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(mask_data));
  const int width_by_32 = width >> 5;
  const int width_by_2 = height >> 1;
  const int in_padding = in_stride - width;
  const int out_width = width >> 1;
  const int out_padding = out_stride - out_width;
  for (int y = 0; y < width_by_2; y++)
  {
    for (int x = 0; x < width_by_32; x++)
    {
      __m256i here = _mm256_load_si256(reinterpret_cast<const __m256i*>(in));
      __m256i next = _mm256_load_si256(reinterpret_cast<const __m256i*>(next_row));

      // 1) (top + bottom + 1) / 2
      // Average packed unsigned 8-bit integers in a and b, and store the results in dst.
      here = _mm256_avg_epu8(here, next);

      // 2) mask rows to prepare for summing left and right
      // _mm_srli_si128(a,b): Shift a right by b bytes while shifting in zeros, and store the results in dst.
      // Compute the bitwise AND of 128 bits (representing integer data) in a and b, and store the result in dst.
      next = _mm256_and_si256(_mm256_srli_si256(here, 1), mask); // shift right by 8 and apply mask
      here = _mm256_and_si256(here, mask);                       // apply mask

      // 3) Average left and right: (left + right + 1) / 2
      // Average packed unsigned 16-bit integers in a and b, and store the results in dst.
      here = _mm256_avg_epu16(here, next);

      // _mm256_packus_epi16: Convert packed 16-bit integers from a and b to
      // packed 8-bit integers using unsigned saturation, and store the results in dst.
      here = _mm256_packus_epi16(here, here);

      // Store 64-bit integer from the first element of a into memory.
      _mm_storel_epi64(reinterpret_cast<__m128i*>(out), _mm256_extractf128_si256(here, 0));
      out += 8;
      _mm_storel_epi64(reinterpret_cast<__m128i*>(out), _mm256_extractf128_si256(here, 1));
      out += 8;

      in += 32;
      next_row += 32;
    }
    in += in_padding + in_stride;
    next_row += in_padding + in_stride;
    out += out_padding;
  }
}
#endif

#ifdef __SSE2__
void halfSample_SSE2(const uint8_t* in, uint8_t* out, int width, int height,
                     int in_stride, int out_stride)
{
  const uint64_t mask_data[2] = {0x00FF00FF00FF00FFull, 0x00FF00FF00FF00FFull};
  const uint8_t* next_row = in + width;

  // Load 128-bits of integer data from memory into dst. mem_addr must be aligned
  // on a 16-byte boundary or a general-protection exception may be generated.
  const __m128i mask = _mm_loadu_si128((const __m128i*)mask_data);
  const int width_by_16 = width >> 4;
  const int width_by_2 = height >> 1;
  const int in_padding = in_stride - width;
  const int out_width = width >> 1;
  const int out_padding = out_stride - out_width;
  for (int y = 0; y < width_by_2; y++)
  {
    for (int x = 0; x < width_by_16; x++)
    {
      __m128i here = _mm_load_si128((const __m128i*)in);
      __m128i next = _mm_load_si128((const __m128i*)next_row);

      // 1) (top + bottom + 1) / 2
      // Average packed unsigned 8-bit integers in a and b, and store the results in dst.
      here = _mm_avg_epu8(here, next);

      // 2) mask rows to prepare for summing left and right
      // _mm_srli_si128(a,b): Shift a right by b bytes while shifting in zeros, and store the results in dst.
      // Compute the bitwise AND of 128 bits (representing integer data) in a and b, and store the result in dst.
      next = _mm_and_si128(_mm_srli_si128(here,1), mask); // shift right by 8 and apply mask
      here = _mm_and_si128(here, mask);                   // apply mask

      // 3) Average left and right: (left + right + 1) / 2
      // Average packed unsigned 16-bit integers in a and b, and store the results in dst.
      here = _mm_avg_epu16(here, next);

      // _mm_packus_epi16: Convert packed 16-bit integers from a and b to
      // packed 8-bit integers using unsigned saturation, and store the results in dst.
      // Store 64-bit integer from the first element of a into memory.
      _mm_storel_epi64((__m128i*)out, _mm_packus_epi16(here, here));
      in += 16;
      next_row += 16;
      out += 8;
    }
    in += in_padding + in_stride;
    next_row += in_padding + in_stride;
    out += out_padding;
  }
}
#endif

#ifdef __ARM_NEON__
void halfSample_NEON(const uint8_t* in, uint8_t* out, int width, int height,
                     int in_stride, int out_stride)
{
  for( int y = 0; y < height; y += 2)
  {
    const uint8_t * in_top = in + y*in_stride;
    const uint8_t * in_bottom = in + (y+1)*in_stride;
    uint8_t * out_data = out + (y >> 1)*out_stride;
    for( int x = width; x > 0 ; x-=16, in_top += 16, in_bottom += 16, out_data += 8)
    {
      uint8x8x2_t top  = vld2_u8( (const uint8_t *)in_top );
      uint8x8x2_t bottom = vld2_u8( (const uint8_t *)in_bottom );
      uint16x8_t sum = vaddl_u8( top.val[0], top.val[1] );
      sum = vaddw_u8( sum, bottom.val[0] );
      sum = vaddw_u8( sum, bottom.val[1] );
      uint8x8_t final_sum = vshrn_n_u16(sum, 2);
      vst1_u8(out_data, final_sum);
    }
  }
}
#endif

} // namespace internal
} // namespace ze
