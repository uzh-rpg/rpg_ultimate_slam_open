#include <imp/features/fast_masked.hpp>

#include <vector>
#ifndef __SSE2__
# error "This file requires SSE2 support. Check your compiler flags."
#else
# include <emmintrin.h>
#endif

#include <fast/fast.h>
#include <fast/corner_10.h>
#include <fast/faster_corner_utilities.h>
#include <imp/features/occupancy_grid_2d.hpp>

namespace ze {

template <bool Aligned>
void fasterCornerDetect10MaskedImpl(
    const uint8_t* img, int img_width, int img_height, int img_stride,
    short barrier, const OccupancyGrid2D& grid, int level, int margin,
    std::vector<Corner>& corners)
{
  // Find log2 of cell_size, grid cell size has to be divisible by 16!
  DEBUG_CHECK_EQ(grid.cell_size_ % 16, 0);
  const int cell_size_log2 = grid.cellSizeLog2();

  const int img_stride_x3 = 3*img_stride;

  // The compiler refuses to reserve a register for this
  register const __m128i barriers = _mm_set1_epi8((uint8_t)barrier);

  // Compute margin, in x-direction it should be multiple of 16 and not smaller
  // than 3. In y-direction only important that it isn't smaller than 3.
  margin = std::max(3, margin);
  int xbegin = margin;
  if (xbegin % 16 > 0)
  {
    xbegin += (16 - xbegin % 16);
  }
  int xend = img_width - margin;
  xend -= xend % 16;
  DEBUG_CHECK_GE(xend, 0);
  DEBUG_CHECK_LT(xbegin, img_width);
  DEBUG_CHECK(xbegin % 16 == 0);
  DEBUG_CHECK(xend % 16 == 0);

  // Compute features.
  for (int y = margin; y < img_height - margin; ++y)
  {
    //
    // Skip first 16 rows.
    //

    // Efficient grid access:
    int grid_row = ((y << level) >> cell_size_log2);
    const int8_t* grid_ptr = grid.occupancy_.data() + grid_row * grid.n_cols_;

    for (int x = xbegin ; x < xend; x += 16)
    {
      if (grid_ptr[((x << level) >> cell_size_log2)])
      {
        continue;
      }

      const uint8_t* p = (uint8_t*)img + y*img_stride + x;
      __m128i lo, hi;
      {
        const __m128i here = fast::load_si128<Aligned>((const __m128i*)(p));
        lo = _mm_subs_epu8(here, barriers);
        hi = _mm_adds_epu8(barriers, here);
      }
      unsigned int ans_b, ans_e;
      {
        __m128i top = fast::load_si128<Aligned>((const __m128i*)(p-img_stride_x3));
        __m128i bottom = fast::load_si128<Aligned>((const __m128i*)(p+img_stride_x3));

        CHECK_BARRIER(lo, hi, top, ans_b);
        CHECK_BARRIER(lo, hi, bottom, ans_e);
        if (!(ans_b | ans_e))
          continue;
      }

      unsigned int ans_m, ans_p, possible;
      {
        __m128i ul = _mm_loadu_si128((const __m128i*)(p-2-2*img_stride));
        __m128i lr = _mm_loadu_si128((const __m128i*)(p+2+2*img_stride));
        CHECK_BARRIER(lo, hi, ul, ans_m);
        CHECK_BARRIER(lo, hi, lr, ans_p);
        possible = (ans_m & ans_b) | (ans_e & ans_p);
        if (!possible)
          continue;
      }

      unsigned int ans_o, ans_n;
      {
        __m128i ll = _mm_loadu_si128((const __m128i*)(p-2+2*img_stride));
        __m128i ur = _mm_loadu_si128((const __m128i*)(p+2-2*img_stride));
        CHECK_BARRIER(lo, hi, ll, ans_o);
        CHECK_BARRIER(lo, hi, ur, ans_n);
        possible &= ans_o | (ans_b & ans_n);
        possible &= ans_n | (ans_e & ans_o);
        if (!possible)
          continue;
      }

      unsigned int ans_h, ans_k;
      {
        __m128i left = _mm_loadu_si128((const __m128i*)(p-3));
        __m128i right = _mm_loadu_si128((const __m128i*)(p+3));
        CHECK_BARRIER(lo, hi, left, ans_h);
        CHECK_BARRIER(lo, hi, right, ans_k);
        possible &= ans_h | (ans_n & ans_k & ans_p);
        possible &= ans_k | (ans_m & ans_h & ans_o);
        if (!possible)
          continue;
      }

      unsigned int ans_a, ans_c;
      {
        __m128i a = _mm_loadu_si128((const __m128i*)(p-1-img_stride_x3));
        __m128i c = _mm_insert_epi16(_mm_srli_si128(a,2), *(const unsigned short*)(p+15-img_stride_x3), 7);
        //__m128i c = _mm_loadu_si128((const __m128i*)(p+1-stride));
        CHECK_BARRIER(lo, hi, a, ans_a);
        CHECK_BARRIER(lo, hi, c, ans_c);
        possible &= ans_a | (ans_e & ans_p);
        possible &= ans_c | (ans_o & ans_e);
        if (!possible)
          continue;
      }

      unsigned int ans_d, ans_f;
      {
        __m128i d = _mm_loadu_si128((const __m128i*)(p-1+img_stride_x3));
        __m128i f = _mm_insert_epi16(_mm_srli_si128(d,2), *(const unsigned short*)(p+15+img_stride_x3), 7);
        //__m128i f = _mm_loadu_si128((const __m128i*)(p+1+stride));
        CHECK_BARRIER(lo, hi, d, ans_d);
        CHECK_BARRIER(lo, hi, f, ans_f);
        const unsigned int ans_abc = ans_a & ans_b & ans_c;
        possible &= ans_d | (ans_abc & ans_n);
        possible &= ans_f | (ans_m & ans_abc);
        if (!possible)
          continue;
      }

      unsigned int ans_g, ans_i;
      {
        __m128i g = _mm_loadu_si128((const __m128i*)(p-3-img_stride));
        __m128i ii = _mm_loadu_si128((const __m128i*)(p-3+img_stride));
        CHECK_BARRIER(lo, hi, g, ans_g);
        CHECK_BARRIER(lo, hi, ii, ans_i);
        possible &= ans_g | (ans_f & ans_p & ans_k);
        possible &= ans_i | (ans_c & ans_n & ans_k);
        if (!possible)
          continue;
      }

      unsigned int ans_j, ans_l;
      {
        __m128i jj = _mm_loadu_si128((const __m128i*)(p+3-img_stride));
        __m128i l = _mm_loadu_si128((const __m128i*)(p+3+img_stride));
        CHECK_BARRIER(lo, hi, jj, ans_j);
        CHECK_BARRIER(lo, hi, l, ans_l);
        const unsigned int ans_ghi = ans_g & ans_h & ans_i;
        possible &= ans_j | (ans_d & ans_o & ans_ghi);
        possible &= ans_l | (ans_m & ans_a & ans_ghi);
        if (!possible)
          continue;
      }

      possible |= (possible >> 16);
      //if(possible & 0x0f) //Does this make it faster?
      {
        if(possible & (1<< 0))
          corners.push_back(Corner(x + 0, y));
        if(possible & (1<< 1))
          corners.push_back(Corner(x + 1, y));
        if(possible & (1<< 2))
          corners.push_back(Corner(x + 2, y));
        if(possible & (1<< 3))
          corners.push_back(Corner(x + 3, y));
        if(possible & (1<< 4))
          corners.push_back(Corner(x + 4, y));
        if(possible & (1<< 5))
          corners.push_back(Corner(x + 5, y));
        if(possible & (1<< 6))
          corners.push_back(Corner(x + 6, y));
        if(possible & (1<< 7))
          corners.push_back(Corner(x + 7, y));
      }
      //if(possible & 0xf0) //Does this make it faster?
      {
        if(possible & (1<< 8))
          corners.push_back(Corner(x + 8, y));
        if(possible & (1<< 9))
          corners.push_back(Corner(x + 9, y));
        if(possible & (1<<10))
          corners.push_back(Corner(x +10, y));
        if(possible & (1<<11))
          corners.push_back(Corner(x +11, y));
        if(possible & (1<<12))
          corners.push_back(Corner(x +12, y));
        if(possible & (1<<13))
          corners.push_back(Corner(x +13, y));
        if(possible & (1<<14))
          corners.push_back(Corner(x +14, y));
        if(possible & (1<<15))
          corners.push_back(Corner(x +15, y));
      }
    }

    //
    // Skip remaining rows
    //
  }
}

void fasterCornerDetectMasked(
    const uint8_t* img, int img_width, int img_height, int img_stride,
    short barrier, const OccupancyGrid2D& grid, int level, int margin,
    std::vector<Corner>& corners)
{
  if (img_width < 22 || img_height < 7)
  {
    LOG(WARNING) << "Image too small";
    return;
  }

  if (fast::is_aligned<16>(img) && fast::is_aligned<16>(img+img_stride))
  {
    fasterCornerDetect10MaskedImpl<true>(
          img, img_width, img_height, img_stride, barrier, grid, level,
          margin, corners);
  }
  else
  {
    fasterCornerDetect10MaskedImpl<false>(
          img, img_width, img_height, img_stride, barrier, grid, level,
          margin, corners);
  }
}

} // namespace ze
