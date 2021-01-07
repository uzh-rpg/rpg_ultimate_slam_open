#ifndef __ARM_NEON__
# error "This file requires NEON support. Check your compiler flags."
#else
# include <arm_neon.h>
#endif

#include <imp/features/fast_masked.hpp>

#include <vector>

#include <fast/fast.h>
#include <fast/corner_9.h>
#include <fast/faster_corner_utilities.h>
#include <imp/features/occupancy_grid_2d.hpp>

namespace ze {

void fasterCornerDetect9MaskedImpl(
    const uint8_t* img, int img_width, int img_height, int img_stride,
    short barrier, const OccupancyGrid2D& grid, int level, int margin,
    std::vector<Corner>& corners)
{
  // Find log2 of cell_size, grid cell size has to be divisible by 16!
  DEBUG_CHECK_EQ(grid.cell_size_ % 16, 0);
  const int cell_size_log2 = grid.cellSizeLog2();

  const int img_stride_x3 = 3 * img_stride;

  // The compiler refuses to reserve a register for this
  register const uint8x16_t barriers = vdupq_n_u8((uint8_t)barrier);
  // cf. neon_test.m for more information about this mask
  register const uint8x16_t magic_mask =
      vcombine_u8(vcreate_u8(0x8040201008040201llu), vcreate_u8(0x8040201008040201llu));

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

      const uint8_t* p = img + y*img_stride + x;
      uint8x16_t lo, hi;
      {
        const uint8x16_t here = vld1q_u8((p));
        lo = vqsubq_u8(here, barriers);
        hi = vqaddq_u8(here, barriers);
      }
      unsigned int ans_0, ans_8, possible;
      {
        uint8x16_t top = vld1q_u8((p-img_stride_x3));
        uint8x16_t bottom = vld1q_u8((p+img_stride_x3));

        CHECK_BARRIER(lo, hi, top, ans_0);
        CHECK_BARRIER(lo, hi, bottom, ans_8);
        possible = ans_0 | ans_8;
        if (!possible)
          continue;
      }

      unsigned int ans_15, ans_1;
      {
        uint8x16_t a = vld1q_u8(p-1-img_stride_x3);
        // FIXXXME: very smart trick, since both a and c are in the same row
        //uint8x16_t c = _mm_insert_epi16(_mm_srli_si128(a,2), *(const unsigned short*)(p+15-stride3), 7);
        uint8x16_t c = vld1q_u8(p+1-img_stride_x3);
        CHECK_BARRIER(lo, hi, a, ans_15);
        CHECK_BARRIER(lo, hi, c, ans_1);
        possible &= ans_8 | (ans_15 & ans_1);
        if (!possible)
          continue;
      }

      unsigned int ans_9, ans_7;
      {
        uint8x16_t d = vld1q_u8(p-1+img_stride_x3);
        // FIXXXME: very smart trick, since both d and f are in the same row
        //uint8x16_t f = _mm_insert_epi16(_mm_srli_si128(d,2), *(const unsigned short*)(p+15+stride3), 7);
        uint8x16_t f = vld1q_u8(p+1+img_stride_x3);
        CHECK_BARRIER(lo, hi, d, ans_9);
        CHECK_BARRIER(lo, hi, f, ans_7);
        possible &= ans_9 | (ans_0 & ans_1);
        possible &= ans_7 | (ans_15 & ans_0);
        if (!possible)
          continue;
      }

      unsigned int ans_12, ans_4;
      {
        uint8x16_t left  = vld1q_u8(p-3);
        uint8x16_t right = vld1q_u8(p+3);
        CHECK_BARRIER(lo, hi, left, ans_12);
        CHECK_BARRIER(lo, hi, right, ans_4);
        possible &= ans_12 | (ans_4 & (ans_1 | ans_7));
        possible &= ans_4 | (ans_12 & (ans_9 | ans_15));
        if (!possible)
          continue;
      }

      unsigned int ans_14, ans_6;
      {
        uint8x16_t ul = vld1q_u8(p-2-2*img_stride);
        uint8x16_t lr = vld1q_u8(p+2+2*img_stride);
        CHECK_BARRIER(lo, hi, ul, ans_14);
        CHECK_BARRIER(lo, hi, lr, ans_6);
        {
          const unsigned int ans_6_7 = ans_6 & ans_7;
          possible &= ans_14 | (ans_6_7 & (ans_4 | (ans_8 & ans_9)));
          possible &= ans_1 | (ans_6_7) | ans_12;
        }
        {
          const unsigned int ans_14_15 = ans_14 & ans_15;
          possible &= ans_6 | (ans_14_15 & (ans_12 | (ans_0 & ans_1)));
          possible &= ans_9 | (ans_14_15) | ans_4;
        }
        if (!possible)
          continue;
      }

      unsigned int ans_10, ans_2;
      {
        uint8x16_t ll = vld1q_u8(p-2+2*img_stride);
        uint8x16_t ur = vld1q_u8(p+2-2*img_stride);
        CHECK_BARRIER(lo, hi, ll, ans_10);
        CHECK_BARRIER(lo, hi, ur, ans_2);
        {
          const unsigned int ans_1_2 = ans_1 & ans_2;
          possible &= ans_10 | (ans_1_2 & ((ans_0 & ans_15) | ans_4));
          possible &= ans_12 | (ans_1_2) | (ans_6 & ans_7);
        }
        {
          const unsigned int ans_9_10 = ans_9 & ans_10;
          possible &= ans_2 | (ans_9_10 & ((ans_7 & ans_8) | ans_12));
          possible &= ans_4 | (ans_9_10) | (ans_14 & ans_15);
        }
        possible &= ans_8 | ans_14 | ans_2;
        possible &= ans_0 | ans_10 | ans_6;
        if (!possible)
          continue;
      }

      unsigned int ans_13, ans_5;
      {
        uint8x16_t g = vld1q_u8(p-3-img_stride);
        uint8x16_t l = vld1q_u8(p+3+img_stride);
        CHECK_BARRIER(lo, hi, g, ans_13);
        CHECK_BARRIER(lo, hi, l, ans_5);
        const unsigned int ans_15_0 = ans_15 & ans_0;
        const unsigned int ans_7_8 = ans_7 & ans_8;
        {
          const unsigned int ans_12_13 = ans_12 & ans_13;
          possible &= ans_5 | (ans_12_13 & ans_14 & ((ans_15_0) | ans_10));
          possible &= ans_7 | (ans_1 & ans_2) | (ans_12_13);
          possible &= ans_2 | (ans_12_13) | (ans_7_8);
        }
        {
          const unsigned int ans_4_5 = ans_4 & ans_5;
          const unsigned int ans_9_10 = ans_9 & ans_10;
          possible &= ans_13 | (ans_4_5 & ans_6 & ((ans_7_8) | ans_2));
          possible &= ans_15 | (ans_4_5) | (ans_9_10);
          possible &= ans_10 | (ans_4_5) | (ans_15_0);
          possible &= ans_15 | (ans_9_10) | (ans_4_5);
        }

        possible &= ans_8 | (ans_13 & ans_14) | ans_2;
        possible &= ans_0 | (ans_5 & ans_6) | ans_10;
        if (!possible)
          continue;
      }

      unsigned int ans_11, ans_3;
      {
        uint8x16_t ii = vld1q_u8(p-3+img_stride);
        uint8x16_t jj = vld1q_u8(p+3-img_stride);
        CHECK_BARRIER(lo, hi, ii, ans_11);
        CHECK_BARRIER(lo, hi, jj, ans_3);
        {
          const unsigned int ans_2_3 = ans_2 & ans_3;
          possible &= ans_11 | (ans_2_3 & ans_4 & ((ans_0 & ans_1) | (ans_5 & ans_6)));
          possible &= ans_13 | (ans_7 & ans_8) | (ans_2_3);
          possible &= ans_8 | (ans_2_3) | (ans_13 & ans_14);
        }
        {
          const unsigned int ans_11_12 = ans_11 & ans_12;
          possible &= ans_3 | (ans_10 & ans_11_12 & ((ans_8 & ans_9) | (ans_13 & ans_14)));
          possible &= ans_1 | (ans_11_12) | (ans_6 & ans_7);
          possible &= ans_6 | (ans_0 & ans_1) | (ans_11_12);
        }
        {
          const unsigned int ans_3_4 = ans_3 & ans_4;
          possible &= ans_9 | (ans_3_4) | (ans_14 & ans_15);
          possible &= ans_14 | (ans_8 & ans_9) | (ans_3_4);
        }
        {
          const unsigned int ans_10_11 = ans_10 & ans_11;
          possible &= ans_5 | (ans_15 & ans_0) | (ans_10_11);
          possible &= ans_0 | (ans_10_11) | (ans_5 & ans_6);
        }
        if (!possible)
          continue;
      }

      possible |= (possible >> 16);

      //if(possible & 0x0f) //! @todo Does this make it faster?
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
      //if(possible & 0xf0) //! @todo Does this make it faster?
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

  fasterCornerDetect9MaskedImpl(
        img, img_width, img_height, img_stride, barrier, grid, level,
        margin, corners);
}

}
