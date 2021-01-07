#pragma once

#include <vector>

#include <imp/core/image.hpp>
#include <ze/common/types.hpp>

namespace ze {

// -----------------------------------------------------------------------------
// Utils:
// TODO(cfo,mwe): Move where?

// Check if memory is 8bit aligned.
inline bool isAligned8(const void* ptr)
{
  return (reinterpret_cast<size_t>(ptr) & 0x7) == 0;
}

// Check if memory is 16bit aligned.
inline bool isAligned16(const void* ptr)
{
  return (reinterpret_cast<size_t>(ptr) & 0xF) == 0;
}

// Check if memory is 32bit aligned.
inline bool isAligned32(const void* ptr)
{
  return (reinterpret_cast<size_t>(ptr) & 0x3) == 0; // Check for 4 byte alignment
}

// -----------------------------------------------------------------------------

using ImgPyr8uC1 = std::vector<Image8uC1>;

//! Create image pyramid where every layer is downsampled by factor 2.
ImgPyr8uC1 createImgPyramid(const Image8uC1& in, int n_levels);

//! Downsample image by factor 2.
void halfSample(const Image8uC1& in, Image8uC1& out, bool use_simd = true);


namespace internal {

void halfSample_AVX2(const uint8_t* in, uint8_t* out, int width, int height,
                     int in_stride, int out_stride);

void halfSample_SSE2(const uint8_t* in, uint8_t* out, int width, int height,
                     int in_stride, int out_stride);

void halfSample_NEON(const uint8_t* in, uint8_t* out, int width, int height,
                     int in_stride, int out_stride);

} // namespace internal
} // namespace ze
