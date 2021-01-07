#pragma once

#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>

namespace ze {

// fwd
class Camera;

using PixelVector = std::vector<Vector2i, Eigen::aligned_allocator<Vector2i>>;

//! Return a vector of pixels lying on the epipolar line on specified scale-level.
PixelVector sampleGreatCircle(
    const Camera& cam,
    const real_t scale, // 1 << level
    const int border_margin,
    const Vector3& f_A,
    const Vector3& f_B,
    const Vector3& f_C,
    const int max_pixels_to_check);

} // namespace ze
