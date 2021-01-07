#pragma once

#include <fast/fast.h>
#include <ze/common/types.hpp>

namespace ze {

// fwd
class OccupancyGrid2D;
using Corner = fast::fast_xy;

void fasterCornerDetectMasked(
    const uint8_t* img, int img_width, int img_height, int img_stride,
    short barrier, const OccupancyGrid2D& grid, int level, int margin,
    std::vector<Corner>& corners);

} // namespace ze
