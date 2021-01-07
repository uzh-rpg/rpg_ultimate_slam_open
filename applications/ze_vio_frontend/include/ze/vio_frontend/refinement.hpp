// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <ze/common/transformation.hpp>
#include <ze/common/types.hpp>

namespace ze {

// fwd
class CameraRig;
class LandmarkTable;
class NFrame;
class NFrameTable;

void optimizeLandmarks(
    const CameraRig& rig,
    const NFrameTable& states,
    NFrame& nframe,
    LandmarkTable& landmarks);

} // namespace ze
