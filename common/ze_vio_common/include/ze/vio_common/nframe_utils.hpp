// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <vector>
#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>

namespace ze {

// fwd
class Frame;
class NFrame;
class LandmarkTable;

std::vector<real_t> getDisparitiesSq(
    const NFrame& nframe_cur,
    const NFrame& nframe_ref);

void getDisparitiesSq(
    const Frame& frame_cur,
    const Frame& frame_ref,
    std::vector<real_t>& disparities_sq);

void setSceneDepthInNFrame(
    NFrame& nframe,
    const LandmarkTable& landmarks,
    const Transformation& T_B_W,
    const TransformationVector& T_C_B,
    const real_t default_min_depth,
    const real_t default_max_depth,
    const real_t default_median_depth);

//! @return Returns {camera_idx, num_features} of camera with least features.
std::pair<uint32_t, uint32_t> getCameraWithLeastFeatures(
    const NFrame& nframe,
    const LandmarkTable& landmarks);

//! @return Indices of cameras in rig without features.
std::vector<uint32_t> getCamerasWithoutFeatures(
    const NFrame& nframe);

} // namespace ze
