// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>
#include <ze/vio_common/landmark_handle.hpp>
#include <ze/vio_common/landmark_types.hpp>
#include <ze/vio_common/motion_type.hpp>

namespace ze {

// fwd;
class CameraRig;
class LandmarkTable;
class NFrame;
class NFrameTable;

void selectLandmarksForBackend(
    const VioMotionType motion_type,
    const TransformationVector& T_C_B,
    const NFrameTable& states,
    NFrame& nframe_kp1,
    LandmarkTable& landmarks,
    std::vector<LandmarkHandle>& lm_opportunistic,
    std::vector<LandmarkHandle>& lm_persistent_new,
    std::vector<LandmarkHandle>& lm_persistent_continued);

/*!
 * @brief Splits a list of landmarks into N good ones and the rest.
 * @return First vector: N good quality landmarks. Second vector: remaining landmarks.
 */
std::pair<std::vector<LandmarkHandle>, std::vector<LandmarkHandle>>
splitNBestLandmarks(
    std::vector<std::pair<LandmarkHandle, int>>& weighted_lm,
    const size_t N);

bool insertIfGoodQualityLandmark(
    LandmarkTable& landmarks,
    const uint32_t slot,
    std::vector<std::pair<LandmarkHandle, int>>& container,
    const TransformationVector& T_C_B,
    const NFrameTable& states,
    const NFrameHandle* ignore_nframe = nullptr);

void splitAndInitializeNewLandmarks(
    const std::vector<LandmarkHandle>& lm_ids,
    LandmarkTable& landmarks,
    NFrame& nframe_kp1);

} // namespace ze
