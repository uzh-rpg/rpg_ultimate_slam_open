// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>
#include <ze/vio_common/landmark_types.hpp>

namespace ze {

// fwd
class LandmarkTable;
class NFrame;

//! @return Number of removed landmarks.
int removeOldLandmarks(
    const uint32_t max_num_landmarks_to_keep,
    const uint32_t current_frame_index,
    LandmarkTable& landmarks);

uint32_t addLandmarkObservations(
    const NFrame& nframe,
    LandmarkTable& landmarks);

//! Set type of all landmarks that are observed by NFrame.
uint32_t setTypeOfConvergedSeedsInNFrame(
    const NFrame& nframe,
    const LandmarkType type,
    LandmarkTable& landmarks);

void setLandmarksLastObservationInNFrame(
    const NFrame& nframe,
    LandmarkTable& landmarks);

} // namespace ze
