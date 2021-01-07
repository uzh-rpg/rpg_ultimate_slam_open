// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <map>
#include <vector>
#include <ze/common/logging.hpp>

#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>
#include <ze/vio_common/landmark_handle.hpp>
#include <ze/vio_common/landmark_types.hpp>

namespace ze {

// fwd
class NFrameTable;
class LandmarkTable;

enum class TriangulationResult
{
  Success,
  FailTooFewObservations,
  FailNoAnchorObservation,
  FailStatesNotAvailable,
  FailTriangulation,
  FailNegativeDepth,
  FailTooFar,
  FailTooClose
};

//! Triangulate a landmark.
std::pair<Vector3, TriangulationResult> triangulateLandmark(
    const NFrameTable& states,
    const TransformationVector& T_C_B,
    const LandmarkObsVec& obs_vec,
    const real_t clip_close = 0.1,
    const real_t clip_far = 1000.0,
    const NFrameHandle* ignore_nframe = nullptr);

//! Iterate over landmark table and retrianglate all landmarks with new camera poses.
uint32_t retriangulateAllLandmarks(
    const NFrameTable& states,
    const TransformationVector& T_C_B,
    LandmarkTable& landmarks,
    bool set_all_failures_to_outlier = false);

//! Get max triangulation angle.
real_t getLandmarkParallaxDegrees(
    const NFrameTable& states,
    const TransformationVector& T_C_B,
    const LandmarkObsVec& obs,
    const Position& p_W,
    const NFrameHandle* ignore_nframe,
    const uint8_t* frame_idx);

//! Gauss-Newton to minimize the reprojection error.
void optimizeLandmark(
    const NFrameTable& states,
    const TransformationVector& T_C_B,
    const LandmarkHandle& lm_h,
    LandmarkTable& landmarks);

} // namespace ze
