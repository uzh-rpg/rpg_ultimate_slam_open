#pragma once

#include <imp/features/feature_detector.hpp>

namespace ze {

//! Returns number of copied features.
//! If cv_keypoints is larger than the allocated feature storage then
//! cv_keypoints are being sorted.
uint32_t copyKeypointsToFeatureWrapper(
    const DetectorType feature_type,
    const Size2u& image_size,
    const real_t margin,
    std::vector<cv::KeyPoint>& cv_keypoints,
    KeypointsWrapper& ze_keypoints);

std::vector<cv::KeyPoint> copyFeatureWrapperToKeypoints(
    const KeypointsWrapper& ze_keypoints,
    const real_t feature_size);

} // namespace ze
