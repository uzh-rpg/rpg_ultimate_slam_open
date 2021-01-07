#include <imp/features/opencv_detector_utils.hpp>

#include <algorithm>
#include <ze/cameras/camera_utils.hpp>

namespace ze {

//------------------------------------------------------------------------------
uint32_t copyKeypointsToFeatureWrapper(
    const DetectorType feature_type,
    const Size2u& image_size,
    const real_t margin,
    std::vector<cv::KeyPoint>& keypoints,
    KeypointsWrapper& features)
{
  int capacity = features.px.cols() - features.num_detected;
  CHECK_GT(capacity, 0);

  if (static_cast<int>(keypoints.size()) > capacity)
  {
    // We have too many keypoints. We therefore need to sort them according to
    // their score.
    std::sort(keypoints.begin(), keypoints.end(),
              [](const cv::KeyPoint& c1, const cv::KeyPoint& c2)
    { return c1.response > c2.response; });
  }

  uint32_t n_features = 0u;
  uint32_t n_features_close_to_border = 0u;
  for (const cv::KeyPoint& kp : keypoints)
  {
    if (kp.response < 0.0f) // Masked keypoints are set to -1.
    {
      continue;
    }

    if (!isVisibleWithMargin(image_size.width(), image_size.height(),
                             kp.pt.x, kp.pt.y,
                             (kp.octave + 1) * margin))
    {
      ++n_features_close_to_border;
      continue;
    }

    if (!features.addKeypoint(
         kp.pt.x, kp.pt.y, kp.response, kp.octave, kp.angle,
         static_cast<uint8_t>(feature_type)))
    {
      break;
    }
    ++n_features;
  }
  return n_features;
}

//------------------------------------------------------------------------------
std::vector<cv::KeyPoint> copyFeatureWrapperToKeypoints(
    const KeypointsWrapper& ze_keypoints,
    const real_t feature_size)
{
  std::vector<cv::KeyPoint> keypoints;
  keypoints.reserve(ze_keypoints.num_detected);
  for (uint32_t i = 0u; i < ze_keypoints.num_detected; ++i)
  {
    auto px = ze_keypoints.px.col(i);
    keypoints.push_back(cv::KeyPoint(cv::Point2f(px(0), px(1)),
                                     feature_size,
                                     ze_keypoints.angles(i),
                                     ze_keypoints.scores(i),
                                     ze_keypoints.levels(i)));
  }
  return keypoints;
}

} // namespace ze
