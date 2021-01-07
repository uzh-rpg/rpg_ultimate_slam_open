#include <imp/features/brisk_detector.hpp>

#include <brisk/brisk.h>

#include <imp/bridge/opencv/image_cv.hpp>
#include <imp/features/opencv_detector_utils.hpp>

namespace ze {

BriskDetector::BriskDetector(const BriskDetectorOptions& options,
                             const Size2u& image_size)
  : AbstractDetector(image_size, DetectorType::Brisk)
  , options_(options)
  , detector_(new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
                options_.brisk_num_octaves,
                options_.brisk_uniformity_radius,
                options_.brisk_absolute_threshold,
                options_.brisk_max_keypoints))
  , grid_(options_.cell_size, image_size_)
{}

uint32_t BriskDetector::detect(const ImagePyramid8uC1& pyr, KeypointsWrapper& features)
{
  int capacity = features.px.cols() - features.num_detected;
  if (capacity <= 0)
  {
    VLOG(100) << "Have no capacity for more corners. Skip BRISK detection.";
    return 0u;
  }

  ImageCv8uC1 cv_img(pyr.at(0));
  std::vector<cv::KeyPoint> keypoints;
  detector_->detect(cv_img.cvMat(), keypoints);

  // Apply mask:
  for (cv::KeyPoint& kp : keypoints)
  {
    if (grid_.isOccupied(kp.pt.x, kp.pt.y))
    {
      kp.response = -1.0;
    }
  }

  uint32_t added_features = copyKeypointsToFeatureWrapper(
        DetectorType::Brisk, image_size_, options_.border_margin, keypoints, features);
  VLOG(100) << "Stored " << added_features << " BRISK corners.";
  return added_features;
}

} // namespace ze
