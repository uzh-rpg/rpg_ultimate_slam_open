#include <imp/features/brisk_descriptor.hpp>

#include <brisk/brisk.h>
#include <imp/bridge/opencv/image_cv.hpp>
#include <imp/features/opencv_detector_utils.hpp>
#include <ze/cameras/camera_utils.hpp>

namespace ze {

BriskDescriptorExtractor::BriskDescriptorExtractor(
    BriskDescriptorOptions options)
  : AbstractDescriptorExtractor(DescriptorType::Brisk)
  , extractor_(std::make_shared<brisk::BriskDescriptorExtractor>(
                 options.rotation_invariance,
                 options.scale_invariance))
{}

void BriskDescriptorExtractor::extract(
    const ImagePyramid8uC1& pyr,
    KeypointsWrapper& keypoints)
{
  if (keypoints.num_detected == 0u)
  {
    LOG(WARNING) << "No keypoints provided for descriptor extraction.";
    return;
  }

  std::vector<cv::KeyPoint> cv_keypoints = copyFeatureWrapperToKeypoints(keypoints, 12);
  ImageCv8uC1 cv_img(pyr.at(0));
  cv::Mat cv_descriptors;
  extractor_->compute(cv_img.cvMat(), cv_keypoints, cv_descriptors);

  // IMPORTANT: The opencv detector may add or remove keypoints!
  CHECK_EQ(cv_keypoints.size(), keypoints.num_detected);
  CHECK_EQ(cv_descriptors.type(), CV_8UC1);
  CHECK(cv_descriptors.isContinuous());

  // Switch rows, cols because Eigen is column-major.
  keypoints.descriptors = Eigen::Map<Descriptors>(cv_descriptors.data,
                                                  cv_descriptors.cols,
                                                  cv_descriptors.rows);
}

} // namespace ze
