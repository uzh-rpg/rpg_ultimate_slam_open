#pragma once

#include <memory>
#include <imp/features/feature_descriptor.hpp>
#include <imp/features/keypoints_wrapper.hpp>
#include <imp/imgproc/image_pyramid.hpp>
#include <opencv2/features2d.hpp>

namespace ze {

struct BriskDescriptorOptions
{
  bool rotation_invariance = false;
  bool scale_invariance = false;
};

class BriskDescriptorExtractor : public AbstractDescriptorExtractor
{
public:
  BriskDescriptorExtractor(
      BriskDescriptorOptions options = BriskDescriptorOptions());
  virtual ~BriskDescriptorExtractor() = default;

  virtual void extract(
      const ImagePyramid8uC1& pyr,
      KeypointsWrapper& keypoints) override;

private:
  std::shared_ptr<cv::DescriptorExtractor> extractor_;
};

} // namespace ze
