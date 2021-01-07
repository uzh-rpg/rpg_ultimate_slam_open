#pragma once

#include <imp/features/feature_detector.hpp>
#include <imp/features/occupancy_grid_2d.hpp>
#include <opencv2/features2d.hpp>

namespace ze {

struct BriskDetectorOptions
{
  size_t brisk_num_octaves = 1; // swe setting

  real_t brisk_uniformity_radius = 0.0; // swe setting, okvis settings: 40

  real_t brisk_absolute_threshold = 45.0; // swe setting, okvis settings: 800

  size_t brisk_max_keypoints = 750; // swe setting

  int border_margin = 32;

  //! Maximum one feature per bucked with cell_size width and height.
  size_t cell_size = 20;
};

class BriskDetector : public AbstractDetector
{
public:

  BriskDetector(const BriskDetectorOptions& options, const Size2u& image_size);

  virtual ~BriskDetector() = default;

  //! Returns number of detected features.
  virtual uint32_t detect(const ImagePyramid8uC1& pyr, KeypointsWrapper& keypoints) override;

  inline virtual void setExistingKeypoints(const Eigen::Ref<const Keypoints>& keypoints) override
  {
    grid_.fillWithKeypoints(keypoints);
  }

  inline virtual void setMask(const Image8uC1::ConstPtr& mask) override
  {
    grid_.applyMask(mask);
  }

  inline virtual void reset() override
  {
    grid_.reset();
  }

private:
  BriskDetectorOptions options_;
  std::unique_ptr<cv::FeatureDetector> detector_;
  OccupancyGrid2D grid_;
};

} // namespace ze
