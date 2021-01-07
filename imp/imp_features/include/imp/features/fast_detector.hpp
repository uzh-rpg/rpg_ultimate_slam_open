#pragma once

#include <imp/features/feature_detector.hpp>
#include <imp/features/occupancy_grid_2d.hpp>
#include <imp/core/size.hpp>

namespace ze {

struct FastDetectorOptions
{
  //! Maximum one feature per bucked with cell_size width and height.
  int cell_size = 32;

  //! Extract features up to this pyramid level.
  int max_level = 2;

  //! Minimum pyramid level at which features should be selected.
  int min_level = 0;

  //! No feature should be within border.
  int border_margin = 8;

  //! Corner threshold.
  int threshold = 10;

  //! Use masked detection.
  int use_masked_detection = true;
};

class FastDetector : public AbstractDetector
{
public:
  FastDetector(const FastDetectorOptions& options, const Size2u& image_size);

  virtual ~FastDetector() = default;

  virtual uint32_t detect(const ImagePyramid8uC1& pyr, KeypointsWrapper& keypoints) override;

  inline virtual void setExistingKeypoints(const Eigen::Ref<const Keypoints>& keypoints) override
  {
    grid_.fillWithKeypoints(keypoints);
  }

  inline virtual void setMask(const Image8uC1::ConstPtr& mask) override
  {
    grid_.applyMask(mask);
  }

  inline virtual void setGrid(const OccupancyGrid2D& grid) override
  {
    DEBUG_CHECK_EQ(grid_.img_size_, grid.img_size_);
    DEBUG_CHECK_EQ(grid_.cell_size_, grid.cell_size_);
    grid_.occupancy_ = grid.occupancy_;
  }

  inline virtual void reset() override
  {
    grid_.reset();
  }

private:
  FastDetectorOptions options_;
  OccupancyGrid2D grid_;
};

} // namespace ze
