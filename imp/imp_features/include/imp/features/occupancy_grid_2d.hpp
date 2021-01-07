#pragma once

#include <ze/common/logging.hpp>
#include <ze/common/types.hpp>
#include <imp/core/size.hpp>
#include <imp/core/image.hpp>

namespace ze {

//! We divide the image into a grid of cells and try to find maximally one
//! feature per cell. This is to ensure good distribution of features in the
//! image.
class OccupancyGrid2D
{
public:
  using Grid = std::vector<int8_t>;

  OccupancyGrid2D(int cell_size, const Size2u& image_size)
    : img_size_(image_size.width(), image_size.height())
    , cell_size_(cell_size)
    , n_cols_(real_t(img_size_.width()) / cell_size_ + 1)
    , n_rows_(real_t(img_size_.height()) / cell_size_ + 1)
    , occupancy_(n_cols_ * n_rows_, 0)
  {}

  const Size2i img_size_;
  const int cell_size_;
  const int n_cols_;
  const int n_rows_;
  Grid occupancy_;

  inline void reset()
  {
    std::fill(occupancy_.begin(), occupancy_.end(), 0);
  }

  inline int cellSizeLog2() const
  {
    DEBUG_CHECK_EQ(cell_size_ % 2, 0);
    int cell_size_log2 = 2;
    for ( ; cell_size_log2 < 10; ++cell_size_log2)
    {
      if (cell_size_ >> cell_size_log2 == 1)
      {
        break;
      }
    }
    LOG_IF(FATAL, cell_size_log2 == 10) << "log2 could not be computed.";
    return cell_size_log2;
  }

  inline size_t size()
  {
    return occupancy_.size();
  }

  inline bool empty()
  {
    return occupancy_.empty();
  }

  inline bool isOccupied(const size_t cell_index) const
  {
    DEBUG_CHECK_LT(cell_index, occupancy_.size());
    return occupancy_[cell_index];
  }

  inline bool isOccupied(const int x, const int y) const
  {
    return isOccupied(getCellIndex(x, y));
  }

  inline bool isOccupied(const int x_at_level, const int y_at_level, real_t inv_scale) const
  {
    return isOccupied(getCellIndex(x_at_level, y_at_level, inv_scale));
  }

  inline void setOccupied(const size_t cell_index)
  {
    DEBUG_CHECK_LT(cell_index, occupancy_.size());
    occupancy_[cell_index] = 1;
  }

  inline size_t getCellIndex(const int x, const int y) const
  {
    const int row = y / cell_size_; //! @todo: integer division is costly,
    const int col = x / cell_size_; //! use shift operator when cell size is power of 2
    DEBUG_CHECK_LT(row, n_rows_);
    DEBUG_CHECK_LT(col, n_cols_);
    return row * n_cols_ + col;
  }

  //! @param inv_scale = (1 << level)
  inline size_t getCellIndex(
      const int x_at_level, const int y_at_level, real_t inv_scale) const
  {
    const int row = (inv_scale * y_at_level) / cell_size_;
    const int col = (inv_scale * x_at_level) / cell_size_;
    DEBUG_CHECK_LT(row, n_rows_);
    DEBUG_CHECK_LT(col, n_cols_);
    return row * n_cols_ + col;
  }

  inline void fillWithKeypoints(const Eigen::Ref<const Keypoints>& keypoints)
  {
    for (int i = 0; i < keypoints.cols(); ++i)
    {
      Vector2i px = keypoints.col(i).cast<int>();
      DEBUG_CHECK_GE(px(0), 0.0);
      DEBUG_CHECK_GE(px(1), 0.0);
      DEBUG_CHECK_LT(px(0), img_size_.width());
      DEBUG_CHECK_LT(px(1), img_size_.height());
      size_t cell_index = getCellIndex(px(0), px(1), 1.0f);
      DEBUG_CHECK_LT(cell_index, occupancy_.size());
      occupancy_[cell_index] = 1;
    }
  }

  inline void applyMask(const Image8uC1::ConstPtr& mask)
  {
    int cell_halfsize = 0.5 * cell_size_;
    for (int r = 0; r < n_rows_; ++r)
    {
      for (int c = 0; c < n_cols_; ++c)
      {
        int x = c * cell_size_ + cell_halfsize;
        int y = r * cell_size_ + cell_halfsize;
        if (mask->pixel(x,y) == 0)
        {
          setOccupied(r * n_cols_ + c);
        }
      }
    }
  }

  void visualizeGrid(Image8uC1& img, real_t scale = 1.0);
};

// Convenience typedefs.
using OccupancyGrid2dVector = std::vector<OccupancyGrid2D>;

} // namespace ze
