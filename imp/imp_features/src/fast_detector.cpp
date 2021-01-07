#include <imp/features/fast_detector.hpp>

#include <algorithm>
#include <fast/fast.h>

#include <imp/features/fast_masked.hpp>

namespace ze {

//------------------------------------------------------------------------------
//! Temporary container used for fast corner detection.
struct FastCorner
{
  real_t x;
  real_t y;
  int level;
  float score;

  FastCorner(float score)
    : x(-1.0f), y(-1.0f), level(-1), score(score)
  {}
};
using FastCorners = std::vector<FastCorner>;

//------------------------------------------------------------------------------
FastDetector::FastDetector(const FastDetectorOptions& options,
                           const Size2u& image_size)
  : AbstractDetector(image_size, DetectorType::Fast)
  , options_(options)
  , grid_(options_.cell_size, image_size_)
{}

//------------------------------------------------------------------------------
uint32_t FastDetector::detect(const ImagePyramid8uC1& pyr, KeypointsWrapper& features)
{
  CHECK_LE(static_cast<size_t>(options_.max_level), pyr.numLevels() - 1);

  int capacity = features.px.cols() - features.num_detected;
  if (capacity <= 0)
  {
    VLOG(100) << "Have no capacity for more corners. Skip FAST detection.";
    return 0u;
  }

  FastCorners corners(grid_.size(), FastCorner(options_.threshold));
  if (options_.use_masked_detection)
  {
    // -------------------------------------------------------------------------
    // Pyramidal feature detection with mask
    for (int level = options_.min_level; level <= options_.max_level; ++level)
    {
      CHECK_DOUBLE_EQ(pyr.scaleFactor(), 0.5);

      // Detect corners.
      std::vector<fast::fast_xy> fast_corners;
      fasterCornerDetectMasked(
            reinterpret_cast<const uint8_t*>(pyr[level].data()),
            pyr[level].width(), pyr[level].height(), pyr[level].stride(),
            options_.threshold, grid_, level, options_.border_margin, fast_corners);

      // Compute corner score (fast score).
      std::vector<int> scores;
      scores.reserve(fast_corners.size());
      fast::fast_corner_score_10(
            reinterpret_cast<const fast::fast_byte*>(pyr[level].data()),
            pyr[level].stride(), fast_corners, options_.threshold, scores);

      // Compute best corner for each cell.
      const int cell_size_log2 = grid_.cellSizeLog2();
      for (size_t i = 0u; i < fast_corners.size(); ++i)
      {
        const fast::fast_xy& xy = fast_corners[i];

        const int row = (xy.y << level) >> cell_size_log2;
        const int col = (xy.x << level) >> cell_size_log2;
        DEBUG_CHECK_LT(row, grid_.n_rows_);
        DEBUG_CHECK_LT(col, grid_.n_cols_);
        const int k = row * grid_.n_cols_ + col;
        const float score = scores[i];
        if (score > corners[k].score)
        {
          FastCorner& c = corners[k];
          c.x = xy.x << level;
          c.y = xy.y << level;
          c.score = score;
          c.level = level;
        }
      }
    }
  }
  else
  {
    // -------------------------------------------------------------------------
    // Pyramidal feature detection
    for (int level = options_.min_level; level <= options_.max_level; ++level)
    {
      const real_t inv_scale = 1.0f / pyr.scaleFactor(level);
      std::vector<fast::fast_xy> fast_corners;

      // Detect corners.
#ifdef __ARM_NEON__
      fast::fast_corner_detect_9_neon(
            reinterpret_cast<const fast::fast_byte*>(pyr[level].data()),
            pyr[level].width(), pyr[level].height(), pyr[level].stride(),
            options_.threshold, fast_corners);
#else
      fast::fast_corner_detect_10_sse2(
            reinterpret_cast<const fast::fast_byte*>(pyr[level].data()),
            pyr[level].width(), pyr[level].height(), pyr[level].stride(),
            options_.threshold, fast_corners);
#endif

      // Compute corner score (fast score).
      std::vector<int> scores;
      scores.reserve(fast_corners.size());
      fast::fast_corner_score_10(
            reinterpret_cast<const fast::fast_byte*>(pyr[level].data()),
            pyr[level].stride(), fast_corners, options_.threshold, scores);

      // Perform non-maxima supression in 3x3 window.
      std::vector<int> nm_corners;
      nm_corners.reserve(fast_corners.size());
      fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

      // Check for each corner if it has higher score than the one already in cell.
      const int maxw = pyr[level].width() - options_.border_margin;
      const int maxh = pyr[level].height() - options_.border_margin;
      for (const int& i : nm_corners)
      {
        const fast::fast_xy& xy = fast_corners.at(i);
        if(xy.x < options_.border_margin || xy.y < options_.border_margin
           || xy.x >= maxw || xy.y >= maxh)
        {
          continue;
        }

        const size_t k = grid_.getCellIndex(xy.x, xy.y, inv_scale);
        if (grid_.occupancy_.at(k))
        {
          continue;
        }

        const float score = scores.at(i);
        if (score > corners.at(k).score)
        {
          FastCorner& c = corners[k];
          c.x = xy.x * inv_scale;
          c.y = xy.y * inv_scale;
          c.score = score;
          c.level = level;
        }
      }
    }
  }

  // Count number of corners with high score.
  int num_corners = std::count_if(
        corners.cbegin(), corners.cend(),
        [&](const FastCorner& c) { return c.score > options_.threshold; });
  VLOG(100) << "Detected " << num_corners << " FAST corners.";

  // Add features with highest score to storage.
  if (num_corners > capacity)
  {
    std::sort(corners.begin(), corners.end(),
              [](const FastCorner& c1, const FastCorner& c2) { return c1.score > c2.score; });
  }
  uint32_t added_features = 0;
  for (const FastCorner& c : corners)
  {
    if (c.score > options_.threshold)
    {
      if (!features.addKeypoint(c.x, c.y, c.score, c.level, 0.0f,
                               static_cast<uint8_t>(DetectorType::Fast)))
      {
        break;
      }
      ++added_features;
    }
  }
  VLOG(100) << "Stored " << added_features << " FAST corners.";
  return added_features;
}

} // namespace ze
