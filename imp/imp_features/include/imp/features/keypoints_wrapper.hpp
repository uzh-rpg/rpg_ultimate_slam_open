#pragma once

#include <ze/common/logging.hpp>
#include <ze/common/types.hpp>

namespace ze {

//! A wrapper that holds references to Keypoints. We typically allocate memory
//! for the maximum amount of keypoints that we desire and num_detected
//! tells us how many we actually have.
struct KeypointsWrapper
{
  Keypoints& px;
  KeypointScores& scores;
  KeypointLevels& levels;
  KeypointAngles& angles;
  KeypointTypes& types;
  Descriptors& descriptors;
  uint32_t& num_detected;

  KeypointsWrapper() = delete;

  KeypointsWrapper(
      Keypoints& px, KeypointScores& scores, KeypointLevels& levels,
      KeypointAngles& angles, KeypointTypes& types,
      Descriptors& descriptors, uint32_t& num_detected)
    : px(px), scores(scores), levels(levels), angles(angles), types(types),
      descriptors(descriptors), num_detected(num_detected)
  {
    DEBUG_CHECK_EQ(px.cols(), scores.size());
    DEBUG_CHECK_EQ(px.cols(), levels.size());
    DEBUG_CHECK_EQ(px.cols(), angles.size());
    DEBUG_CHECK_EQ(px.cols(), types.size());
    DEBUG_CHECK_GE(px.cols(), static_cast<int>(num_detected));
  }

  inline bool addKeypoint(
      real_t x, real_t y, real_t score, uint8_t level,
      real_t angle, uint8_t type)
  {
    if (num_detected < static_cast<uint32_t>(px.cols()))
    {
      px(0, num_detected) = x;
      px(1, num_detected) = y;
      scores(num_detected) = score;
      levels(num_detected) = level;
      angles(num_detected) = angle;
      types(num_detected) = type;
      ++num_detected;
      return true;
    }
    return false;
  }
};

} // namespace ze
