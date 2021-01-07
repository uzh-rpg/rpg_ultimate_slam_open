// RPG-BSD License, Titus Cieslewski
// Heavily modified by ZE
// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved

#pragma once

#include <string>
#include <vector>
#include <rosbag/bag.h>

#include <imp/bridge/ros/ros_bridge.hpp>
#include <imp/core/image.hpp>
#include <ze/common/macros.hpp>

namespace ze {

class RosbagImageQuery
{
public:
  ZE_POINTER_TYPEDEFS(RosbagImageQuery);

  RosbagImageQuery() = default;
  RosbagImageQuery(const std::string& bagfile_path);
  ~RosbagImageQuery() = default;

  bool loadRosbag(const std::string& bagfile_path);

  StampedImage getStampedImageAtTime(
      const std::string& img_topic,
      const int64_t stamp_ns,
      const real_t search_range_ms = 10.0);

private:
  rosbag::Bag bag_;
};

}  // namespace ze
