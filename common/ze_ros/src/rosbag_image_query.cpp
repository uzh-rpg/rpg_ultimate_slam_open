// RPG-BSD License, Titus Cieslewski
// Heavily modified by ZE
// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved

#include <ze/ros/rosbag_image_query.hpp>

#include <glog/logging.h>
#include <rosbag/view.h>
#include <ze/common/file_utils.hpp>
#include <ze/common/time_conversions.hpp>

namespace ze {

RosbagImageQuery::RosbagImageQuery(const std::string& bagfile_path)
{
  CHECK(loadRosbag(bagfile_path));
}

bool RosbagImageQuery::loadRosbag(const std::string& bagfile_path)
{
  CHECK(fileExists(bagfile_path)) << "File does not exist: " << bagfile_path;
  try
  {
    VLOG(1) << "Opening rosbag " << bagfile_path << " ...";
    bag_.open(bagfile_path, rosbag::BagMode::Read);
  }
  catch (rosbag::BagException& exception)
  {
    LOG(ERROR) << "Could not open rosbag: " << bagfile_path << ": " << exception.what();
    return false;
  }
  return true;
}

StampedImage RosbagImageQuery::getStampedImageAtTime(
    const std::string& img_topic,
    const int64_t stamp_ns,
    const real_t search_range_ms)
{
  // Considering rounding errors.
  const int64_t search_range_ns = millisecToNanosec(search_range_ms);
  ros::Time time_min, time_max;
  time_min.fromNSec(stamp_ns - search_range_ns);
  time_max.fromNSec(stamp_ns + search_range_ns);
  rosbag::View view(bag_, rosbag::TopicQuery(img_topic), time_min, time_max);

  VLOG(100) << "Found messages that fit = " << view.size();
  int64_t best_time_diff = std::numeric_limits<int64_t>::max();
  sensor_msgs::ImageConstPtr best_match_message;
  for (const rosbag::MessageInstance& message : view)
  {
    const int64_t time_diff =
        std::abs(static_cast<int64_t>(message.getTime().toNSec()) - stamp_ns);
    if (time_diff < best_time_diff)
    {
      best_time_diff = time_diff;
      best_match_message = message.instantiate<sensor_msgs::Image>();
      CHECK(best_match_message);
    }
    else
    {
      break; // Already passed relevant time.
    }
  }

  // Extract image
  int64_t best_match_stamp = -1;
  ImageBase::Ptr best_match_img;
  if (best_match_message)
  {
    best_match_stamp = best_match_message->header.stamp.toNSec();
    best_match_img = toImageCpu(*best_match_message);
  }
  else
  {
    LOG(WARNING) << "No image found in bag with this timestamp. If this "
                 << "problem is persistent, you may need to re-index the bag: "
                 << "rosrun ze_rosbag_tools bagrestamper.py -i dataset.bag -o dataset_new.bag";
  }
  return std::make_pair(best_match_stamp, best_match_img);
}

}  // namespace ze
