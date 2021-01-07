// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Modified: Robotics and Perception Group

#pragma once

#include <map>
#include <string>
#include <memory>
#include <vector>

#include<gflags/gflags.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/Imu.h>
#include <ze_ros_msg/bmx055_acc.h>
#include <ze_ros_msg/bmx055_gyr.h>

#include <ze/data_provider/data_provider_base.hpp>

DECLARE_bool(vio_use_events);
DECLARE_bool(vio_use_events_and_images);

namespace ze {

class DataProviderRosbag : public DataProviderBase
{
public:
  // optional: imu_topics, required: camera_topics
  DataProviderRosbag(
      const std::string& bag_filename,
      const std::map<std::string, size_t>& imu_topics,
      const std::map<std::string, size_t>& camera_topics,
      const std::map<std::string, size_t>& dvs_topics);

  // optional: accel_topics, gyro_topics
//  DataProviderRosbag(
//      const std::string& bag_filename,
//      const std::map<std::string, size_t>& gyro_topics,
//      const std::map<std::string, size_t>& accel_topics,
//      const std::map<std::string, size_t>& camera_topics);

  virtual ~DataProviderRosbag() = default;

  virtual bool spinOnce() override;

  virtual bool ok() const override;

  virtual size_t imuCount() const;

  virtual size_t cameraCount() const;

  virtual size_t dvsCount() const;

  size_t size() const;

private:
  void loadRosbag(const std::string& bag_filename);
  void initBagView(const std::vector<std::string>& topics);

  inline bool cameraSpin(const sensor_msgs::ImageConstPtr m_img,
                         const rosbag::MessageInstance& m);
  inline bool dvsSpin(const dvs_msgs::EventArrayConstPtr m_img,
                         const rosbag::MessageInstance& m);
  inline bool imuSpin(const sensor_msgs::ImuConstPtr m_imu,
                      const rosbag::MessageInstance& m);
  inline bool accelSpin(const ze_ros_msg::bmx055_accConstPtr m_acc,
                        const rosbag::MessageInstance& m);
  inline bool gyroSpin(const ze_ros_msg::bmx055_gyrConstPtr m_gyr,
                       const rosbag::MessageInstance& m);

  std::unique_ptr<rosbag::Bag> bag_;
  std::unique_ptr<rosbag::View> bag_view_;
  rosbag::View::iterator bag_view_it_;
  int n_processed_images_ = 0;
  int n_processed_event_arrays_ = 0;

  // subscribed topics:
  std::map<std::string, size_t> img_topic_camidx_map_; // camera_topic --> camera_id
  std::map<std::string, size_t> dvs_topic_camidx_map_; // dvs_topic --> dvs_id
  std::map<std::string, size_t> imu_topic_imuidx_map_; // imu_topic --> imu_id

  std::map<std::string, size_t> accel_topic_imuidx_map_; // accel_topic --> imu_id
  std::map<std::string, size_t> gyro_topic_imuidx_map_; // gyro_topic --> imu_id

  //! Do we operate on split ros messages or the combined imu messages?
  bool uses_split_messages_;

  int64_t last_imu_stamp_ = -1;
  int64_t last_acc_stamp_ = -1;
  int64_t last_gyr_stamp_ = -1;
};

} // namespace ze
