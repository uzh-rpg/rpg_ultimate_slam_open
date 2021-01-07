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

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <dvs_msgs/EventArray.h>
#include <ze_ros_msg/bmx055_acc.h>
#include <ze_ros_msg/bmx055_gyr.h>
#include <image_transport/image_transport.h>

#include <ze/data_provider/data_provider_base.hpp>

namespace ze {

class DataProviderRostopic : public DataProviderBase
{
public:
  // optional: imu_topics, required: camera_topics
  DataProviderRostopic(
      const std::map<std::string, size_t>& imu_topics,
      const std::map<std::string, size_t>& camera_topics,
      const std::map<std::string, size_t>& dvs_topics,
      uint32_t polling_rate = 1000u,
      uint32_t img_queue_size = 0u,
      uint32_t imu_queue_size = 0u,
      uint32_t events_queue_size = 0u);

  // optional: accel_topics, gyro_topics, required: camera_topics
  DataProviderRostopic(
      const std::map<std::string, size_t>& accel_topics,
      const std::map<std::string, size_t>& gyro_topics,
      const std::map<std::string, size_t>& camera_topics,
      uint32_t polling_rate = 1000u,
      uint32_t img_queue_size = 100u,
      uint32_t imu_queue_size = 1000u);


  virtual ~DataProviderRostopic() = default;

  virtual bool spinOnce() override;

  virtual bool ok() const override;

  virtual size_t imuCount() const;

  virtual size_t cameraCount() const;

  virtual size_t dvsCount() const;

  void imgCallback(
      const sensor_msgs::ImageConstPtr& m_img,
      uint32_t cam_idx);

  void imuCallback(
      const sensor_msgs::ImuConstPtr& m_imu,
      uint32_t imu_idx);

  void eventsCallback(
      const dvs_msgs::EventArrayConstPtr& m_events,
      uint32_t events_idx);

  void accelCallback(const ze_ros_msg::bmx055_accConstPtr& m_acc,
                     uint32_t imu_idx);
  void gyroCallback(const ze_ros_msg::bmx055_gyrConstPtr& m_gyr,
                    uint32_t imu_idx);

private:
  ros::CallbackQueue queue_;
  ros::NodeHandle nh_;
  image_transport::ImageTransport img_transport_;
  std::vector<image_transport::Subscriber> sub_cams_;
  std::vector<ros::Subscriber> sub_dvs_;
  std::vector<ros::Subscriber> sub_imus_;
  std::vector<ros::Subscriber> sub_accels_;
  std::vector<ros::Subscriber> sub_gyros_;
  ros::Subscriber kill_sub_;
  uint32_t polling_rate_;

  //! Do we operate on split ros messages or the combined imu messages?
  bool uses_split_messages_;
};

} // namespace ze
