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

#include <memory>
#include <deque>
#include <imp/core/image_base.hpp>
#include <ze/common/types.hpp>
#include <ze/common/time_conversions.hpp>
#include <ze/imu/imu_buffer.hpp>
#include <ze/data_provider/data_provider_base.hpp>

DECLARE_int32(data_size_augmented_event_packet);

namespace ze {

class DataProviderBase;

// convenience typedefs
using ImuStampsVector = std::vector<ImuStamps>;
using ImuAccGyrVector = std::vector<ImuAccGyrContainer>;

// callback typedefs
using SynchronizedImageImuCallback =
  std::function<void (const StampedImages&    /*image*/,
                      const ImuStampsVector& /*imu_timestamps*/,
                      const ImuAccGyrVector& /*imu_measurements*/)>;
using SynchronizedEventsImuCallback =
  std::function<void (const StampedEventArray& /*event_arrays*/,
                      const ImuStampsVector& /*imu_timestamps*/,
                      const ImuAccGyrVector& /*imu_measurements*/)>;
using SynchronizedImageEventsImuCallback =
  std::function<void (const StampedImages&    /*image*/,
                      const StampedEventArray& /*event_arrays*/,
                      const ImuStampsVector& /*imu_timestamps*/,
                      const ImuAccGyrVector& /*imu_measurements*/,
                      const bool& no_motion_prior)>;
// -----------------------------------------------------------------------------
//! Container to buffer received images.
struct ImageBufferItem
{
  int64_t stamp    		    { -1 };
  ImageBasePtr img     { nullptr };
  int32_t camera_idx			{ -1 };
  inline bool empty()
  {
    return stamp == -1;
  }

  inline void reset()
  {
    stamp = -1;
    img.reset();
    camera_idx = -1;
  }
};
using ImageBuffer = std::vector<ImageBufferItem>;

//! Container to buffer received event arrays.
struct EventsBufferItem
{
  int64_t stamp        { -1 };
  EventArrayPtr events     { nullptr };
  inline bool empty()
  {
    return stamp == -1;
  }

  inline void reset()
  {
    stamp = -1;
    events.reset();
  }
};
using EventsBuffer = std::vector<EventsBufferItem>;

// Forward class.
class EventFrameGenerator;

class CameraImuSynchronizerBase
{
public:
  // Base class needs virtual destructor
  virtual ~CameraImuSynchronizerBase() = default;

  // Convenience typedefs
  using EventBuffer = std::deque<dvs_msgs::Event>;

  void registerCameraImuCallback(const SynchronizedImageImuCallback& callback);
  void registerCameraImuCallback(const SynchronizedEventsImuCallback& callback);
  void registerCameraImuCallback(const SynchronizedImageEventsImuCallback& callback);

  //! Register callback function to call when new IMU message is available.
  void registerImuCallback(const ImuCallback& imu_callback)
  {
    imu_callback_ = imu_callback;
  }

  //! Add Image to the frame synchronizer.
  void addImageData(
      int64_t stamp,
      const ImageBasePtr& image,
      uint32_t camera_idx);
  //! Add Event array to the frame synchronizer.
  void addEventsData(
      int64_t stamp,
      const EventArrayPtr& events,
      const bool& use_events_and_images);

  //! Workflow when only using events, no images.
  void onlyEventsNoImagesLogic();

protected:
  //! Allowed time differences of images in bundle.
   static constexpr real_t c_camera_bundle_time_accuracy_ns = millisecToNanosec(2.0);

   //! Stamp of previous synchronized image bundle.
   int64_t last_img_bundle_min_stamp_ { -1 };

  //! Default constructor.
  CameraImuSynchronizerBase(DataProviderBase& data_provider);

  //! Stamp of previous synchronized image bundle.
  int64_t last_event_package_broadcast_stamp_ { -1 };

  //! Num IMUs to synchronize.
  uint32_t num_imus_;

  //! Num images to synchronize
  uint32_t num_cameras_;

  //! Num event cameras to synchronize.
  uint32_t num_dvs_;

  ////// @todo: missing members compared to image based pipeline.
  /// Such as sync_imgs_ready_to_process_stamp_, last_img_bundle_min_stamp_
  StampedImages sync_images_ready_to_process_;
  int64_t sync_imgs_ready_to_process_stamp_ { -1 };
  StampedEventArrays event_packages_ready_to_process_;
  int64_t last_package_stamp_;
  size_t last_package_ev_;

  int64_t timeshift_cam_imu_;

  //! Image buffer is buffering images of max the last 2*rig_size images.
  ImageBuffer image_buffer_;
  //! Event buffer stores all the events, size of event_count_.
  EventBuffer event_buffer_;
  size_t cur_ev_; // index of the last event processed yet
  ros::Time end_stamp_last_package_;

  //! This function checks if we have all data ready to call the callback.
  virtual void checkImuDataAndCallback() = 0;

  //! Validate the contents of the IMU buffer relative to the image buffers.
  bool validateImuBuffers(
      const int64_t& min_stamp,
      const int64_t& max_stamp,
      const std::vector<std::tuple<int64_t, int64_t, bool>>& oldest_newest_stamp_vector);

  //! Max time difference of images in a bundle
  int64_t img_bundle_max_dt_nsec_ = millisecToNanosec(2.0);

  //! Count number of synchronized frames.
  int sync_frame_count_  { 0 };

  //! Count number of synchronized event arrays.
  int sync_event_arrays_count_  { 0 };

  //! Registered callback for synchronized measurements.
  SynchronizedImageImuCallback image_imu_callback_;

  //! Registered callback for synchronized measurements.
  SynchronizedEventsImuCallback events_imu_callback_;

  //! Registered callback for synchronized measurements.
  SynchronizedImageEventsImuCallback image_events_imu_callback_;

  //! Registered callback for raw IMU measurements.
  ImuCallback imu_callback_;

  bool no_motion_prior_for_backend_;
};

} // namespace ze
