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

#include <ze/data_provider/camera_imu_synchronizer_base.hpp>

#include <ze/data_provider/data_provider_base.hpp>
#include <ze/data_provider/data_provider_factory.hpp>
#include <ze/common/time_conversions.hpp>

DEFINE_int32(data_sync_init_skip_n_frames, 0,
             "How many frames should be skipped at the beginning.");

DEFINE_int32(data_sync_stop_after_n_frames, -1,
             "How many frames should be processed?");

DEFINE_bool(data_use_time_interval, false,
            "Specify whether events packets are created at a fixed \"time\""
            " rate (true) or fixed event rate (false). Only used for the events"
            " only pipeline.");

DEFINE_int32(data_interval_between_event_packets, 10000,
             "What is the interval between two event packets? "
             "Specified either in milliseconds if data_use_time_interval = true,"
             " or number of events if false. Only used for the events only"
             " pipeline.");

DEFINE_int32(data_size_augmented_event_packet, 100,
             "What is the size of each augmented event packet,"
             " in number of events. Only used for the events only pipeline.");

namespace ze {

CameraImuSynchronizerBase::CameraImuSynchronizerBase(
                    DataProviderBase& data_provider)
  : num_imus_(data_provider.imuCount())
  , num_cameras_(data_provider.cameraCount())
  , num_dvs_(data_provider.dvsCount())
  , timeshift_cam_imu_(secToNanosec(FLAGS_timeshift_cam_imu))
{
  cur_ev_ = 0;
  last_package_stamp_ = -1;
  last_package_ev_ = 0;

  LOG(INFO) << "t_cam = t_imu - " << timeshift_cam_imu_ << " [ns]";
}

void CameraImuSynchronizerBase::registerCameraImuCallback(
    const SynchronizedImageImuCallback& callback)
{
  image_imu_callback_ = callback;
}

void CameraImuSynchronizerBase::registerCameraImuCallback(
    const SynchronizedEventsImuCallback& callback)
{
  events_imu_callback_ = callback;
}

void CameraImuSynchronizerBase::registerCameraImuCallback(
    const SynchronizedImageEventsImuCallback& callback)
{
  image_events_imu_callback_ = callback;
}

void CameraImuSynchronizerBase::addImageData(
    int64_t stamp, const ImageBase::Ptr& img, uint32_t camera_idx)
{
  CHECK_LT(camera_idx, num_cameras_);

  // Correct for timestamp delay between IMU and Frames.
  stamp += timeshift_cam_imu_;

  // Skip frame processing for first N frames.
  if (sync_frame_count_ < FLAGS_data_sync_init_skip_n_frames)
  {
    if (camera_idx == 0)
    {
      ++sync_frame_count_;
    }
    return;
  }

  // Add image to first available slot in our buffer:
  int slot = -1;
  for (size_t i = 0u; i < image_buffer_.size(); ++i)
  {
    if (image_buffer_[i].empty())
    {
      slot = i;
      break;
    }
  }

  if (slot == -1)
  {
    // No space in buffer to process frame. Delete oldest one. This happens
    // also when the processing is not fast enough such that frames are skipped.
    int64_t min_stamp = std::numeric_limits<int64_t>::max();
    for (size_t i = 0u; i < image_buffer_.size(); ++i)
    {
      if (!image_buffer_[i].empty() && image_buffer_[i].stamp < min_stamp)
      {
        slot = i;
        min_stamp = image_buffer_[i].stamp;
      }
    }
  }

  image_buffer_[slot].stamp = stamp;
  image_buffer_[slot].img = img;
  image_buffer_[slot].camera_idx = camera_idx;

  // Now check, if we have all images from this bundle:
  uint32_t num_imgs = 0u;
  for (size_t i = 0; i <= image_buffer_.size(); ++i)
  {
    if (std::abs(stamp - image_buffer_[i].stamp) < millisecToNanosec(2))
    {
      ++num_imgs;
    }
  }

  if (num_imgs != num_cameras_)
  {
    return; // We don't have all frames yet.
  }

  // We have frames with very close timestamps. Put them together in a vector.
  sync_images_ready_to_process_.clear();
  sync_images_ready_to_process_.resize(num_imgs, {-1, nullptr});
  for (size_t i = 0; i <= image_buffer_.size(); ++i)
  {
    ImageBufferItem& item = image_buffer_[i];
    if (std::abs(stamp - item.stamp) < c_camera_bundle_time_accuracy_ns)
    {
      DEBUG_CHECK_GT(item.stamp, 0);
      DEBUG_CHECK(item.img);
      sync_images_ready_to_process_.at(item.camera_idx) = { item.stamp, item.img };
    }
  }

  // Double-check that we have all images.
  for (size_t i = 0; i < sync_images_ready_to_process_.size(); ++i)
  {
    if (sync_images_ready_to_process_.at(i).first == -1)
    {
      LOG(ERROR) << "Sync images failed!";
      sync_images_ready_to_process_.clear();
      return;
    }
  }
  sync_imgs_ready_to_process_stamp_ = sync_images_ready_to_process_.front().first;

  checkImuDataAndCallback();
}

void CameraImuSynchronizerBase::onlyEventsNoImagesLogic() {
  // 1. Split the event queue in packages of equal temporal size / equal number of events
  // 2. Augment each event package with additional events in the past
  // (so that the front-end can choose how many events it needs within the package)
  // 3. Add the augmented event packages to the processing queue
  //    (they will be broadcast as soon as enough IMU measurements are available)

  int64_t interval_between_packets;
  if(FLAGS_data_use_time_interval)
    interval_between_packets = ros::Duration(FLAGS_data_interval_between_event_packets/1000.0).toNSec();
  else
    interval_between_packets = FLAGS_data_interval_between_event_packets;

  const size_t package_size = FLAGS_data_size_augmented_event_packet;

  for(; cur_ev_ < event_buffer_.size(); cur_ev_++)
  {
    const dvs_msgs::Event& e = event_buffer_[cur_ev_];
    const int64_t cur_stamp = e.ts.toNSec();

    bool create_packet = (!FLAGS_data_use_time_interval &&
                          (cur_ev_ - last_package_ev_ >= (size_t) interval_between_packets)) ||
        (FLAGS_data_use_time_interval &&
         (cur_stamp - last_package_stamp_ > interval_between_packets));

    if(create_packet)
    {
      const int64_t package_stamp = cur_stamp;

      VLOG(100) << "Creating event package from " << last_package_stamp_
                << " to " << cur_stamp
                << " with stamp: " << package_stamp;

      // Create the augmented event package
      /// That looks more like it is reducing the event package...
      /// Instead of taking all events it only takes as much events as package
      /// size, if there are less then just takes all of them.
      ///
      /// This is always true except at the beginning: this is because the
      /// buffer size is huge, and gets only flushed when it is full and just by
      /// the amount that it is overflowing, therefore cur_ev_ will be always
      /// a huge value, hence cur_ev_ > package_size always when the buffer is
      /// bigger than "package_size" events...
      /// So we are effectively always taking the last "package_size" number of
      ///  events in the event_array_ptr... No matter what...
      size_t ev = cur_ev_ > package_size ?
                    cur_ev_ - package_size : 0;

      // Build the actual event package
      VLOG(100) << "Augmented event package: " << event_buffer_[ev].ts.toNSec()
                << " to " << cur_stamp
                << " with " << cur_ev_ - ev + 1 << " events";

      EventArrayPtr event_array_ptr = std::make_shared<EventArray>();
      for(; ev <= cur_ev_ ; ev++)
        event_array_ptr->push_back(event_buffer_[ev]);

      event_packages_ready_to_process_.push_back(StampedEventArray(package_stamp, event_array_ptr));

      // Prepare to build a new event package
      last_package_stamp_ = cur_stamp;
      last_package_ev_ = cur_ev_;
    }
  }
}

void CameraImuSynchronizerBase::addEventsData(
    int64_t /*stamp*/, const std::shared_ptr<EventArray>& events,
    const bool& use_events_and_images)
{
  // Use the IMU event frame generation.
  event_buffer_.insert(event_buffer_.end(), events->begin(), events->end());

  if (!use_events_and_images) {
    // Use the pipeline meant for only events and no images

    onlyEventsNoImagesLogic();
    checkImuDataAndCallback();

    // Clear events from the DVS buffer
    static size_t event_history_size_ = 500000;

    if (cur_ev_ > event_history_size_)
    {
      size_t remove_events = cur_ev_ - event_history_size_;

      event_buffer_.erase(event_buffer_.begin(), event_buffer_.begin()
                          + remove_events);
      cur_ev_ -= remove_events;
      last_package_ev_ -= remove_events;

      VLOG(10) << "Removed " << remove_events << " events from the DVS buffer";
    }

    VLOG(1000) << "num events added to the queue: " << events->size();
    VLOG(1000) << "total num events: " << event_buffer_.size();
  } else {
    checkImuDataAndCallback();
  }
}

bool CameraImuSynchronizerBase::validateImuBuffers(
    const int64_t& min_stamp,
    const int64_t& max_stamp,
    const std::vector<std::tuple<int64_t, int64_t, bool> >&
    oldest_newest_stamp_vector)
{
  // Check if we have received some IMU measurements for at least one of the imu's.
  if (std::none_of(oldest_newest_stamp_vector.begin(),
                   oldest_newest_stamp_vector.end(),
                   [](const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp)
  {
                   if (std::get<2>(oldest_newest_stamp))
  {
                   return true;
}
                   return false;
})
      )
  {
    LOG(WARNING) << "Received all images but no imu measurements!";
    return false;
  }

  // At least one IMU measurements before image
  if (std::none_of(oldest_newest_stamp_vector.begin(),
                   oldest_newest_stamp_vector.end(),
                   [&min_stamp](const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp)
  {
                   if (std::get<0>(oldest_newest_stamp) < min_stamp) {
                   return true;
}
                   return false;
})
      )
  {
    LOG(WARNING) << "Oldest IMU measurement is newer than image timestamp.";
    return false;
  }

  // At least one IMU measurements after image
  if (std::none_of(oldest_newest_stamp_vector.begin(),
                   oldest_newest_stamp_vector.end(),
                   [&max_stamp](const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp) {
                   if (std::get<1>(oldest_newest_stamp) > max_stamp)
  {
                   return true;
}
                   return false;
})
      )
  {
    VLOG(100) << "Waiting for IMU measurements.";
    return false;
  }

  return true;
}

} // namespace ze
