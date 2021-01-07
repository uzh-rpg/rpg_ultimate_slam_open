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
#include <stdint.h>

#include <ze/data_provider/camera_imu_synchronizer.hpp>

#include <functional>
#include <gflags/gflags.h>

#include <ze/common/logging.hpp>
#include <ze/data_provider/data_provider_base.hpp>

#include <ze/common/egg_timer.hpp>

namespace ze {

CameraImuSynchronizer::CameraImuSynchronizer(DataProviderBase& data_provider,
                                     const bool& use_events_and_images,
                                     const bool& use_events)
  : CameraImuSynchronizerBase(data_provider),
    use_events_and_images_(use_events_and_images),
    use_events_(use_events)
{
  CHECK_GE(FLAGS_data_size_augmented_event_packet, 0) <<
    "data_size_augmented_event_packet should be positive.";
  subscribeDataProvider(data_provider);
  initBuffers();
}

void CameraImuSynchronizer::subscribeDataProvider(DataProviderBase& data_provider)
{
  using namespace std::placeholders;

  if(num_cameras_ == 0u && num_dvs_ == 0u) {
    LOG(ERROR) << "DataProvider must at least expose a single camera topic.";
  }

  if (!use_events_and_images_) {
    if (num_cameras_ > 0u && !use_events_) {
      data_provider.registerCameraCallback(
            std::bind(&CameraImuSynchronizer::addImageData, this, _1, _2, _3));
    }
    else if (num_dvs_ > 0u)
    {
      data_provider.registerCameraCallback(
            std::bind(&CameraImuSynchronizer::addEventsData, this, _1, _2,
                      false));
    }
  } else {
    if (num_dvs_ > 0u && num_cameras_ >0u) {
      data_provider.registerCameraCallback(
            std::bind(&CameraImuSynchronizer::addImageData, this, _1, _2, _3));
      data_provider.registerCameraCallback(
            std::bind(&CameraImuSynchronizer::addEventsData, this, _1, _2,
                      true));
    } else {
      LOG(ERROR) << "No callback register for the data_provider./n"
                 << "The flag use_events_and_images is true, but "
                 << "num_dvs_ or num_cameras_ is not higher than 0";
    }
  }

  if (num_imus_ > 0u)
  {
    data_provider.registerImuCallback(
          std::bind(&CameraImuSynchronizer::addImuData, this, _1, _2, _3, _4));
  }
}

void CameraImuSynchronizer::initBuffers()
{
  image_buffer_.resize(2 * num_cameras_);
  event_buffer_.clear();
  imu_buffers_ = ImuBufferVector(num_imus_);
}

void CameraImuSynchronizer::addImuData(
    int64_t stamp, const Vector3& acc, const Vector3& gyr, const uint32_t imu_idx)
{
  Vector6 acc_gyr;
  acc_gyr.head<3>() = acc;
  acc_gyr.tail<3>() = gyr;
  //stamp -= timeshift_cam_imu_;
  imu_buffers_[imu_idx].insert(stamp, acc_gyr);

  if (imu_callback_)
  {
    imu_callback_(stamp, acc, gyr, imu_idx);
  }

  checkImuDataAndCallback();
}

void CameraImuSynchronizer::checkImuDataAndCallback() {
  if(!use_events_and_images_){
    if (use_events_) {
      checkImuDataAndEventsCallback();
    } else {
      checkImuDataAndImageCallback();
    }
  } else {
    checkImuDataAndImageAndEventsCallback();
  }
}

void CameraImuSynchronizer::checkImuDataAndImageCallback()
{
  if (sync_imgs_ready_to_process_stamp_ < 0)
  {
    return; // Images are not synced yet.
  }

  // always provide imu structures in the callback (empty if no imu present)
  ImuStampsVector imu_timestamps(num_imus_);
  ImuAccGyrVector imu_measurements(num_imus_);

  if (num_imus_ != 0)
  {
    // get oldest / newest stamp for all imu buffers
    std::vector<std::tuple<int64_t, int64_t, bool>> oldest_newest_stamp_vector(num_imus_);
    std::transform(
          imu_buffers_.begin(),
          imu_buffers_.end(),
          oldest_newest_stamp_vector.begin(),
          [](const ImuSyncBuffer& imu_buffer) {
            return imu_buffer.getOldestAndNewestStamp();
          });

    // imu buffers are not consistent with the image buffers
    if (!validateImuBuffers(
          sync_imgs_ready_to_process_stamp_,
          sync_imgs_ready_to_process_stamp_,
          oldest_newest_stamp_vector))
    {
      return;
    }

    // If this is the very first image bundle, we send all IMU messages that we have
    // received so far. For every later image bundle, we just send the IMU messages
    // that we have received in between.
    for (size_t i = 0; i < num_imus_; ++i)
    {
      if(last_img_bundle_min_stamp_ < 0)
      {
        int64_t oldest_stamp = std::get<0>(oldest_newest_stamp_vector[i]);
        std::tie(imu_timestamps[i], imu_measurements[i]) =
            imu_buffers_[i].getBetweenValuesInterpolated(
              oldest_stamp,
              sync_imgs_ready_to_process_stamp_);
      }
      else
      {
        std::tie(imu_timestamps[i], imu_measurements[i]) =
            imu_buffers_[i].getBetweenValuesInterpolated(
              last_img_bundle_min_stamp_,
              sync_imgs_ready_to_process_stamp_);
      }
    }
  }

  // Let's process the callback.
  image_imu_callback_(sync_images_ready_to_process_, imu_timestamps, imu_measurements);

  // Reset Buffer:
  for (size_t i = 0; i <= image_buffer_.size(); ++i)
  {
    ImageBufferItem& item = image_buffer_[i];
    if (std::abs(sync_imgs_ready_to_process_stamp_ - item.stamp)
        < c_camera_bundle_time_accuracy_ns)
    {
      item.reset();
    }
  }
  last_img_bundle_min_stamp_ = sync_imgs_ready_to_process_stamp_;
  sync_imgs_ready_to_process_stamp_ = -1;
  sync_images_ready_to_process_.clear();
}

void CameraImuSynchronizer::checkImuDataAndEventsCallback()
{
  if (event_packages_ready_to_process_.empty())
  {
    return; // No event package to process yet
  }

  std::vector<bool> discard_event_packet(event_packages_ready_to_process_.size(), false);

  for(size_t i = 0; i < event_packages_ready_to_process_.size(); ++i)
  {
    const StampedEventArray& event_package = event_packages_ready_to_process_[i];
    const int64_t event_package_stamp = event_package.first;

    VLOG(1000) << "Current event package stamp:" << event_package_stamp;

    // always provide imu structures in the callback (empty if no imu present)
    ImuStampsVector imu_timestamps(num_imus_);
    ImuAccGyrVector imu_measurements(num_imus_);

    CHECK_GT(num_imus_, 0u) << "Camera IMU synchronizer with no IMU is not implemented";

    // get oldest / newest stamp for all imu buffers
    std::vector<std::tuple<int64_t, int64_t, bool>> oldest_newest_stamp_vector(num_imus_);
    std::transform(
          imu_buffers_.begin(),
          imu_buffers_.end(),
          oldest_newest_stamp_vector.begin(),
          [](const ImuSyncBuffer& imu_buffer) {
      return imu_buffer.getOldestAndNewestStamp();
    });

    const int64_t oldest_imu_stamp = std::get<0>(oldest_newest_stamp_vector[0]);
    const int64_t newest_imu_stamp = std::get<1>(oldest_newest_stamp_vector[0]);
    VLOG(1000) << "Oldest IMU stamp: " << oldest_imu_stamp;
    VLOG(1000) << "Newest IMU stamp: " << newest_imu_stamp;

    // imu buffers are not consistent with the image buffers
    if (!validateImuBuffers(
          event_package_stamp,
          event_package_stamp,
          oldest_newest_stamp_vector))
    {
      if(oldest_imu_stamp >= event_package_stamp)
      {
        // Oldest IMU measurement is newer than image timestamp
        // This will happen only at the very beginning, thus
        // it is safe to simply discard the event package
        VLOG(1000) << "Discarding event packet at stamp:" << event_package_stamp;
        discard_event_packet[i] = true;
      }
    }
    else
    {
      // We have enough IMU measurements to broadcast the current packet
      VLOG(1000) << "last_event_package_broadcast_stamp: " << last_event_package_broadcast_stamp_;

      // If this is the very first event package, we send all IMU messages that we have
      // received so far. For every later event package, we just send the IMU messages
      // that we have received in between.
      for (size_t i = 0; i < num_imus_; ++i)
      {
        if(last_event_package_broadcast_stamp_ < 0)
        {
          int64_t oldest_stamp = std::get<0>(oldest_newest_stamp_vector[i]);
          std::tie(imu_timestamps[i], imu_measurements[i]) =
              imu_buffers_[i].getBetweenValuesInterpolated(
                oldest_stamp,
                event_package_stamp);
        }
        else
        {
          std::tie(imu_timestamps[i], imu_measurements[i]) =
              imu_buffers_[i].getBetweenValuesInterpolated(
                //event_package.second->at(0).ts.toNSec(), <-- Should be this...
                last_event_package_broadcast_stamp_,
                event_package_stamp);
        }
      }

      // Let's process the callback
      events_imu_callback_(event_package, imu_timestamps, imu_measurements);

      // Discard the event package, now that it's been processed
      discard_event_packet[i] = true;

      last_event_package_broadcast_stamp_ = event_package_stamp;
    }
  }

  // Discard all the event packages that have been marked
  StampedEventArrays event_packages_ready_to_process_new;
  for(size_t i=0; i<event_packages_ready_to_process_.size(); ++i)
  {
    if(!discard_event_packet[i])
    {
      event_packages_ready_to_process_new.push_back(event_packages_ready_to_process_[i]);
    }
  }
  event_packages_ready_to_process_ = event_packages_ready_to_process_new;
}

void CameraImuSynchronizer::checkImuDataAndImageAndEventsCallback()
{
  if (sync_imgs_ready_to_process_stamp_ < 0)
  {
    return; // Images are not synced yet.
  }

  // always provide imu structures in the callback (empty if no imu present)
  ImuStampsVector imu_timestamps(num_imus_);
  ImuAccGyrVector imu_measurements(num_imus_);

  // Process IMU data. We should only do that if we come from addImuData...
  if (num_imus_ != 0)
  {
    // get oldest / newest stamp for all imu buffers
    std::vector<std::tuple<int64_t, int64_t, bool>> oldest_newest_stamp_vector(num_imus_);
    std::transform(
          imu_buffers_.begin(),
          imu_buffers_.end(),
          oldest_newest_stamp_vector.begin(),
          [](const ImuSyncBuffer& imu_buffer) {
            return imu_buffer.getOldestAndNewestStamp();
          });

    // imu buffers are not consistent with the image buffers
    // or the imu buffer is not filled until the image stamp, return.
    // This ensures we collect all IMU data before timestamp of the image.
    if (!validateImuBuffers(
          sync_imgs_ready_to_process_stamp_,
          sync_imgs_ready_to_process_stamp_,
          oldest_newest_stamp_vector))
    {
      return;
    }

    // If this is the very first image bundle, we send all IMU messages that we have
    // received so far. For every later image bundle, we just send the IMU messages
    // that we have received in between.
    for (size_t i = 0; i < num_imus_; ++i)
    {
      if(last_img_bundle_min_stamp_ < 0)
      {
        int64_t oldest_stamp = std::get<0>(oldest_newest_stamp_vector[i]);
        std::tie(imu_timestamps[i], imu_measurements[i]) =
            imu_buffers_[i].getBetweenValuesInterpolated(
              oldest_stamp,
              sync_imgs_ready_to_process_stamp_);
      }
      else
      {
        std::tie(imu_timestamps[i], imu_measurements[i]) =
            imu_buffers_[i].getBetweenValuesInterpolated(
              last_img_bundle_min_stamp_,
              sync_imgs_ready_to_process_stamp_);
      }
    }
  }

  // We should do this only when we come from the addEventsData...
  // Check that we have all events until the timestamp given by the image.
  // If not return, and wait until the event buffer is filled until this
  // timestamp.
  // This timeout might not be strictly necessary, as at most, we will check
  // for events after this image timestamp until we receive a new frame.
  StampedEventArray event_array;
  static constexpr uint64_t kCollectEventsTimeoutNs = 100000u;
  static ze::EggTimer timer (kCollectEventsTimeoutNs);
  static bool start_timer = true;
  const int64_t& last_event_timestamp =
      static_cast<int64_t>(event_buffer_.back().ts.toNSec());
  if(sync_imgs_ready_to_process_stamp_ > last_event_timestamp) {
    if (start_timer) {
      start_timer = false;
      timer.reset();
    }

    if (!timer.finished()) {
      VLOG(2) << "Trying to collect events until image timestamp.\n"
              << "Current image timestamp is: "
              << sync_imgs_ready_to_process_stamp_ << '\n'
              << "Last event timestamp is: "
              << static_cast<int64_t>(event_buffer_.back().ts.toNSec()) << '\n'
              << "Image timestamp - Last event timestamp = "
              << sync_imgs_ready_to_process_stamp_ - last_event_timestamp;
      return;
    } else {
      VLOG(2) << "Collect Events Timeout reached: "
              << "processing current packet.";
      start_timer = true;
    }
  } else {
    start_timer = true;
  }

  // Get Events data between frames, similar to IMU sync above.
  int64_t t1 = sync_imgs_ready_to_process_stamp_;
  extractAndEraseEvents(t1, FLAGS_data_size_augmented_event_packet,
                        &event_buffer_, &event_array);

  // Let's process the callback.
  //events_imu_callback_(event_array,
   //                          imu_timestamps, imu_measurements);
  image_events_imu_callback_(sync_images_ready_to_process_, event_array,
                             imu_timestamps, imu_measurements,
                             no_motion_prior_for_backend_);

  // Reset Buffer:
  for (size_t i = 0; i <= image_buffer_.size(); ++i)
  {
    ImageBufferItem& item = image_buffer_[i];
    if (std::abs(sync_imgs_ready_to_process_stamp_ - item.stamp)
        < c_camera_bundle_time_accuracy_ns)
    {
      item.reset();
    }
  }
  last_img_bundle_min_stamp_ = sync_imgs_ready_to_process_stamp_;
  sync_imgs_ready_to_process_stamp_ = -1;
  sync_images_ready_to_process_.clear();
}

void CameraImuSynchronizer::extractAndEraseEvents(
                                const int64_t& t1,
                                int max_num_events_in_packet,
                                EventBuffer* event_buffer,
                                StampedEventArray* event_array)
{
  CHECK_NOTNULL(event_buffer);
  CHECK_NOTNULL(event_array);

  event_array->second = std::make_shared<EventArray>();

  // Check that the event buffer is not empty.
  if (event_buffer->empty()) {
    LOG(WARNING) << "Event buffer is empty.";
    // Fill with empty events.
    event_array->first = t1;
    CHECK(event_array->second->empty()) << "Event array vector should be empty";
    return;
  }

  // Fill the event array with a fixed number of events.
  event_array->second->resize((uint32_t)max_num_events_in_packet);
  size_t num_stored_events = 0;
  for (int i = event_buffer->size() - 1; i >= 0; i--) {
    const dvs_msgs::Event& event (event_buffer->at(i));
    const int64_t& cur_stamp (event.ts.toNSec());
    // Don't use the most recent events as they will be used in the next call.
    // Can be improved by doing a binary search instead...
    if (cur_stamp > t1) {
      continue;
    }
    // Store all events that come before t1 until we reach the limit of events
    // in array or there are no more events in buffer.
    if (num_stored_events < (uint32_t)max_num_events_in_packet && i != 0) {
      // Store them in order, first the oldest event, last the most recent.
      // So no push back.
      event_array->second->at((uint32_t)max_num_events_in_packet - 1 - num_stored_events) =
          event;
      num_stored_events++;
    } else {
      // If we got here because there are no more events in the buffer, but
      // we did not reach the maximum size of events in the event array,
      // add the last event and skip.
      if (num_stored_events == (uint32_t)max_num_events_in_packet) {
        if (i == 0) {
          // Do nothing, buffer empty, and event_array full.
          break;
        } else {
          // Buffer still with old values, event_array full.
          // Clean old values.
          event_buffer->erase(event_buffer->begin(), event_buffer->begin() + i);
        }
      } else {
        // Here i has to be equal to 0.
        // Therefore, the buffer is empty, but we did not completely
        // fill the event_array.
        event_array->second->at((uint32_t)max_num_events_in_packet - 1 - num_stored_events) =
            event;
        num_stored_events++;
        // Clear non-used space.
        event_array->second->erase(event_array->second->begin(),
                                    event_array->second->end()
                                    - num_stored_events);
      }
      break;
    }
  }

  // Use the last timestamp as stamp for the event_array.
  event_array->first = t1;
 // So there is this subtle mistake that we are not actually takin all events
 // that could have been generated before t1 in this code... Just the events
 // before t1 that were in the buffer when this function is called...
 // LOG(WARNING) << "Difference between requested timestamp t1 and actual last "
 //              << "timestamp of the event package: "
 //              << t1 - event_array->second->back().ts.toNSec();
}

} // namespace ze
