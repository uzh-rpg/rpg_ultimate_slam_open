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
#include <imp/core/image_base.hpp>
#include <ze/common/ringbuffer.hpp>
#include <ze/common/time_conversions.hpp>
#include <ze/common/types.hpp>
#include <ze/data_provider/camera_imu_synchronizer_base.hpp>


namespace ze {

// fwd
class DataProviderBase;
class ImageBase;

// -----------------------------------------------------------------------------
class CameraImuSynchronizer: public CameraImuSynchronizerBase
{
public:
  // convenience typedefs
  using ImuSyncBuffer = Ringbuffer<real_t, 6, 1000>;
  using ImuBufferVector = std::vector<ImuSyncBuffer>;

  //! Default constructor.
  CameraImuSynchronizer(DataProviderBase& data_provider,
                        const bool& use_events_and_images,
                        const bool& use_events);

  //! Add IMU measurement to the frame synchronizer.
  void addImuData(
      int64_t stamp,
      const Vector3& acc,
      const Vector3& gyr,
      const uint32_t imu_idx);

private:
  //! Register callbacks in data provider to this class' addImgData and addImuData.
  void subscribeDataProvider(DataProviderBase& data_provider);

  //! IMU buffer stores all imu measurements, size of imu_count_.
  ImuBufferVector imu_buffers_;

  //! Initialize the image and imu buffers
  void initBuffers();

  //! This function checks if we have all data ready to call the callback.
  //! The callback refers to the actual processing of the data, tracking,
  //! adding to the pipeline, etc.
  virtual void checkImuDataAndCallback();
  void checkImuDataAndEventsCallback();
  void checkImuDataAndImageCallback();
  void checkImuDataAndImageAndEventsCallback();

  //! Fills an array with FLAGS_data_size_augmented_event_packet events
  //! in the given event_buffer that have a timestamp < t1.
  //! It also erases those and the older ones from the event buffer.
  //! The array is timestamped with the value of t1.
  void extractAndEraseEvents(const int64_t& t1,
                             int max_num_events_in_packet,
                             EventBuffer* event_buffer,
                             StampedEventArray* event_array);

  bool use_events_and_images_;
  bool use_events_;
};

} // namespace ze
