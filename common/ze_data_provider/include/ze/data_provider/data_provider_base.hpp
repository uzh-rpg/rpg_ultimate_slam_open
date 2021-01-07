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

#include <atomic>
#include <functional>
#include <memory>

#include <ze/common/macros.hpp>
#include <ze/common/noncopyable.hpp>
#include <ze/common/signal_handler.hpp>
#include <ze/common/types.hpp>

// fwd
namespace cv {
class Mat;
}

namespace ze {

// fwd
class ImageBase;

using ImuCallback =
  std::function<void (int64_t /*timestamp*/,
                      const Vector3& /*acc*/,
                      const Vector3& /*gyr*/,
                      uint32_t /*imu-idx*/)>;

using GyroCallback =
  std::function<void (int64_t /*timestamp*/,
                      const Vector3& /*gyr*/,
                      uint32_t /*imu-idx*/)>;
using AccelCallback =
  std::function<void (int64_t /*timestamp*/,
                      const Vector3& /*acc*/,
                      uint32_t /*imu-idx*/)>;

using CameraCallback =
  std::function<void (int64_t /*timestamp*/,
                      const std::shared_ptr<ImageBase>& /*img*/,
                      uint32_t /*camera-idx*/)>;

using DVSCallback =
  std::function<void (int64_t /*timestamp*/,
                      const std::shared_ptr<EventArray>& /*events*/,
                      uint32_t /*dvs-idx*/)>;

enum class DataProviderType {
  Csv,
  Rosbag,
  Rostopic
};

//! A data provider registers to a data source and triggers callbacks when
//! new data is available.
class DataProviderBase : Noncopyable
{
public:
  ZE_POINTER_TYPEDEFS(DataProviderBase);

  DataProviderBase() = delete;
  DataProviderBase(DataProviderType type);
  virtual ~DataProviderBase() = default;

  //! Process all callbacks. Waits until callback is processed.
  void spin();

  //! Read next data field and process callback. Returns false when datatset finished.
  virtual bool spinOnce() = 0;

  //! False if there is no more data to process or there was a shutdown signal.
  virtual bool ok() const = 0;

  //! Pause data provider.
  virtual void pause();

  //! Stop data provider.
  virtual void shutdown();

  //! Number of imus to process.
  virtual size_t imuCount() const = 0;

  //! Number of cameras to process.
  virtual size_t cameraCount() const = 0;

  //! Number of event cameras to process.
  virtual size_t dvsCount() const = 0;

  //! Register callback function to call when new IMU message is available.
  void registerImuCallback(const ImuCallback& imu_callback);

  //! Register callback function to call when new camera message is available.
  void registerCameraCallback(const CameraCallback& camera_callback);

  //! Register callback function to call when new event message is available.
  void registerCameraCallback(const DVSCallback& dvs_callback);

  //! Register callback function to call when new Gyroscope message is available.
  void registerGyroCallback(const GyroCallback& gyro_callback);

  //! Register callback function to call when new Accelerometer message is available.
  void registerAccelCallback(const AccelCallback& accel_callback);

protected:
  DataProviderType type_;
  ImuCallback imu_callback_;
  CameraCallback camera_callback_;
  DVSCallback dvs_callback_;
  GyroCallback gyro_callback_;
  AccelCallback accel_callback_;
  volatile bool running_ = true;

private:
  SimpleSigtermHandler signal_handler_; //!< Sets running_ to false when Ctrl-C is pressed.
};

} // namespace ze
