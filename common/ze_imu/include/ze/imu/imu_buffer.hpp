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

#pragma once

#include <mutex>

#include <ze/imu/imu_model.hpp>
#include <ze/common/ringbuffer.hpp>

namespace ze {

struct InterpolatorDifferentiatorLinear
{
  typedef Vector6 return_t;

  //! The name could be more descriptive, but the current naming allows for
  //! using the interpolators defined in ringbuffer.h
  template<typename Ringbuffer_T>
  static return_t interpolate(
      Ringbuffer_T* buffer,
      int64_t time,
      const typename Ringbuffer_T::timering_t::iterator it_before)
  {
    // the end value
    const auto it_after = it_before + 1;
    if (it_after == buffer->times().end())
    {
      LOG(WARNING) << "Interpolation hit end of buffer.";
      return (return_t() << buffer->data().col(it_before.container_index()),
          Vector3::Zero()).finished();
    }

    const real_t offset = static_cast<real_t>(time - *it_before);
    const real_t duration = static_cast<real_t>(*it_after - *it_before);
    const Vector3 before = buffer->data().col(it_before.container_index());
    const Vector3 after = buffer->data().col(it_after.container_index());

    return (return_t() << before + (after - before) * offset / duration,
        (after - before) / duration).finished();
  }

  template<typename Ringbuffer_T>
  static return_t interpolate(
      Ringbuffer_T* buffer,
      int64_t time)
  {
    const auto it_before = buffer->iterator_equal_or_before(time);
    // caller should check the bounds:
    CHECK(it_before != buffer->times().end());

    return interpolate(buffer, time, it_before);
  }
};
using DefaultInterpolator = InterpolatorLinear;

//! An IMU Buffer with an underlying Gyro and Accel model that also corrects
//! measurement timestamps. The timestamps are corrected when inserted into the
//! buffers.
template<int BufferSize, typename GyroInterp,
typename AccelInterp = GyroInterp>
class ImuBuffer
{
public:
  ZE_POINTER_TYPEDEFS(ImuBuffer);

  ImuBuffer(ImuModel::Ptr imu_model);

  void insertGyroscopeMeasurement(time_t stamp, const Vector3);
  void insertAccelerometerMeasurement(time_t stamp, const Vector3);

  //! Insert an IMU measurement at a given timestamp: First three values refer
  //! to the accelerometer, last 3 the gyroscope.
  void insertImuMeasurement(int64_t time, const ImuAccGyr value);

  //! Get the rectified values of the IMU at a given timestamp. Interpolates
  //! if necessary.
  //! Return flag if successful
  bool get(int64_t time, Eigen::Ref<ImuAccGyr> out);

  //! Get all values between two timestamps, synchronize Accelerometer and
  //! Gyroscope, interpolate edges to fit start and end. Interpolates Gyro
  //! and Accel measurements to have equal timestamps. Rectify all measurements.
  std::pair<ImuStamps, ImuAccGyrContainer>
  getBetweenValuesInterpolated(int64_t stamp_from, int64_t stamp_to);

  //! Get the oldest and newest timestamps for which both Accelerometers
  //! and Gyroscopes have measurements.
  std::tuple<int64_t, int64_t, bool> getOldestAndNewestStamp() const;

  //! Get the delay corrected timestamps (Delays are negative if in the past).
  inline int64_t correctStampGyro(int64_t t)
  {
    return t + gyro_delay_;
  }
  inline int64_t correctStampAccel(int64_t t)
  {
     return t + accel_delay_;
  }

protected:
  bool getAccelerometerDistorted(int64_t time, Eigen::Ref<Vector3> out);
  bool getGyroscopeDistorted(int64_t time, Eigen::Ref<Vector3> out);

private:
  //! The underlying storage structures for accelerometer and gyroscope
  //! measurements.
  Ringbuffer<real_t, 3, BufferSize> acc_buffer_;
  Ringbuffer<real_t, 3, BufferSize> gyr_buffer_;

  ImuModel::Ptr imu_model_;

  //! Store the accelerometer and gyroscope delays in nanoseconds
  int64_t gyro_delay_;
  int64_t accel_delay_;
};

// A set of explicit declarations
typedef ImuBuffer<2000, InterpolatorLinear> ImuBufferLinear2000;
typedef ImuBuffer<5000, InterpolatorLinear> ImuBufferLinear5000;
typedef ImuBuffer<2000, InterpolatorNearest> ImuBufferNearest2000;
typedef ImuBuffer<5000, InterpolatorNearest> ImuBufferNearest5000;
typedef ImuBuffer<2000, InterpolatorDifferentiatorLinear, InterpolatorLinear>
ImuBufferDiff2000;
typedef ImuBuffer<5000, InterpolatorDifferentiatorLinear, InterpolatorLinear>
ImuBufferDiff5000;

} // namespace ze
