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

#include <ze/imu/imu_buffer.hpp>

namespace ze {

template<int BufferSize, typename GyroInterp, typename AccelInterp>
ImuBuffer<BufferSize, GyroInterp, AccelInterp>::ImuBuffer(ImuModel::Ptr imu_model)
  : imu_model_(imu_model)
  , gyro_delay_(secToNanosec(imu_model->gyroscopeModel()->intrinsicModel()->delay()))
  , accel_delay_(secToNanosec(imu_model->accelerometerModel()->intrinsicModel()->delay()))
{
}

template<int BufferSize, typename GyroInterp, typename AccelInterp>
void ImuBuffer<BufferSize, GyroInterp, AccelInterp>::insertImuMeasurement(
    int64_t time, const ImuAccGyr value)
{
  acc_buffer_.insert(correctStampAccel(time), value.head<3>(3));
  gyr_buffer_.insert(correctStampGyro(time), value.tail<3>(3));
}


template<int BufferSize, typename GyroInterp, typename AccelInterp>
void ImuBuffer<BufferSize, GyroInterp, AccelInterp>::insertGyroscopeMeasurement(
    time_t time, const Vector3 value)
{
  gyr_buffer_.insert(correctStampGyro(time), value);
}

template<int BufferSize, typename GyroInterp, typename AccelInterp>
void ImuBuffer<BufferSize, GyroInterp, AccelInterp>::insertAccelerometerMeasurement(
    time_t time, const Vector3 value)
{
  acc_buffer_.insert(correctStampAccel(time), value);
}

template<int BufferSize, typename GyroInterp, typename AccelInterp>
bool ImuBuffer<BufferSize, GyroInterp, AccelInterp>::get(int64_t time,
                                              Eigen::Ref<ImuAccGyr> out)
{
  std::lock_guard<std::mutex> gyr_lock(gyr_buffer_.mutex());
  std::lock_guard<std::mutex> acc_lock(acc_buffer_.mutex());

  if (time > gyr_buffer_.times().back()
      || time > acc_buffer_.times().back())
  {
    return false;
  }

  const auto gyro_before = gyr_buffer_.iterator_equal_or_before(time);
  const auto acc_before = acc_buffer_.iterator_equal_or_before(time);

  if (gyro_before == gyr_buffer_.times().end()
      || acc_before == acc_buffer_.times().end()) {
    return false;
  }

  VectorX w = GyroInterp::interpolate(&gyr_buffer_, time, gyro_before);
  VectorX a = AccelInterp::interpolate(&acc_buffer_, time, acc_before);

  out = imu_model_->undistort(a, w);
  return true;
}

template<int BufferSize, typename GyroInterp, typename AccelInterp>
bool ImuBuffer<BufferSize, GyroInterp, AccelInterp>::getAccelerometerDistorted(
    int64_t time,
    Eigen::Ref<Vector3> out)
{
  return acc_buffer_.getValueInterpolated(time, out);
}

template<int BufferSize, typename GyroInterp, typename AccelInterp>
bool ImuBuffer<BufferSize, GyroInterp, AccelInterp>::getGyroscopeDistorted(
    int64_t time,
    Eigen::Ref<Vector3> out)
{
  return gyr_buffer_.getValueInterpolated(time, out);
}

template<int BufferSize, typename GyroInterp, typename AccelInterp>
std::pair<ImuStamps, ImuAccGyrContainer>
ImuBuffer<BufferSize, GyroInterp, AccelInterp>::getBetweenValuesInterpolated(
    int64_t stamp_from, int64_t stamp_to)
{
  //Takes gyroscope timestamps and interpolates accelerometer measurements at
  // same times. Rectifies all measurements.
  CHECK_GE(stamp_from, 0u);
  CHECK_LT(stamp_from, stamp_to);
  ImuAccGyrContainer rectified_measurements;
  ImuStamps stamps;

  std::lock_guard<std::mutex> gyr_lock(gyr_buffer_.mutex());
  std::lock_guard<std::mutex> acc_lock(acc_buffer_.mutex());

  if(gyr_buffer_.times().size() < 2)
  {
    LOG(WARNING) << "Buffer has less than 2 entries.";
    // return empty means unsuccessful.
    return std::make_pair(stamps, rectified_measurements);
  }

  const time_t oldest_stamp = gyr_buffer_.times().front();
  const time_t newest_stamp = gyr_buffer_.times().back();
  if (stamp_from < oldest_stamp)
  {
    LOG(WARNING) << "Requests older timestamp than in buffer.";
    // return empty means unsuccessful.
    return std::make_pair(stamps, rectified_measurements);
  }
  if (stamp_to > newest_stamp)
  {
    LOG(WARNING) << "Requests newer timestamp than in buffer.";
    // return empty means unsuccessful.
    return std::make_pair(stamps, rectified_measurements);
  }

  const auto it_from_before = gyr_buffer_.iterator_equal_or_before(stamp_from);
  const auto it_to_after = gyr_buffer_.iterator_equal_or_after(stamp_to);
  CHECK(it_from_before != gyr_buffer_.times().end());
  CHECK(it_to_after != gyr_buffer_.times().end());
  const auto it_from_after = it_from_before + 1;
  const auto it_to_before = it_to_after - 1;
  if (it_from_after == it_to_before)
  {
    LOG(WARNING) << "Not enough data for interpolation";
    // return empty means unsuccessful.
    return std::make_pair(stamps, rectified_measurements);
  }

  // resize containers
  const size_t range = it_to_before.index() - it_from_after.index() + 3;
  rectified_measurements.resize(Eigen::NoChange, range);
  stamps.resize(range);

  // first element
  VectorX w = GyroInterp::interpolate(&gyr_buffer_, stamp_from, it_from_before);
  VectorX a = AccelInterp::interpolate(&acc_buffer_, stamp_from);
  stamps(0) = stamp_from;
  rectified_measurements.col(0) = imu_model_->undistort(a, w);

  // this is a real edge case where we hit the two consecutive timestamps
  //  with from and to.
  size_t col = 1;
  if (range > 2)
  {
    for (auto it=it_from_before+1; it!=it_to_after; ++it) {
      w = GyroInterp::interpolate(&gyr_buffer_, (*it), it);
      a = AccelInterp::interpolate(&acc_buffer_, (*it));
      stamps(col) = (*it);
      rectified_measurements.col(col) = imu_model_->undistort(a, w);
      ++col;
    }
  }

  // last element
  w = GyroInterp::interpolate(&gyr_buffer_, stamp_to, it_to_before);
  a = AccelInterp::interpolate(&acc_buffer_, stamp_to);
  stamps(range - 1) = stamp_to;
  rectified_measurements.col(range - 1) = imu_model_->undistort(a, w);

  return std::make_pair(stamps, rectified_measurements);
}

template<int BufferSize, typename GyroInterp, typename AccelInterp>
std::tuple<int64_t, int64_t, bool>
ImuBuffer<BufferSize, GyroInterp, AccelInterp>::getOldestAndNewestStamp() const
{
  std::tuple<int64_t, int64_t, bool> accel =
      acc_buffer_.getOldestAndNewestStamp();
  std::tuple<int64_t, int64_t, bool> gyro =
      gyr_buffer_.getOldestAndNewestStamp();

  if (!std::get<2>(accel) || !std::get<2>(gyro))
  {
    return std::make_tuple(-1, -1, false);
  }

  int64_t oldest = std::get<0>(accel) < std::get<0>(gyro) ?
                     std::get<0>(gyro) : std::get<0>(accel);

  int64_t newest = std::get<1>(accel) < std::get<1>(gyro) ?
                     std::get<1>(accel) : std::get<1>(gyro);

  // This is an extreme edge case where the accel and gyro measurements
  // do not overlap at all.
  if (oldest > newest)
  {
    return std::make_tuple(-1, -1, false);
  }

  return std::make_tuple(oldest, newest, true);
}

// A set of explicit declarations
template class ImuBuffer<2000, InterpolatorLinear>;
template class ImuBuffer<5000, InterpolatorLinear>;
template class ImuBuffer<2000, InterpolatorNearest>;
template class ImuBuffer<5000, InterpolatorNearest>;
template class ImuBuffer<2000, InterpolatorDifferentiatorLinear,
InterpolatorLinear>;
template class ImuBuffer<5000, InterpolatorDifferentiatorLinear,
InterpolatorLinear>;

} // namespace ze
