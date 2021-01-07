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

#include <ze/common/buffer.hpp>

namespace ze {

template <typename Scalar, int Dim>
std::tuple<int64_t, Eigen::Matrix<Scalar, Dim, 1>, bool>
Buffer<Scalar,Dim>::getNearestValue(int64_t stamp)
{
  CHECK_GE(stamp, 0);

  std::lock_guard<std::mutex> lock(mutex_);
  if(buffer_.empty())
  {
    LOG(WARNING) << "Buffer is empty.";
    return std::make_tuple(-1, Vector(), false);
  }

  auto it_before = iterator_equal_or_before(stamp);
  if(it_before->first == stamp)
  {
    return std::make_tuple(it_before->first, it_before->second, true);
  }

  // Compute time difference between stamp and closest entries.
  auto it_after = iterator_equal_or_after(stamp);
  int64_t dt_after = -1, dt_before = -1;
  if(it_after != buffer_.end())
  {
    dt_after = it_after->first - stamp;
  }
  if(it_before != buffer_.end())
  {
    dt_before = stamp - it_before->first;
  }

  // Select which entry is closest based on time difference.
  std::pair<int64_t,Vector> result;
  if(dt_after < 0 && dt_before < 0)
  {
    CHECK(false) << "Should not occur.";
    return std::make_tuple(-1, Vector(), false);
  }
  else if(dt_after < 0)
  {
    return std::make_tuple(it_before->first, it_before->second, true);
  }
  else if(dt_before < 0)
  {
    return std::make_tuple(it_after->first, it_after->second, true);
  }
  else if(dt_after > 0 && dt_before > 0 && dt_after < dt_before)
  {
    return std::make_tuple(it_after->first, it_after->second, true);
  }
  return std::make_tuple(it_before->first, it_before->second, true);
}

template <typename Scalar, int Dim>
std::pair<Eigen::Matrix<Scalar, Dim, 1>, bool> Buffer<Scalar,Dim>::getOldestValue() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(buffer_.empty())
  {
    return std::make_pair(Vector(), false);
  }
  return std::make_pair(buffer_.begin()->second, true);
}

template <typename Scalar, int Dim>
std::pair<Eigen::Matrix<Scalar, Dim, 1>, bool> Buffer<Scalar,Dim>::getNewestValue() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(buffer_.empty())
  {
    return std::make_pair(Vector(), false);
  }
  return std::make_pair(buffer_.rbegin()->second, true);
}

template <typename Scalar, int Dim>
std::tuple<int64_t, int64_t, bool> Buffer<Scalar,Dim>::getOldestAndNewestStamp() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(buffer_.empty())
  {
    return std::make_tuple(-1, -1, false);
  }
  return std::make_tuple(buffer_.begin()->first, buffer_.rbegin()->first, true);
}

template <typename Scalar, int Dim>
std::pair<Eigen::Matrix<int64_t, Eigen::Dynamic, 1>, Eigen::Matrix<Scalar, Dim, Eigen::Dynamic> >
Buffer<Scalar,Dim>::getBetweenValuesInterpolated(int64_t stamp_from, int64_t stamp_to)
{
  CHECK_GE(stamp_from, 0);
  CHECK_LT(stamp_from, stamp_to);
  Eigen::Matrix<int64_t, Eigen::Dynamic, 1> stamps;
  Eigen::Matrix<Scalar, Dim, Eigen::Dynamic> values;

  std::lock_guard<std::mutex> lock(mutex_);
  if(buffer_.size() < 2)
  {
    LOG(WARNING) << "Buffer has less than 2 entries.";
    return std::make_pair(stamps, values); // return empty means unsuccessful.
  }

  const int64_t oldest_stamp = buffer_.begin()->first;
  const int64_t newest_stamp = buffer_.rbegin()->first;
  if(stamp_from < oldest_stamp)
  {
    LOG(WARNING) << "Requests older timestamp than in buffer.";
    return std::make_pair(stamps, values); // return empty means unsuccessful.
  }
  if(stamp_to > newest_stamp)
  {
    LOG(WARNING) << "Requests newer timestamp than in buffer.";
    return std::make_pair(stamps, values); // return empty means unsuccessful.
  }

  auto it_from_before = iterator_equal_or_before(stamp_from);
  auto it_to_after = iterator_equal_or_after(stamp_to);
  CHECK(it_from_before != buffer_.end());
  CHECK(it_to_after != buffer_.end());
  auto it_from_after = it_from_before;
  ++it_from_after;
  auto it_to_before = it_to_after;
  --it_to_before;
  if(it_from_after == it_to_before)
  {
    LOG(WARNING) << "Not enough data for interpolation";
    return std::make_pair(stamps, values); // return empty means unsuccessful.
  }

  // Count number of measurements.
  size_t n = 0;
  auto it = it_from_after;
  while(it != it_to_after)
  {
    ++n;
    ++it;
  }
  n += 2;

  // Interpolate values at start and end and copy in output vector.
  stamps.resize(n);
  values.resize(kDim, n);
  for(size_t i = 0; i < n; ++i)
  {
    if(i == 0)
    {
      stamps(i) = stamp_from;
      const double w =
          static_cast<double>(stamp_from - it_from_before->first) /
          static_cast<double>(it_from_after->first - it_from_before->first);
      values.col(i) = (1.0 - w) * it_from_before->second + w * it_from_after->second;
    }
    else if(i == n-1)
    {
      stamps(i) = stamp_to;
      const double w =
          static_cast<double>(stamp_to - it_to_before->first) /
          static_cast<double>(it_to_after->first - it_to_before->first);
      values.col(i) = (1.0 - w) * it_to_before->second + w * it_to_after->second;
    }
    else
    {
      stamps(i) = it_from_after->first;
      values.col(i) = it_from_after->second;
      ++it_from_after;
    }
  }

  return std::make_pair(stamps, values);
}

template <typename Scalar, int Dim>
typename Buffer<Scalar,Dim>::VectorBuffer::iterator Buffer<Scalar,Dim>::iterator_equal_or_before(int64_t stamp)
{
  DEBUG_CHECK(!mutex_.try_lock()) << "Call lock() before accessing data.";
  auto it = buffer_.lower_bound(stamp);

  if(it->first == stamp)
  {
    return it; // Return iterator to key if exact key exists.
  }
  if(stamp > buffer_.rbegin()->first)
  {
    return (--buffer_.end()); // Pointer to last value.
  }
  if(it == buffer_.begin())
  {
    return buffer_.end(); // Invalid if data before first value.
  }
  --it;
  return it;
}

template <typename Scalar, int Dim>
typename Buffer<Scalar,Dim>::VectorBuffer::iterator Buffer<Scalar,Dim>::iterator_equal_or_after(int64_t stamp)
{
  DEBUG_CHECK(!mutex_.try_lock()) << "Call lock() before accessing data.";
  return buffer_.lower_bound(stamp);
}

} // namespace ze
