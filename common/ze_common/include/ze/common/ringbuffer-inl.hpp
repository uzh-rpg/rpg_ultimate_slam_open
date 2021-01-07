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

namespace ze {

template <typename Scalar, size_t ValueDim, size_t Size>
typename Ringbuffer<Scalar, ValueDim, Size>::TimeDataBoolTuple
Ringbuffer<Scalar, ValueDim, Size>::getNearestValue(time_t stamp)
{
  CHECK_GE(stamp, 0u);

  std::lock_guard<std::mutex> lock(mutex_);

  if(times_.empty())
  {
    LOG(WARNING) << "Buffer is empty.";
    return std::make_tuple(-1, DataType(), false);
  }

  auto it_before = iterator_equal_or_before(stamp);
  //! @todo an approx equality could return the desired result immediately
  if(*it_before == stamp)
  {
    return std::make_tuple(*it_before, dataAtTimeIterator(it_before), true);
  }

  auto it_after = iterator_equal_or_after(stamp);
  //! @todo an approx equality could return the desired result immediately
  if(*it_after == stamp)
  {
    return std::make_tuple(*it_after, dataAtTimeIterator(it_after), true);
  }

  // Compute time difference between stamp and closest entries.
  time_t dt_after = -1, dt_before = -1;
  if(it_after != times_.end())
  {
    dt_after = *it_after - stamp;
  }
  if(it_before != times_.end())
  {
    dt_before = stamp - *it_before;
  }

  // Select which entry is closest based on time difference.
  if(dt_after < 0 && dt_before < 0)
  {
    CHECK(false) << "Should not occur.";
    return std::make_tuple(-1, DataType(), false);
  }
  else if(dt_after < 0)
  {
    return std::make_tuple(*it_before, dataAtTimeIterator(it_before), true);
  }
  else if(dt_before < 0)
  {
    return std::make_tuple(*it_after, dataAtTimeIterator(it_after), true);
  }
  else if(dt_after > 0 && dt_before > 0 && dt_after < dt_before)
  {
    return std::make_tuple(*it_after, dataAtTimeIterator(it_after), true);
  }

  return std::make_tuple(*it_before, dataAtTimeIterator(it_before), true);
}

template <typename Scalar, size_t ValueDim, size_t Size>
typename Ringbuffer<Scalar, ValueDim, Size>::DataBoolPair
Ringbuffer<Scalar, ValueDim, Size>::getOldestValue() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(times_.empty())
  {
    return std::make_pair(DataType(), false);
  }
  return std::make_pair(dataAtTimeIterator(times_.begin()), true);
}

template <typename Scalar, size_t ValueDim, size_t Size>
typename Ringbuffer<Scalar, ValueDim, Size>::DataBoolPair
Ringbuffer<Scalar, ValueDim, Size>::getNewestValue() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(times_.empty())
  {
    return std::make_pair(DataType(), false);
  }
  return std::make_pair(dataAtTimeIterator((times_.end()-1)), true);
}

template <typename Scalar, size_t ValueDim, size_t Size>
std::tuple<int64_t, int64_t, bool>
Ringbuffer<Scalar, ValueDim, Size>::getOldestAndNewestStamp() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(times_.empty())
  {
    return std::make_tuple(-1, -1, false);
  }
  return std::make_tuple(times_.front(), times_.back(), true);
}

template <typename Scalar, size_t ValueDim, size_t Size>
template <typename Interpolator>
typename Ringbuffer<Scalar, ValueDim, Size>::TimeDataRangePair
Ringbuffer<Scalar, ValueDim, Size>::getBetweenValuesInterpolated(
    time_t stamp_from,
    time_t stamp_to)
{
  CHECK_GE(stamp_from, 0u);
  CHECK_LT(stamp_from, stamp_to);
  times_dynamic_t stamps;
  data_dynamic_t values;

  std::lock_guard<std::mutex> lock(mutex_);
  if(times_.size() < 2)
  {
    LOG(WARNING) << "Buffer has less than 2 entries.";
    return std::make_pair(stamps, values); // return empty means unsuccessful.
  }

  const time_t oldest_stamp = times_.front();
  const time_t newest_stamp = times_.back();
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
  CHECK(it_from_before != times_.end());
  CHECK(it_to_after != times_.end());
  if(it_from_before == it_to_after)
  {
    LOG(FATAL) << "Not enough data for interpolation";
    return std::make_pair(stamps, values); // return empty means unsuccessful.
  }
  auto it_from_after = it_from_before + 1;
  auto it_to_before = it_to_after - 1;

  // resize containers
  size_t range = it_to_before.index() - it_from_after.index() + 3;
  stamps.resize(range);
  values.resize(ValueDim, range);

  // first element interpolated:
  stamps(0) = stamp_from;
  values.col(0) = Interpolator::interpolate(this, stamp_from, it_from_before);

  // this is a real edge case where we hit the two consecutive timestamps
  //  with from and to.
  if (range > 2)
  {
    // will we cross the boundaries of the ringbuffer?
    if (it_to_before.container_index() < it_from_after.container_index())
    {
      // first batch at end of data structure
      size_t end_block_size = times_raw_.size() - it_from_after.container_index();
      stamps.segment(1, end_block_size) = times_raw_.segment(
                                            it_from_after.container_index(),
                                            end_block_size);

      values.middleCols(1, end_block_size) =
          data_.middleCols(it_from_after.container_index(), end_block_size);
      // second batch at beginning
      size_t begin_block_size = range - 2 - end_block_size;
      stamps.segment(end_block_size + 1, begin_block_size) =
          times_raw_.segment(0, begin_block_size);

      values.middleCols(end_block_size + 1, begin_block_size) =
          data_.middleCols(0, begin_block_size);
    }
    // copyable in a single block
    else
    {
      stamps.segment(1, range - 2) = times_raw_.segment(
                                       it_from_after.container_index(),
                                       range - 2);

      values.middleCols(1, range - 2) = data_.middleCols(
                                                  it_from_after.container_index(),
                                                  range - 2);
    }
  }

  // last element interpolated
  stamps(range - 1) = stamp_to;

  values.col(range - 1) = Interpolator::interpolate(this, stamp_to, it_to_before);

  return std::make_pair(stamps, values);
}

template <typename Scalar, size_t ValueDim, size_t Size>
template <typename Interpolator>
typename Ringbuffer<Scalar, ValueDim, Size>::data_dynamic_t
Ringbuffer<Scalar, ValueDim, Size>::getValuesInterpolated(
    times_dynamic_t stamps)
{
  CHECK_GT(stamps.size(), 0);

  std::lock_guard<std::mutex> lock(mutex_);
  time_t oldest_time = times_.front();
  time_t newest_time = times_.back();

  data_dynamic_t values(ValueDim, stamps.size());

  // Starting point
  auto it_before = iterator_equal_or_before(stamps(0));
  values.col(0) = Interpolator::interpolate(this, stamps(0), it_before);

  for (int i = 1; i < stamps.size(); ++i)
  {
    // ensure that we stay within the bounds of the buffer
    CHECK_LT(stamps(i), newest_time);
    CHECK_GT(stamps(i), oldest_time);

    // advance to next value
    while (*(it_before + 1) < stamps(i))
    {
      ++it_before;
    }

    values.col(i) = Interpolator::interpolate(this, stamps(i), it_before);
  }

  return values;
}

template <typename Scalar, size_t ValueDim, size_t Size>
template <typename Interpolator>
bool Ringbuffer<Scalar, ValueDim, Size>::getValueInterpolated(
    time_t stamp,
    Eigen::Ref<typename Ringbuffer<Scalar, ValueDim, Size>::data_dynamic_t> out)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (stamp > times_.back())
  {
    return false;
  }

  // Starting point
  auto it_before = iterator_equal_or_before(stamp);
  if (it_before == times_.end())
  {
    return false;
  }

  out = Interpolator::interpolate(this, stamp, it_before);

  return true;
}

template <typename Scalar, size_t ValueDim, size_t Size>
typename Ringbuffer<Scalar, ValueDim, Size>::timering_t::iterator
Ringbuffer<Scalar, ValueDim, Size>::iterator_equal_or_before(time_t stamp)
{
  DEBUG_CHECK(!mutex_.try_lock()) << "Call lock() before accessing data.";
  auto it = lower_bound(stamp);

  if(*it == stamp)
  {
    return it; // Return iterator to key if exact key exists.
  }
  if(stamp > times_.back())
  {
    return (--times_.end()); // Pointer to last value.
  }
  if(it == times_.begin())
  {
    return times_.end(); // Invalid if data before first value.
  }
  --it;
  return it;
}

template <typename Scalar, size_t ValueDim, size_t Size>
typename Ringbuffer<Scalar, ValueDim, Size>::timering_t::iterator
Ringbuffer<Scalar, ValueDim, Size>::iterator_equal_or_after(time_t stamp)
{
  DEBUG_CHECK(!mutex_.try_lock()) << "Call lock() before accessing data.";
  return lower_bound(stamp);
}

template <typename Scalar, size_t ValueDim, size_t Size>
typename Ringbuffer<Scalar, ValueDim, Size>::timering_t::iterator
Ringbuffer<Scalar, ValueDim, Size>::lower_bound(time_t stamp)
{
  // implements a heuristic that assumes approx. equally spaced timestamps
  time_t time_range = times_.back() - times_.front();

  // stamp is out of range
  if (stamp > times_.back())
  {
    return times_.end();
  }

  if (stamp < times_.front())
  {
    return times_.begin();
  }

  size_t offset = times_.size() * (stamp - times_.front()) / time_range;
  // make sure we stay within the bounds
  if (offset >= times_.size())
  {
    offset = times_.size() - 1;
  }
  typename timering_t::iterator candidate = times_.begin() + offset;

  if (*candidate == stamp)
  {
    return candidate;
  }

  // iterate backwards
  if (*candidate > stamp)
  {
    while (candidate > times_.begin())
    {
      if (*(candidate - 1) < stamp)
      {
        return candidate;
      }
      candidate = candidate - 1;
    }
    // no match
  }
  else
  {
    while (candidate < times_.end() - 1)
    {
      candidate++;
      if (*candidate >= stamp)
      {
        return candidate;
      }
    }
    // no match
  }

  // no match points to end
  return times_.end();
}

} // namespace ze
