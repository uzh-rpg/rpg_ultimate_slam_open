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

#include <map>
#include <tuple>
#include <thread>
#include <utility>
#include <mutex>
#include <Eigen/Dense>
#include <ze/common/logging.hpp>
#include <ze/common/ring_view.hpp>

#include <ze/common/types.hpp>
#include <ze/common/time_conversions.hpp>

namespace ze {

//! @todo: move the interpolators somewhere where they make more sense?
//!
//! Interpolators have to implement:
//! _ interpolate(Ringbuffer<...>*, int64_t time, Ringbuffer<...>timering_t::iterator);
//! Passing the (optional) interator to the timestamp right before the to be
//! interpolated value speeds up the process.
//! The passed it_before is expected to be valid.
//!
//! A nearest neighbour "interpolator".
struct InterpolatorNearest
{
  template<typename Ringbuffer_T>
  static typename Ringbuffer_T::DataType interpolate(
      Ringbuffer_T* buffer,
      int64_t time,
      typename Ringbuffer_T::timering_t::iterator it_before)
  {
    // the end value
    auto it_after = it_before + 1;
    if (it_after == buffer->times_.end())
    {
      LOG(WARNING) << "Interpolation hit end of buffer.";
      return buffer->dataAtTimeIterator(it_before);
    }

    // The times are ordered, we can guarantee those differences to be positive
    if ((time - *it_before) < (*it_after - time))
    {
      return buffer->dataAtTimeIterator(it_before);
    }
    return buffer->dataAtTimeIterator(it_after);
  }

  template<typename Ringbuffer_T>
  static typename Ringbuffer_T::DataType interpolate(
      Ringbuffer_T* buffer,
      int64_t time)
  {
    auto it_before = buffer->iterator_equal_or_before(time);
    // caller should check the bounds:
    CHECK(it_before != buffer->times_.end());

    return interpolate(buffer, time, it_before);
  }
};

//! A simple linear interpolator
struct InterpolatorLinear
{
  template<typename Ringbuffer_T>
  static typename Ringbuffer_T::DataType interpolate(
      Ringbuffer_T* buffer,
      int64_t time,
      typename Ringbuffer_T::timering_t::iterator it_before)
  {
    // the end value
    auto it_after = it_before + 1;
    if (it_after == buffer->times_.end())
    {
      LOG(WARNING) << "Interpolation hit end of buffer.";
      return buffer->dataAtTimeIterator(it_before);
    }

    const real_t w1 =
        static_cast<real_t>(time - *it_before) /
        static_cast<real_t>(*it_after - *it_before);

    return (real_t{1.0} - w1) * buffer->dataAtTimeIterator(it_before)
        + w1 * buffer->dataAtTimeIterator(it_after);
  }

  template<typename Ringbuffer_T>
  static typename Ringbuffer_T::DataType interpolate(
      Ringbuffer_T* buffer,
      int64_t time)
  {
    auto it_before = buffer->iterator_equal_or_before(time);
    // caller should check the bounds:
    CHECK(it_before != buffer->times_.end());

    return interpolate(buffer, time, it_before);
  }
};
using DefaultInterpolator = InterpolatorLinear;


//! A fixed size timed buffer templated on the number of entries.
//! Opposed to the `Buffer`, values are expected to be received ORDERED in
//! TIME!
// Oldest entry: buffer.begin(), newest entry: buffer.rbegin()
template <typename Scalar, size_t ValueDim, size_t Size>
class Ringbuffer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Ringbuffer is friend with the interpolator types.
  friend struct InterpolatorNearest;
  friend struct InterpolatorLinear;

  typedef int64_t time_t;
  typedef Eigen::Matrix<time_t, Size, 1> times_t;
  typedef Eigen::Matrix<time_t, Eigen::Dynamic, 1> times_dynamic_t;
  typedef Eigen::Matrix<Scalar, ValueDim, Size> data_t;
  typedef Eigen::Matrix<Scalar, ValueDim, Eigen::Dynamic> data_dynamic_t;

  // time ring is used to keep track of the positions of the data
  // in the dataring
  // uses fixed size ring_view
  typedef ring_view<time_t> timering_t;

  using DataType = Eigen::Matrix<Scalar, ValueDim, 1>;
  using DataTypeMap = Eigen::Map<DataType>;

  // a series of return types
  using DataBoolPair = std::pair<DataType, bool>;
  using TimeDataBoolTuple = std::tuple<time_t, DataType, bool>;
  using TimeDataRangePair = std::pair<times_dynamic_t, data_dynamic_t>;

  Ringbuffer()
    : times_(timering_t(times_raw_.data(),
                        times_raw_.data() + Size,
                        times_raw_.data(),
                        0))
  {}

  //! no copy, no move as there is no way to track the mutex
  Ringbuffer(const Ringbuffer& from) = delete;
  Ringbuffer(const Ringbuffer&& from) = delete;

  inline void insert(time_t stamp,
                     const DataType& data)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    times_.push_back(stamp);
    data_.col(times_.back_idx()) = data;
  }

  //! Get value with timestamp closest to stamp. Boolean returns if successful.
  std::tuple<time_t, DataType, bool> getNearestValue(time_t stamp);

  //! Get oldest value in buffer.
  std::pair<DataType, bool> getOldestValue() const;

  //! Get newest value in buffer.
  std::pair<DataType, bool> getNewestValue() const;

  //! Get timestamps of newest and oldest entry.
  std::tuple<time_t, time_t, bool> getOldestAndNewestStamp() const;

  /*! @brief Get Values between timestamps.
   *
   * If timestamps are not matched, the values
   * are interpolated. Returns a vector of timestamps and a block matrix with
   * values as columns. Returns empty matrices if not successful.
   */
  template <typename Interpolator = DefaultInterpolator>
  TimeDataRangePair
  getBetweenValuesInterpolated(time_t stamp_from, time_t stamp_to);

  //! Get the values of the container at the given timestamps
  //! The requested timestamps are expected to be in order!
  template <typename Interpolator = DefaultInterpolator>
  data_dynamic_t getValuesInterpolated(times_dynamic_t stamps);

  //! Interpolate a single value
  template <typename Interpolator = DefaultInterpolator>
  bool getValueInterpolated(time_t t,  Eigen::Ref<data_dynamic_t> out);

  inline void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    times_.reset();
  }

  inline size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return times_.size();
  }

  inline bool empty() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return times_.empty();
  }

  //! technically does not remove but only moves the beginning of the ring
  inline void removeDataBeforeTimestamp(time_t stamp)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    removeDataBeforeTimestamp_impl(stamp);
  }

  inline void removeDataOlderThan(real_t seconds)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if(times_.empty())
    {
      return;
    }

    removeDataBeforeTimestamp_impl(
          times_.back() - secToNanosec(seconds));
  }

  inline void lock() const
  {
    mutex_.lock();
  }

  inline void unlock() const
  {
    mutex_.unlock();
  }

  const data_t& data() const
  {
    CHECK(!mutex_.try_lock()) << "Call lock() before accessing data.";
    return data_;
  }

  const timering_t& times() const
  {
    CHECK(!mutex_.try_lock()) << "Call lock() before accessing data.";
    return times_;
  }

  typename timering_t::iterator iterator_equal_or_before(time_t stamp);
  typename timering_t::iterator iterator_equal_or_after(time_t stamp);

  //! returns an iterator to the first element in the times_ ring that
  //! is greater or equal to stamp
  inline typename timering_t::iterator lower_bound(time_t stamp);

  inline std::mutex& mutex() {return mutex_;}

protected:
  mutable std::mutex mutex_;
  data_t data_;
  times_t times_raw_;
  timering_t times_;

  //! return the data at a given point in time
  inline DataType dataAtTimeIterator(typename timering_t::iterator iter) const
  {
    //! @todo: i believe this is wrong.
    return data_.col(iter.container_index());
  }

  //! return the data at a given point in time (const)
  inline DataType dataAtTimeIterator(typename timering_t::const_iterator iter) const
  {
    //! @todo: i believe this is wrong.
    return data_.col(iter.container_index());
  }

  //! shifts the starting point of the ringbuffer to the given timestamp
  //! no resizing or deletion happens.
  inline void removeDataBeforeTimestamp_impl(time_t stamp)
  {
    auto it = lower_bound(stamp);
    times_.reset_front(it.container_index());
  }
};

} // namespace ze

#include <ze/common/ringbuffer-inl.hpp>
