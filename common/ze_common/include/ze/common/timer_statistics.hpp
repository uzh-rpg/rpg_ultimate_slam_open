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

#include <ze/common/running_statistics.hpp>
#include <ze/common/timer.hpp>
#include <ze/common/types.hpp>

namespace ze {

// fwd
class TimedScope;

//! Collect statistics over multiple timings in milliseconds.
class TimerStatistics
{
public:
  inline void start()
  {
    t_.start();
  }

  //! Using the concept of "Initialization is Resource Acquisition" idiom, this
  //! function returns a timer object. When this timer object is destructed,
  //! the timer is stopped.
  inline TimedScope timeScope();

  inline real_t stop()
  {
    real_t t = t_.stopAndGetMilliseconds();
    stat_.addSample(t);
    return t;
  }

  inline real_t numTimings() const { return stat_.numSamples(); }
  inline real_t accumulated() const { return stat_.sum(); }
  inline real_t min() const { return stat_.min(); }
  inline real_t max() const { return stat_.max(); }
  inline real_t mean() const { return stat_.mean(); }
  inline real_t variance() const { return stat_.var(); }
  inline real_t standarDeviation() const { return stat_.std(); }
  inline void reset() { stat_.reset(); }
  inline const RunningStatistics& statistics() const { return stat_; }

private:
  Timer t_;
  RunningStatistics stat_;
};

//! This object is return from TimerStatistics::timeScope()
class TimedScope
{
public:
  TimedScope() = delete;

  TimedScope(TimerStatistics* timer)
    : timer_(timer)
  {
    timer_->start();
  }

  ~TimedScope()
  {
    timer_->stop();
  }
private:
  TimerStatistics* timer_;
};

inline TimedScope TimerStatistics::timeScope()
{
  // Returning a pointer to this should be safe, as inserting more elements in
  // the unordered map does not invalidate any references to other elements in
  // the unordered map (http://stackoverflow.com/questions/16781886).
  return TimedScope(this);
}

} // end namespace ze
