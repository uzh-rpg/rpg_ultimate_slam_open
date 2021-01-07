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

#include <chrono>
#include <thread>
#include <iostream>

#include <ze/common/string_utils.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/timer.hpp>
#include <ze/common/timer_collection.hpp>
#include <ze/common/timer_statistics.hpp>

TEST(TimerTests, testTimerStatistics)
{
  ze::TimerStatistics timer;
  for(int i = 0; i < 10; ++i)
  {
    timer.start();
    std::this_thread::sleep_for(ze::Timer::ms(10));
    timer.stop();
  }
  EXPECT_EQ(timer.numTimings(), 10u);
  EXPECT_NEAR(timer.mean(), 10.0, 0.5);
  EXPECT_NEAR(timer.accumulated(), 100.0, 10.0);
  EXPECT_GT(timer.max(), timer.min());
}

TEST(TimerTests, testTimerScope)
{
  ze::TimerStatistics timer;
  for(int i = 0; i < 10; ++i)
  {
    auto t = timer.timeScope();
    std::this_thread::sleep_for(ze::Timer::ms(10));
  }
  EXPECT_EQ(timer.numTimings(), 10u);
  EXPECT_NEAR(timer.mean(), 10.0, 0.5);
  EXPECT_NEAR(timer.accumulated(), 100.0, 10.0);
  EXPECT_GT(timer.max(), timer.min());
}

TEST(TimerTests, testTimerCollection)
{
  DECLARE_TIMER(TestTimer, timers, foo, bar);

  for(int i = 0; i < 10; ++i)
  {
    timers[TestTimer::foo].start();
    std::this_thread::sleep_for(ze::Timer::ms(50));
    timers[TestTimer::foo].stop();

    timers[TestTimer::bar].start();
    std::this_thread::sleep_for(ze::Timer::ms(10));
    timers[TestTimer::bar].stop();
  }
  VLOG(1) << timers;
  EXPECT_NO_FATAL_FAILURE(timers.saveToFile("/tmp", "test_timer.yaml"));
  EXPECT_EQ(timers.size(), 2u);
}

ZE_UNITTEST_ENTRYPOINT
