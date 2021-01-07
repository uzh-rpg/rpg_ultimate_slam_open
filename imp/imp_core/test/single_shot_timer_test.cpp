#include <gtest/gtest.h>

// system includes
#include <assert.h>
#include <cstdint>
#include <iostream>

#include <imp/core/timer.hpp>


TEST(IMPCoreTestSuite, singleShotTimerTest)
{
  ze::SingleShotTimer::TimePoint start_outer = ze::SingleShotTimer::Clock::now();
  ze::SingleShotTimer timer("unit test timer");
  ze::SingleShotTimer::TimePoint start_inner = ze::SingleShotTimer::Clock::now();

  for (int i=0; i<10000; ++i)
  {
    // idle
  }

  ze::SingleShotTimer::TimePoint end_inner = ze::SingleShotTimer::Clock::now();
  ze::SingleShotTimer::Nanoseconds duration = timer.elapsedNs();
  ze::SingleShotTimer::TimePoint end_outer = ze::SingleShotTimer::Clock::now();

  ze::SingleShotTimer::Nanoseconds duration_inner =
      std::chrono::duration_cast<ze::SingleShotTimer::Nanoseconds>(
        end_inner - start_inner);
  ze::SingleShotTimer::Nanoseconds duration_outer =
      std::chrono::duration_cast<ze::SingleShotTimer::Nanoseconds>(
        end_outer - start_outer);

  ASSERT_LE(duration_inner.count(), duration.count());
  ASSERT_GE(duration_outer.count(), duration.count());
}
