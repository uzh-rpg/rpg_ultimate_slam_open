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

#include <functional>
#include <limits>

#include <ze/common/logging.hpp>
#include <ze/common/types.hpp>
#include <ze/common/time_conversions.hpp>
#include <ze/common/timer.hpp>

namespace ze {

//! Benchmark utilty for unit tests. Runs a function many times and reports the
//! minimum time. See test_benchmark.cpp for usage examples.
template <typename Lambda>
uint64_t runTimingBenchmark(
    const Lambda& benchmark_fun, uint32_t num_iter_per_epoch, uint32_t num_epochs,
    const std::string& benchmark_name = "", bool print_results = false)
{
  // Define lambda that runs experiment num_iter_per_epoch times and measures.
  auto executeFunMultipleTimes = [=]() -> int64_t
  {
    Timer t;
    // Measurement starts.
    for (uint32_t i = 0; i < num_iter_per_epoch; ++i)
    {
      benchmark_fun();
    }
    // Measurement ends.
    return t.stopAndGetNanoseconds();
  };

  uint64_t min_time = std::numeric_limits<uint64_t>::max();
  for (uint32_t i = 0; i < num_epochs; ++i)
  {
    // Call function.
    uint64_t timing = executeFunMultipleTimes();

    // According to Andrei Alexandrescu, the best measure is to take the minimum.
    // See talk: https://www.youtube.com/watch?v=vrfYLlR8X8k
    min_time = std::min(timing, min_time);
  }

  if(print_results)
  {
    VLOG(1) << "Benchmark: " << benchmark_name << "\n"
            << "> Time for " << num_iter_per_epoch << " iterations: "
            << nanosecToMillisecTrunc(min_time) << " milliseconds\n"
            << "> Time for 1 iteration: "
            << nanosecToMillisecTrunc(min_time) / num_iter_per_epoch << " milliseconds";
  }

  return min_time;
}


} // namespace ze
