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

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/running_statistics.hpp>
#include <ze/common/running_statistics_collection.hpp>

TEST(RunningStatisticsTest, testRunningStatistics)
{
  ze::RunningStatistics stat;
  stat.addSample(1.1);
  stat.addSample(2.2);
  stat.addSample(3.3);
  stat.addSample(2.7);
  stat.addSample(4.5);
  EXPECT_EQ(stat.numSamples(), 5u);
  EXPECT_FLOATTYPE_EQ(stat.max(), 4.5);
  EXPECT_FLOATTYPE_EQ(stat.min(), 1.1);
  EXPECT_FLOATTYPE_EQ(stat.sum(), 13.8);
  EXPECT_FLOATTYPE_EQ(stat.mean(), 2.76);
  EXPECT_FLOATTYPE_EQ(stat.var(), 1.5979999999999994);
  EXPECT_FLOATTYPE_EQ(stat.std(), 1.2641202474448383);
  VLOG(1) << "Statistics:\n" << stat;
}

TEST(RunningStatisticsTest, testCollection)
{
  using namespace ze;

  DECLARE_STATISTICS(Statistics, stats, foo, bar);
  stats[Statistics::foo].addSample(1.0);
  stats[Statistics::foo].addSample(1.0);
  stats[Statistics::foo].addSample(1.0);
  stats[Statistics::bar].addSample(2.0);
  EXPECT_EQ(stats[Statistics::foo].numSamples(), 3u);
  EXPECT_EQ(stats[Statistics::bar].numSamples(), 1u);
  EXPECT_FLOATTYPE_EQ(stats[Statistics::foo].mean(), 1.0);
  EXPECT_FLOATTYPE_EQ(stats[Statistics::bar].mean(), 2.0);
  VLOG(1) << stats;
}

ZE_UNITTEST_ENTRYPOINT

