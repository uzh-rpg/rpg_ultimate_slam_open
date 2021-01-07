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

#include <ze/common/benchmark.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/random.hpp>
#include <ze/common/running_statistics.hpp>

TEST(RandomTests, testRandomSampling)
{
  using namespace ze;

  // Deterministic sampling results always the same series of random numbers.
  EXPECT_EQ(sampleUniformIntDistribution<uint8_t>(true), 140u);
  EXPECT_EQ(sampleUniformIntDistribution<uint8_t>(true), 151u);
  EXPECT_EQ(sampleUniformIntDistribution<uint8_t>(true), 183u);

  EXPECT_EQ(sampleUniformIntDistribution<int>(true), 209652396);
  EXPECT_EQ(sampleUniformIntDistribution<int>(true), 398764591);
  EXPECT_EQ(sampleUniformIntDistribution<int>(true), 924231285);

  EXPECT_NEAR(sampleUniformRealDistribution<double>(true), 0.592844, 1e-5);
  EXPECT_NEAR(sampleUniformRealDistribution<double>(true), 0.844265, 1e-5);
  EXPECT_NEAR(sampleUniformRealDistribution<double>(true), 0.857945, 1e-5);

  EXPECT_NEAR(sampleNormalDistribution<double>(true, 1.0, 4.0), 5.4911797, 1e-5);
  EXPECT_NEAR(sampleNormalDistribution<double>(true, 1.0, 4.0), 1.2834369, 1e-5);
  EXPECT_NEAR(sampleNormalDistribution<double>(true, 1.0, 4.0), -4.689303, 1e-5);

  EXPECT_TRUE (flipCoin(true, 0.7));
  EXPECT_FALSE(flipCoin(true, 0.7));
  EXPECT_FALSE(flipCoin(true, 0.7));
  EXPECT_FALSE(flipCoin(true, 0.7));
  EXPECT_TRUE (flipCoin(true, 0.7));
  EXPECT_TRUE (flipCoin(true, 0.7));

  // Non-deterministic sampling, always results in different numbers:
  EXPECT_NE(sampleUniformIntDistribution<int>(false), 209652396);
  EXPECT_NE(sampleUniformIntDistribution<int>(false), 398764591);
  EXPECT_NE(sampleUniformIntDistribution<int>(false), 924231285);

  // Test mean and standard deviation of normal distribution.
  {
    RunningStatistics statistics;
    for (int i = 0; i < 10000; ++i)
    {
      statistics.addSample(sampleNormalDistribution<double>(false, 2.0, 5.0));
    }
    EXPECT_NEAR(statistics.mean(), 2.0, 0.2);
    EXPECT_NEAR(statistics.std(),  5.0, 0.2);
  }

  // Test coin flips.
  {
    RunningStatistics statistics;
    for (int i = 0; i < 10000; ++i)
    {
      statistics.addSample(static_cast<int>(flipCoin(false, 0.2)));
    }
    EXPECT_NEAR(statistics.mean(), 0.2, 0.2);
  }
}

TEST(RandomTests, testDistribution)
{
  using namespace ze;

  // Deterministic sampling results always the same series of random numbers.
  {
    auto f = uniformDistribution<uint8_t>(true);
    EXPECT_EQ(f(), 140u);
    EXPECT_EQ(f(), 151u);
    EXPECT_EQ(f(), 183u);
  }

  // Deterministic sampling results always the same series of random numbers.
  {
    auto f = uniformDistribution<double>(true, 1.0, 2.0);
    EXPECT_NEAR(f(), 1.59284, 1e-5);
    EXPECT_NEAR(f(), 1.84427, 1e-5);
    EXPECT_NEAR(f(), 1.85795, 1e-5);
  }

  // Deterministic sampling results always the same series of random numbers.
  {
    auto f = normalDistribution<float>(true, 3.0, 5.0);
    EXPECT_NEAR(f(), 14.06103, 1e-5);
    EXPECT_NEAR(f(), 8.81539, 1e-5);
    EXPECT_NEAR(f(), 6.87001, 1e-5);
  }
}

TEST(RandomTests, benchmark)
{
  using namespace ze;

  auto lambda1 = [&]()
  {
    int sum = 0;
    for (int i = 0; i < 100000; ++i)
      sum += sampleUniformIntDistribution<uint8_t>(false);
  };
  runTimingBenchmark(lambda1, 10, 10, "sampleSeparately", true);

  auto lambda2 = [&]()
  {
    int sum = 0;
    auto dist = uniformDistribution<uint8_t>(false);
    for (int i = 0; i < 100000; ++i)
      sum += dist();
  };
  runTimingBenchmark(lambda2, 10, 10, "sampleFromDistribution", true);

  auto lambda3 = [&]()
  {
    int sum = 0;
    static std::mt19937 gen_deterministic(0);
    std::uniform_int_distribution<uint8_t> distribution(0, 255);
    for (int i = 0; i < 100000; ++i)
    {
      sum += distribution(gen_deterministic);
    }
  };
  runTimingBenchmark(lambda3, 10, 10, "Using std interface", true);
}

ZE_UNITTEST_ENTRYPOINT
