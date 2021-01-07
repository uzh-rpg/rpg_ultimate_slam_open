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

#include <string>
#include <vector>

#include <ze/common/buffer.hpp>
#include <ze/common/test_entrypoint.hpp>


TEST(BufferTest, testRemoveOlderThanTimestamp)
{
  ze::Buffer<double, 2> buffer;
  for(int i = 1; i < 10; ++i)
  {
    buffer.insert(i, Eigen::Vector2d(i, i));
  }

  buffer.removeDataBeforeTimestamp(3);
  buffer.lock();
  EXPECT_EQ(buffer.data().begin()->first, 3);
  EXPECT_EQ(buffer.data().rbegin()->first, 9);
  buffer.unlock();
}

TEST(BufferTest, testRemoveOlderThan)
{
  ze::Buffer<double, 2> buffer;
  for(int i = 1; i < 10; ++i)
  {
    buffer.insert(ze::secToNanosec(i), Eigen::Vector2d(i, i));
  }

  buffer.removeDataOlderThan(3.0);
  buffer.lock();
  EXPECT_EQ(buffer.data().begin()->first, ze::secToNanosec(6));
  EXPECT_EQ(buffer.data().rbegin()->first, ze::secToNanosec(9));
  buffer.unlock();
}

TEST(BufferTest, testIterator)
{
  ze::Buffer<double, 2> buffer;
  for(int i = 1; i < 10; ++i)
  {
    buffer.insert(ze::secToNanosec(i), Eigen::Vector2d(i, i));
  }

  buffer.lock();

  // Check before/after
  EXPECT_EQ(buffer.iterator_equal_or_before(ze::secToNanosec(3.5))->first,
            ze::secToNanosec(3));
  EXPECT_EQ(buffer.iterator_equal_or_after(ze::secToNanosec(3.5))->first,
            ze::secToNanosec(4));

  // Check equal
  EXPECT_EQ(buffer.iterator_equal_or_before(ze::secToNanosec(3))->first,
            ze::secToNanosec(3));
  EXPECT_EQ(buffer.iterator_equal_or_after(ze::secToNanosec(4))->first,
            ze::secToNanosec(4));

  // Expect out of range:
  EXPECT_EQ(buffer.iterator_equal_or_before(ze::secToNanosec(0.8)),
            buffer.data().end());
  EXPECT_EQ(buffer.iterator_equal_or_before(ze::secToNanosec(9.1)),
            (--buffer.data().end()));
  EXPECT_EQ(buffer.iterator_equal_or_after(ze::secToNanosec(9.1)),
            buffer.data().end());
  EXPECT_EQ(buffer.iterator_equal_or_after(ze::secToNanosec(0.8)),
            buffer.data().begin());

  buffer.unlock();
}

TEST(BufferTest, testNearestValue)
{
  ze::Buffer<double, 2> buffer;
  EXPECT_FALSE(std::get<2>(buffer.getNearestValue(ze::secToNanosec(1))));

  for(int i = 1; i < 10; ++i)
  {
    buffer.insert(ze::secToNanosec(i), Eigen::Vector2d(i, i));
  }

  EXPECT_EQ(std::get<1>(buffer.getNearestValue(ze::secToNanosec(1)))[0], 1);
  EXPECT_EQ(std::get<1>(buffer.getNearestValue(ze::secToNanosec(0.4)))[0], 1);
  EXPECT_EQ(std::get<1>(buffer.getNearestValue(ze::secToNanosec(1.4)))[0], 1);
  EXPECT_EQ(std::get<1>(buffer.getNearestValue(ze::secToNanosec(11.0)))[0], 9);
}

TEST(BufferTest, testOldestNewestValue)
{
  ze::Buffer<double, 2> buffer;
  EXPECT_FALSE(buffer.getOldestValue().second);
  EXPECT_FALSE(buffer.getNewestValue().second);

  for(int i = 1; i < 10; ++i)
  {
    buffer.insert(ze::secToNanosec(i), Eigen::Vector2d(i, i));
  }

  EXPECT_EQ(buffer.getNewestValue().first[0], 9);
  EXPECT_EQ(buffer.getOldestValue().first[0], 1);
}

TEST(BufferTest, testInterpolation)
{
  using namespace ze;

  Buffer<real_t, 2> buffer;

  for(int i = 0; i < 10; ++i)
    buffer.insert(secToNanosec(i), Vector2(i, i));

  Eigen::Matrix<int64_t, Eigen::Dynamic, 1> stamps;
  Eigen::Matrix<real_t, 2, Eigen::Dynamic> values;
  std::tie(stamps, values) = buffer.getBetweenValuesInterpolated(
        secToNanosec(1.2), secToNanosec(5.4));

  EXPECT_EQ(stamps.size(), values.cols());
  EXPECT_EQ(stamps.size(), 6);
  EXPECT_EQ(stamps(0), secToNanosec(1.2));
  EXPECT_EQ(stamps(stamps.size()-1), secToNanosec(5.4));
  EXPECT_FLOATTYPE_EQ(values(0, 0), 1.2);
  EXPECT_FLOATTYPE_EQ(values(0, stamps.size()-1), 5.4);

  std::tie(stamps, values) = buffer.getBetweenValuesInterpolated(
        secToNanosec(0), secToNanosec(9));
  EXPECT_EQ(stamps(0), secToNanosec(0));
  EXPECT_EQ(stamps(stamps.size()-1), secToNanosec(9));
  EXPECT_FLOATTYPE_EQ(values(0, 0), 0);
  EXPECT_FLOATTYPE_EQ(values(0, stamps.size()-1), 9);
}

ZE_UNITTEST_ENTRYPOINT
