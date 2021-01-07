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
#include <iostream>

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/time_conversions.hpp>
#include <ze/common/path_utils.hpp>
#include <ze/ros/rosbag_image_query.hpp>
#include <imp/core/image_base.hpp>

TEST(RosbagImageQueryTests, testImageQuery)
{
  using namespace ze;

  std::string data_dir = getTestDataDir("rosbag_euroc_snippet");
  std::string bag_filename = joinPath(data_dir, "dataset.bag");
  RosbagImageQuery rosbag(bag_filename);

  {
    // stamp of the first image in the bag:
    int64_t nsec = nanosecFromSecAndNanosec(1403636617, 863555500);
    auto res = rosbag.getStampedImageAtTime("/cam0/image_raw", nsec);
    EXPECT_EQ(res.first, nsec);
    EXPECT_TRUE(res.second != nullptr);
  }

  {
    // stamp of some image in the bag:
    int64_t nsec = nanosecFromSecAndNanosec(1403636618, 463555500);
    auto res = rosbag.getStampedImageAtTime("/cam0/image_raw", nsec);
    EXPECT_EQ(res.first, nsec);
    EXPECT_TRUE(res.second != nullptr);
  }
}

ZE_UNITTEST_ENTRYPOINT
