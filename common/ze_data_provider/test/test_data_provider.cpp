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
//
// Modified: Robotics and Perception Group

#include <string>
#include <iostream>

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/path_utils.hpp>
#include <ze/data_provider/data_provider_csv.hpp>
#include <ze/data_provider/data_provider_rosbag.hpp>
#include <imp/core/image_base.hpp>

TEST(DataProviderTests, testCsv)
{
  using namespace ze;

  std::string data_dir = getTestDataDir("csv_dataset");
  EXPECT_FALSE(data_dir.empty());

  DataProviderCsv dp(joinPath(data_dir, "data"), {{"imu0", 0}}, {{"cam0", 0}});

  size_t num_imu_measurements = 0u;
  int64_t expect_imu_stamp = 1403636579758555392;
  dp.registerImuCallback(
        [&](int64_t stamp, const Vector3& /*acc*/, const Vector3& /*gyr*/, const uint32_t /*imu_idx*/)
  {
    ++num_imu_measurements;
    if(stamp == expect_imu_stamp)
    {
      expect_imu_stamp = 0;
    }
  });

  size_t num_cam_measurements = 0u;
  int64_t expect_cam_stamp = 1403636579763555584;
  dp.registerCameraCallback(
        [&](int64_t stamp, const ImageBase::Ptr& /*img*/, uint32_t /*cam_idx*/)
  {
    ++num_cam_measurements;
    if(stamp == expect_cam_stamp)
    {
      expect_cam_stamp = 0;
    }
  });

  dp.spin();

  EXPECT_EQ(expect_imu_stamp, 0);
  EXPECT_EQ(expect_cam_stamp, 0);
  EXPECT_EQ(num_cam_measurements, 5u);
  EXPECT_EQ(num_imu_measurements, 69u);
}

TEST(DataProviderTests, testRosbag)
{
  using namespace ze;

  std::string data_dir = getTestDataDir("rosbag_euroc_snippet");
  std::string bag_filename = joinPath(data_dir, "dataset.bag");
  ASSERT_TRUE(fileExists(bag_filename));

  DataProviderRosbag dp(bag_filename, {{"/imu0", 0}}, { {"/cam0/image_raw", 0},
                                                        {"/cam1/image_raw", 1} });

  size_t num_imu_measurements = 0u;
  dp.registerImuCallback(
        [&](int64_t /*stamp*/, const Vector3& /*acc*/, const Vector3& /*gyr*/, const uint32_t imu_idx)
  {
    if(imu_idx == 0)
    {
      ++num_imu_measurements;
    }
  });

  size_t num_cam0_measurements = 0u, num_cam1_measurements = 0u;
  dp.registerCameraCallback(
        [&](int64_t /*stamp*/, const ImageBase::Ptr& /*img*/, uint32_t cam_idx)
  {
    if(cam_idx == 0)
    {
      ++num_cam0_measurements;
    }
    if(cam_idx == 1)
    {
      ++num_cam1_measurements;
    }
  });

  dp.spin();

  EXPECT_EQ(num_cam0_measurements, 21u);
  EXPECT_EQ(num_cam1_measurements, 20u);
  EXPECT_EQ(num_imu_measurements, 210u);
}

TEST(DataProviderTests, testRosbagCamOnly)
{
  using namespace ze;

  std::string data_dir = getTestDataDir("rosbag_euroc_snippet");
  std::string bag_filename = joinPath(data_dir, "dataset.bag");
  ASSERT_TRUE(fileExists(bag_filename));

  DataProviderRosbag dp(bag_filename, {}, { {"/cam0/image_raw", 0},
                                            {"/cam1/image_raw", 1} });

  size_t num_imu_measurements = 0u;
  dp.registerImuCallback(
        [&](int64_t /*stamp*/, const Vector3& /*acc*/, const Vector3& /*gyr*/, const uint32_t /*imu_idx*/)
  {
    ++num_imu_measurements;
  });

  size_t num_cam0_measurements = 0u, num_cam1_measurements = 0u;
  dp.registerCameraCallback(
        [&](int64_t /*stamp*/, const ImageBase::Ptr& /*img*/, uint32_t cam_idx)
  {
    if(cam_idx == 0)
    {
      ++num_cam0_measurements;
    }
    if(cam_idx == 1)
    {
      ++num_cam1_measurements;
    }
  });

  dp.spin();

  EXPECT_EQ(num_cam0_measurements, 21u);
  EXPECT_EQ(num_cam1_measurements, 20u);
  EXPECT_EQ(num_imu_measurements, 0u);
}


ZE_UNITTEST_ENTRYPOINT
