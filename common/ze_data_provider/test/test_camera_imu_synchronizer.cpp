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

#include <imp/core/image_base.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/image_cv.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/data_provider/data_provider_csv.hpp>
#include <ze/data_provider/data_provider_rosbag.hpp>
#include <ze/data_provider/camera_imu_synchronizer.hpp>

namespace ze {
// a dummy data provider
class DataProviderDummy : public DataProviderBase
{
public:
  DataProviderDummy(): DataProviderBase(DataProviderType::Rosbag) {}
  void spin() {}
  virtual bool spinOnce() { return true; }
  virtual bool ok() const { return true; }
  virtual size_t imuCount() const
  {
    return imu_count_;
  };
  virtual size_t cameraCount() const
  {
    return camera_count_;
  }
  size_t imu_count_;
  size_t camera_count_;
};
}

TEST(CameraImuSynchronizerTest, testCsv)
{
  using namespace ze;

  std::string data_dir = getTestDataDir("csv_dataset");
  EXPECT_FALSE(data_dir.empty());

  DataProviderCsv dp(data_dir+"/data", {{"imu0", 0}}, {{"cam0", 0}});
  CameraImuSynchronizer sync(dp);
  sync.registerCameraImuCallback(
        [&](const StampedImages& images,
            const ImuStampsVector& imu_timestamps,
            const ImuAccGyrVector& imu_measurements)
  {
    static int64_t last_img_stamp = 1403636579758555392;
    VLOG(1) << "Image stamp = " << images[0].first << ", "
            << "IMU-min = " << imu_timestamps[0](0) << ", "
            << "IMU-max = " << imu_timestamps[0](imu_timestamps.size()-1) << "\n";
    EXPECT_EQ(last_img_stamp,  imu_timestamps[0](0));
    EXPECT_EQ(images[0].first, imu_timestamps[0](imu_timestamps[0].size()-1));
    EXPECT_EQ(static_cast<int>(imu_timestamps[0].size()), imu_measurements[0].cols());
    last_img_stamp = images[0].first;
  });

  dp.spin();
}

TEST(CameraImuSynchronizerTest, testCameraOnlyPublishesFullFrames)
{
  using namespace ze;
  // fake data provider with two cameras
  DataProviderDummy data_provider;
  data_provider.camera_count_ = 2;
  data_provider.imu_count_ = 0;

  CameraImuSynchronizer sync(data_provider);

  size_t measurements = 0u;
  sync.registerCameraImuCallback(
        [&](const StampedImages& images,
            const ImuStampsVector& imu_timestamps,
            const ImuAccGyrVector& imu_measurements)
        {
          ++measurements;
          EXPECT_EQ(2, images.size());
          EXPECT_EQ(0, imu_timestamps.size());
          EXPECT_EQ(0, imu_measurements.size());
        }
  );

  // trigger callbacks twice for images
  int64_t stamp = 1403636579763555584;

  auto img1 = std::make_shared<ImageRaw8uC1>(1, 1);
  auto img2 = std::make_shared<ImageRaw8uC1>(1, 1);

  sync.addImageData(stamp, img1, 0);
  sync.addImageData(stamp, img2, 1);

  // cam imu callback only called once
  EXPECT_EQ(1, measurements);
}

TEST(CameraImuSynchronizerTest, testCameraOnlyPublishesWithImu)
{
  using namespace ze;
  // fake data provider with two cameras
  DataProviderDummy data_provider;
  data_provider.camera_count_ = 1;
  data_provider.imu_count_ = 1;

  CameraImuSynchronizer sync(data_provider);

  size_t measurements = 0u;
  sync.registerCameraImuCallback(
        [&](const StampedImages& images,
            const ImuStampsVector& imu_timestamps,
            const ImuAccGyrVector& imu_measurements)
        {
          ++measurements;
          EXPECT_EQ(1, images.size());
          EXPECT_EQ(1, imu_timestamps.size());
          EXPECT_EQ(1, imu_measurements.size());
        }
  );

  int64_t stamp1 = 1403636579763555583;
  int64_t stamp2 = 1403636579763555584;
  int64_t stamp3 = 1403636579763555585;

  auto img1 = std::make_shared<ImageRaw8uC1>(1, 1);

  sync.addImuData(stamp1, Vector3(), Vector3(), 0);
  EXPECT_EQ(0, measurements);

  sync.addImageData(stamp2, img1, 0);
  EXPECT_EQ(0, measurements);

  sync.addImuData(stamp3, Vector3(), Vector3(), 0);
  EXPECT_EQ(1, measurements);
}

TEST(CameraImuSynchronizerTest, testImagesAreDiscardedIfFramesInconsistent)
{
  using namespace ze;
  // fake data provider with two cameras
  DataProviderDummy data_provider;
  data_provider.camera_count_ = 2;
  data_provider.imu_count_ = 0;

  CameraImuSynchronizer sync(data_provider);

  size_t measurements = 0u;
  sync.registerCameraImuCallback(
        [&](const StampedImages& images,
            const ImuStampsVector& imu_timestamps,
            const ImuAccGyrVector& imu_measurements)
        {
          ++measurements;
          EXPECT_EQ(2, images.size());
          EXPECT_EQ(0, imu_timestamps.size());
          EXPECT_EQ(0, imu_measurements.size());
        }
  );

  // trigger callbacks twice for images
  int64_t stamp1 = 1403636579763555584;
  int64_t stamp2 = 1403636579863555584;

  auto img1 = std::make_shared<ImageRaw8uC1>(1, 1);
  auto img2 = std::make_shared<ImageRaw8uC1>(1, 1);

  sync.addImageData(stamp1, img1, 0);
  sync.addImageData(stamp2, img2, 1);

  // cam imu callback only called once
  EXPECT_EQ(0, measurements);

  // next image of cam0 that matches timestamp will trigger callback
  sync.addImageData(stamp2, img1, 0);

  EXPECT_EQ(1, measurements);
}

TEST(CameraImuSynchronizerTest, testImagesAreDiscardedIfNoImuBefore)
{
  using namespace ze;
  // fake data provider with two cameras
  DataProviderDummy data_provider;
  data_provider.camera_count_ = 1;
  data_provider.imu_count_ = 1;

  CameraImuSynchronizer sync(data_provider);

  size_t measurements = 0u;
  sync.registerCameraImuCallback(
        [&](const StampedImages& images,
            const ImuStampsVector& imu_timestamps,
            const ImuAccGyrVector& imu_measurements)
        {
          ++measurements;
        }
  );

  int64_t stamp1 = 1403636579763555584;
  int64_t stamp2 = 1403636579763555584;  // same as stamp1
  int64_t stamp3 = 1403636579763555585;  // after stamp1

  auto img1 = std::make_shared<ImageRaw8uC1>(1, 1);

  sync.addImageData(stamp1, img1, 0);
  sync.addImuData(stamp2, Vector3(), Vector3(), 0);
  sync.addImuData(stamp3, Vector3(), Vector3(), 0);

  EXPECT_EQ(0, measurements);
}

TEST(CameraImuSynchronizerTest, testImagesAreDiscardedIfNoImuAfter)
{
  using namespace ze;
  // fake data provider with two cameras
  DataProviderDummy data_provider;
  data_provider.camera_count_ = 1;
  data_provider.imu_count_ = 1;

  CameraImuSynchronizer sync(data_provider);

  size_t measurements = 0u;
  sync.registerCameraImuCallback(
        [&](const StampedImages& images,
            const ImuStampsVector& imu_timestamps,
            const ImuAccGyrVector& imu_measurements)
        {
          ++measurements;
        }
  );

  int64_t stamp1 = 1403636579763555584;
  int64_t stamp2 = 1403636579763555584;  // same as stamp1
  int64_t stamp3 = 1403636579763555583;  // before stamp1

  auto img1 = std::make_shared<ImageRaw8uC1>(1, 1);

  sync.addImuData(stamp3, Vector3(), Vector3(), 0);
  sync.addImageData(stamp1, img1, 0);
  sync.addImuData(stamp2, Vector3(), Vector3(), 0);

  EXPECT_EQ(0, measurements);
}

ZE_UNITTEST_ENTRYPOINT
