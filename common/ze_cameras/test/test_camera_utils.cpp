// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.

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

#include <iostream>
#include <ze/common/test_entrypoint.hpp>
#include <ze/cameras/camera_utils.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/path_utils.hpp>
#include <ze/common/config.hpp>
#ifdef ZE_USE_OPENCV
#include <opencv2/highgui/highgui.hpp>
#endif

TEST(CameraUtilsTest, randomKeypoints)
{
  const int margin = 20;
  const int num_keypoints = 200;
  ze::Keypoints kps = ze::generateRandomKeypoints(ze::Size2u(640, 480), margin, num_keypoints);
  ASSERT_EQ(kps.cols(), num_keypoints);
  for (size_t i = 0; i < num_keypoints; ++i)
  {
    EXPECT_GE(kps(0,i), margin);
    EXPECT_GE(kps(1,i), margin);
    EXPECT_LT(kps(0,i), 640 - margin - 1);
    EXPECT_LT(kps(1,i), 480 - margin - 1);
  }

#ifdef ZE_USE_OPENCV
  if (false)
  {
    cv::Mat img(480, 640, CV_8UC1, cv::Scalar(0));
    for (size_t i = 0; i < num_keypoints; ++i)
    {
      cv::circle(img, cv::Point(kps(0,i), kps(1,i)), 2, cv::Scalar(255));
    }
    cv::imshow("img", img);
    cv::waitKey(0);
  }
#endif
}

TEST(CameraUtilsTest, uniformKeypoints)
{
  using namespace ze;

  {
    uint32_t margin = 0;
    Size2u img_size(752, 480);
    Keypoints kps = generateUniformKeypoints(img_size, margin, 50);
    for (int i = 0; i < kps.cols(); ++i)
    {
      EXPECT_TRUE(isVisibleWithMargin(img_size, kps.col(i), margin));
    }
  }

  {
    uint32_t margin = 10;
    Size2u img_size(752, 480);
    Keypoints kps = generateUniformKeypoints(img_size, margin, 50);
    for (int i = 0; i < kps.cols(); ++i)
    {
      EXPECT_TRUE(isVisibleWithMargin(img_size, kps.col(i), margin));
    }
  }
}

TEST(CameraUtilsTest, overlappingFieldOfView)
{
  using namespace ze;

  std::string data_dir = ze::getTestDataDir("camera_models");
  std::string yaml_file = ze::joinPath(data_dir, "camera_rig_1.yaml");
  ze::CameraRig::Ptr rig = ze::cameraRigFromYaml(yaml_file);

  real_t overlap1 = overlappingFieldOfView(*rig, 0u, 1u);
  real_t overlap2 = overlappingFieldOfView(*rig, 1u, 0u);

  EXPECT_NEAR(overlap1, overlap2, 0.1);
}

ZE_UNITTEST_ENTRYPOINT

