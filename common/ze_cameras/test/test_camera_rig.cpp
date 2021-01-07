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
#include <ze/common/test_utils.hpp>
#include <ze/common/path_utils.hpp>
#include <ze/cameras/camera.hpp>
#include <ze/cameras/camera_rig.hpp>

TEST(CameraRigTests, testYamlLoading)
{
  using namespace ze;
  CameraRig::Ptr rig =
      cameraRigFromYaml(joinPath(getTestDataDir("camera_models"),
                                 "camera_rig_1.yaml"));

  EXPECT_EQ(rig->size(), 2);
  EXPECT_NEAR(rig->at(0).projectionParameters()(0), 458.654, 1e-3);
  EXPECT_NEAR(rig->at(1).projectionParameters()(0), 457.587, 1e-3);
  EXPECT_NEAR(rig->T_C_B(0).getTransformationMatrix()(0, 0), 0.0148655, 1e-3);
  EXPECT_NEAR(rig->T_C_B(1).getTransformationMatrix()(0, 0), 0.0125553, 1e-3);
  EXPECT_STREQ(rig->label().c_str(), "Euroc");
  EXPECT_STREQ(rig->at(0).label().c_str(), "cam0");
  EXPECT_STREQ(rig->at(1).label().c_str(), "cam1");
}

TEST(CameraRigTests, testStereoPairIdentification)
{
  using namespace ze;

  {
    CameraRig::Ptr rig =
        cameraRigFromYaml(joinPath(getTestDataDir("camera_models"),
                                   "camera_rig_1.yaml"));
    StereoIndexPairs pairs = identifyStereoPairsInRig(*rig, 0.7, 0.1);
    EXPECT_EQ(pairs.size(), 1u);
  }

  {
    CameraRig::Ptr rig =
        cameraRigFromYaml(joinPath(getTestDataDir("camera_models"),
                                   "camera_rig_2.yaml"));
    StereoIndexPairs pairs = identifyStereoPairsInRig(*rig, 0.7, 0.1);
    EXPECT_EQ(pairs.size(), 1u);
  }

}

ZE_UNITTEST_ENTRYPOINT
