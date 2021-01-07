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

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/path_utils.hpp>

#include <ze/imu/imu_model.hpp>
#include <ze/imu/imu_rig.hpp>
#include <ze/imu/imu_yaml_serialization.hpp>

TEST(ImuYamlSerialization, testYamlLoading)
{
  std::string data_dir = ze::getTestDataDir("imu_models");
  std::string yaml_file = ze::joinPath(data_dir, "/zurich_eye_one.yaml");
  ASSERT_TRUE(ze::fileExists(yaml_file));

  ze::ImuRig::Ptr rig = ze::ImuRig::loadFromYaml(yaml_file);

  EXPECT_EQ(rig->size(), 3);
  EXPECT_STREQ(rig->label().c_str(), "zuricheyeonetest");
  EXPECT_STREQ(rig->at(0).label().c_str(), "bmx0");
  EXPECT_EQ(ze::ImuIntrinsicType::ScaleMisalignment,
            rig->at(0).accelerometerModel()->intrinsicModel()->type());
  EXPECT_EQ(ze::ImuIntrinsicType::ScaleMisalignment,
            rig->at(0).gyroscopeModel()->intrinsicModel()->type());

  EXPECT_STREQ(rig->at(1).label().c_str(), "bmx1");
  EXPECT_EQ(ze::ImuIntrinsicType::ScaleMisalignmentSizeEffect,
            rig->at(1).accelerometerModel()->intrinsicModel()->type());
  EXPECT_EQ(ze::ImuIntrinsicType::ScaleMisalignmentGSensitivity,
            rig->at(1).gyroscopeModel()->intrinsicModel()->type());

  EXPECT_STREQ(rig->at(2).label().c_str(), "bmx2");
  EXPECT_EQ(ze::ImuIntrinsicType::Calibrated,
            rig->at(2).accelerometerModel()->intrinsicModel()->type());
  EXPECT_EQ(ze::ImuIntrinsicType::Calibrated,
            rig->at(2).gyroscopeModel()->intrinsicModel()->type());
}

ZE_UNITTEST_ENTRYPOINT
