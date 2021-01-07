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

#include <ze/imu/imu_intrinsic_model.hpp>

TEST(IntrinsicModelTests, testIntrinsicModelCalibrated)
{
  using namespace ze;
  ImuIntrinsicModelCalibrated::Ptr model =
      std::make_shared<ImuIntrinsicModelCalibrated>();

  ASSERT_TRUE(ImuIntrinsicType::Calibrated == model->type());
}

TEST(IntrinsicModelTests, testIntrinsicModelScaleMisalignment)
{
  using namespace ze;
  Vector3 b; b << 1.0, 2.0, 3.0;
  Matrix3 M; M << 1.0, 0.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
  ImuIntrinsicModelScaleMisalignment::Ptr model =
      std::make_shared<ImuIntrinsicModelScaleMisalignment>(0.1, 10, b, M);

  EXPECT_TRUE(ImuIntrinsicType::ScaleMisalignment == model->type());
  EXPECT_DOUBLE_EQ(0.1, model->delay());
  EXPECT_EQ(10, model->range());
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(b, model->b()));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(M, model->M()));

  Vector3 primary = Vector3::Random();
  Vector3 secondary = Vector3::Random();
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(primary, model->undistort(
      model->distort(primary, secondary), secondary)));

  //make sure that inputs actually get altered.
  EXPECT_FALSE(EIGEN_MATRIX_EQUAL_DOUBLE(model->distort(
      primary, secondary), model->undistort(primary, secondary)));
}

TEST(IntrinsicModelTests, testIntrinsicModelScaleMisalignmentGSensitivity)
{
  using namespace ze;
  Vector3 b; b << 1.0, 2.0, 3.0;
  Matrix3 M; M << 1.0, 0.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
  Matrix3 Ma; Ma << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;

  ImuIntrinsicModelScaleMisalignmentGSensitivity::Ptr model =
      std::make_shared<ImuIntrinsicModelScaleMisalignmentGSensitivity>(
        0.1, 10, b, M, Ma);

  ASSERT_TRUE(ImuIntrinsicType::ScaleMisalignmentGSensitivity == model->type());

  EXPECT_DOUBLE_EQ(0.1, model->delay());
  EXPECT_EQ(10, model->range());
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(b, model->b()));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(M, model->M()));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(Ma, model->Ma()));

  Vector3 primary = Vector3::Random();
  Vector3 secondary = Vector3::Random();
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(primary, model->undistort(
      model->distort(primary, secondary), secondary)));

  //make sure that inputs actually get altered.
  EXPECT_FALSE(EIGEN_MATRIX_EQUAL_DOUBLE(model->distort(
      primary, secondary), model->undistort(primary, secondary)));
}

TEST(IntrinsicModelTests, testIntrinsicModelScaleMisalignmentSizeEffect)
{
  using namespace ze;
  Vector3 b; b << 1.0, 2.0, 3.0;
  Matrix3 M; M << 1.0, 0.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
  Matrix3 R; R << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
  ImuIntrinsicModelScaleMisalignmentSizeEffect::Ptr model =
      std::make_shared<ImuIntrinsicModelScaleMisalignmentSizeEffect>(
        0.1, 10, b, M, R);

  ASSERT_TRUE(ImuIntrinsicType::ScaleMisalignmentSizeEffect == model->type());

  EXPECT_DOUBLE_EQ(0.1, model->delay());
  EXPECT_EQ(10, model->range());
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(b, model->b()));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(M, model->M()));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(R, model->R()));

  Vector3 primary = Vector3::Random();
  Vector6 secondary = Vector6::Random();
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(primary, model->undistort(
      model->distort(primary, secondary), secondary)));

  //make sure that inputs actually get altered.
  EXPECT_FALSE(EIGEN_MATRIX_EQUAL_DOUBLE(model->distort(
      primary, secondary), model->undistort(primary, secondary)));
}

ZE_UNITTEST_ENTRYPOINT
