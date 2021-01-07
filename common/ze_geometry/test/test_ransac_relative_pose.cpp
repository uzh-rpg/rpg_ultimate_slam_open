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

#include <random>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/matrix.hpp>
#include <ze/common/timer.hpp>
#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>
#include <ze/cameras/camera_utils.hpp>
#include <ze/cameras/camera_impl.hpp>
#include <ze/geometry/ransac_relative_pose.hpp>

TEST(RansacRelativePoseTests, testCopyBearings)
{
  using namespace ze;

  size_t n = 100;
  Bearings f(3, n);
  f.setRandom();
  BearingsVector v = bearingsVectorFromBearings(f);
  for(size_t i = 0; i < n; ++i)
  {
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(v[i], f.col(i)));
  }
}

TEST(RansacRelativePoseTests, testFivePointAndTranslationOnly)
{
  using namespace ze;

  size_t n_points = 100;
  PinholeCamera cam = createPinholeCamera(640, 480, 329.11, 329.11, 320.0, 240.0);
  Keypoints px_ref;
  Bearings f_ref;
  Positions pos_ref;
  std::tie(px_ref, f_ref, pos_ref) = generateRandomVisible3dPoints(cam, n_points, 0, 2.0, 8.0);

  Transformation T_cur_ref;
  T_cur_ref.setRandom(1.5, 20.0 / 180.0 * M_PI);

  Bearings f_cur = T_cur_ref.transformVectorized(pos_ref);
  normalizeBearings(f_cur);

  // Insert outliers.
  size_t n_outliers = 20;
  for(size_t i = 0; i < n_outliers; ++i)
  {
    f_cur.col(i).swap(f_ref.col(i));
  }

  // Some outliers, Five-Point
  Transformation T;
  RansacRelativePose ransac(cam, 1.0);
  ransac.ogv_verbosity_level_ = 2;
  bool success = ransac.solve(f_ref, f_cur, RelativePoseAlgorithm::FivePoint, T);
  EXPECT_TRUE(success);
  EXPECT_EQ(ransac.inliers().size(), n_points - n_outliers);
  std::cout << "Error = " << (T.inverse() * T_cur_ref).log().transpose() << std::endl;

  // Some outliers, Estimate relative translation.
  T.getRotation() = T_cur_ref.getRotation();
  success = ransac.solve(f_ref, f_cur, RelativePoseAlgorithm::TwoPointTranslationOnly, T);
  EXPECT_TRUE(success);
  EXPECT_EQ(ransac.inliers().size(), n_points - n_outliers);
  std::cout << "Error = " << (T.inverse() * T_cur_ref).log().transpose() << std::endl;
}

TEST(RansacRelativePoseTests, testRotationOnly)
{
  using namespace ze;

  size_t n_points = 100;
  PinholeCamera cam = createPinholeCamera(640, 480, 329.11, 329.11, 320.0, 240.0);
  Keypoints px_ref;
  Bearings f_ref;
  Positions pos_ref;
  std::tie(px_ref, f_ref, pos_ref) = generateRandomVisible3dPoints(cam, n_points, 0, 2.0, 8.0);

  Transformation T_cur_ref;
  T_cur_ref.setRandom(1.5, 20.0 / 180.0 * M_PI);
  T_cur_ref.getPosition() = Vector3::Zero();

  Bearings f_cur = T_cur_ref.transformVectorized(pos_ref);
  normalizeBearings(f_cur);

  // Insert outliers.
  size_t n_outliers = 20;
  for(size_t i = 0; i < n_outliers; ++i)
  {
    f_cur.col(i).swap(f_ref.col(i));
  }

  // Some outliers, Five-Point
  Transformation T;
  RansacRelativePose ransac(cam, 1.0);
  ransac.ogv_verbosity_level_ = 2;
  bool success = ransac.solve(f_ref, f_cur, RelativePoseAlgorithm::TwoPointRotationOnly, T);
  EXPECT_TRUE(success);
  EXPECT_EQ(ransac.inliers().size(), n_points - n_outliers);
  std::cout << "Error = " << (T.inverse() * T_cur_ref).log().transpose() << std::endl;
}

ZE_UNITTEST_ENTRYPOINT
