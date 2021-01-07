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
#include <ze/common/test_utils.hpp>
#include <ze/common/matrix.hpp>
#include <ze/common/numerical_derivative.hpp>
#include <ze/common/timer.hpp>
#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>
#include <ze/cameras/camera_utils.hpp>
#include <ze/cameras/camera_impl.hpp>
#include <ze/geometry/clam.hpp>

TEST(ClamTests, testJacobians)
{
#ifndef ZE_SINGLE_PRECISION_FLOAT
  using namespace ze;

  Transformation T_C_B, T_Bc_Br;
  T_C_B.setRandom(); // Random camera to imu/body transformation.
  T_Bc_Br = Transformation::exp((Vector6() << 0.2, 0.2, 0.2, 0.1, 0.1, 0.1).finished());

  PinholeCamera cam = createPinholeCamera(640, 480, 329.11, 329.11, 320.0, 240.0);
  Keypoint px_Cr(230.4, 325.6);

  Bearing f_Cr = cam.backProject(px_Cr);
  Bearing f_Br = T_C_B.getRotation().inverse().rotate(f_Cr);
  Bearing p_Br = T_C_B.inverse().getPosition();

  real_t inv_depth = 0.455;

  Keypoint px_Cc = cam.project(T_C_B * T_Bc_Br * T_C_B.inverse() * (f_Cr * (1.0 / inv_depth)));

  Matrix26 H1;
  Matrix21 H2;
  Vector2 res = reprojectionResidual(
        f_Br, p_Br, cam, T_C_B, T_Bc_Br, inv_depth, px_Cc, &H1, &H2);
  EXPECT_TRUE(EIGEN_MATRIX_ZERO(res, 1e-10));

  Matrix26 H1_numeric = numericalDerivative<Vector2, Transformation>(
        std::bind(&reprojectionResidual, f_Br, p_Br, cam, T_C_B,
                  std::placeholders::_1, inv_depth, px_Cc, nullptr, nullptr), T_Bc_Br);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(H1, H1_numeric, 1e-6));

  Matrix21 H2_numeric = numericalDerivative<Vector2, real_t>(
        std::bind(&reprojectionResidual, f_Br, p_Br, cam, T_C_B,
                  T_Bc_Br, std::placeholders::_1, px_Cc, nullptr, nullptr), inv_depth);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(H2, H2_numeric, 1e-6));
#else
  LOG(WARNING) << "Numerical derivative test ignored for single precision float.";
#endif
}


TEST(ClamTests, testExperiment)
{
  using namespace ze;

  Transformation T_C_B, T_Bc_Br;
  T_C_B.setRandom(); // Random camera to imu/body transformation.
  T_Bc_Br = Transformation::exp((Vector6() << 0.2, 0.2, 0.2, 0.1, 0.1, 0.1).finished());

  const size_t n = 120;
  std::string data_dir = ze::getTestDataDir("synthetic_room_pinhole");
  CameraRig::Ptr rig = ze::cameraRigFromYaml(data_dir + "/calib_rig.yaml");
  Keypoints px_Cr_true = generateRandomKeypoints(rig->at(0).size(), 10, n);
  Bearings f_Cr = rig->at(0).backProjectVectorized(px_Cr_true);
  Positions p_Cr = f_Cr;

  // Obtain the 3D points by applying a random scaling between 1 and 3 meters.
  std::ranlux24 gen;
  std::uniform_real_distribution<real_t> scale(1.0, 3.0);
  VectorX depth_true(n);
  for(size_t i = 0; i < n; ++i)
  {
    depth_true(i) = scale(gen);
    p_Cr.col(i) *= depth_true(i);
  }

  Positions p_Br = T_C_B.inverse().transformVectorized(p_Cr);
  Positions p_Cc = (T_C_B * T_Bc_Br).transformVectorized(p_Br);
  Keypoints px_Cc_true = rig->at(0).projectVectorized(p_Cc);

  // Apply some noise to the keypoints to simulate measurements.
  Keypoints px_Cc_noisy = px_Cc_true;
  const real_t stddev = 1.0;
  std::normal_distribution<real_t> px_noise(0.0, stddev);
  for(size_t i = 0; i < n; ++i)
  {
    px_Cc_noisy(0,i) += px_noise(gen);
    px_Cc_noisy(1,i) += px_noise(gen);
  }
  Bearings f_Cc_noisy = rig->at(0).backProjectVectorized(px_Cc_noisy);

  // Perturb pose:
  Transformation T_Bc_Br_perturbed =
      T_Bc_Br * Transformation::exp((Vector6() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());

  // Test Localization, without prior:
  {
    ClamFrameData data;
    data.f_C = f_Cc_noisy;
    data.p_Br = p_Br;
    data.T_C_B = T_C_B;
    std::vector<ClamFrameData> data_vec = { data };

    ClamLandmarks landmarks;

    real_t pos_prior_weight = 0.0;
    real_t rot_prior_weight = 0.0;
    ze::Timer t;
    Clam optimizer(
          landmarks, data_vec, *rig, T_Bc_Br, pos_prior_weight, rot_prior_weight);
    Transformation T_Bc_Br_estimate = T_Bc_Br_perturbed;

    ClamState state;
    state.at<0>() = T_Bc_Br_estimate;
    optimizer.optimize(state);
    VLOG(1) << "optimization took " << t.stopAndGetMilliseconds() << " ms\n";

    // Compute error:
    T_Bc_Br_estimate = state.at<0>();
    Transformation T_err = T_Bc_Br * T_Bc_Br_estimate.inverse();
    real_t pos_error = T_err.getPosition().norm();
    real_t ang_error = T_err.getRotation().log().norm();
    CHECK_LT(pos_error, 0.005);
    CHECK_LT(ang_error, 0.005);
    VLOG(1) << "ang error = " << ang_error;
    VLOG(1) << "pos error = " << pos_error;
  }

  // Test Mapping, with rotation prior:
  {
    ClamFrameData data;
    data.landmark_measurements.reserve(n);
    for(size_t i = 0; i < n; ++i)
    {
      if(isVisible(rig->at(0).width(), rig->at(0).height(), px_Cc_noisy.col(i)))
      {
        data.landmark_measurements.push_back(std::make_pair(i, px_Cc_noisy.col(i)));
      }
    }
    VLOG(1) << "# Visible Landmarks = " << data.landmark_measurements.size();
    data.T_C_B = T_C_B;
    std::vector<ClamFrameData> data_vec = { data };

    ClamLandmarks landmarks;
    landmarks.f_Br = T_C_B.getRotation().inverse().rotateVectorized(f_Cr);
    Vector3 t_Br_Cr = T_C_B.inverse().getPosition();
    landmarks.origin_Br = t_Br_Cr.replicate(1, landmarks.f_Br.cols());

    real_t pos_prior_weight = 0.2;
    real_t rot_prior_weight = 10.0;
    ze::Timer t;
    Clam optimizer(
          landmarks, data_vec, *rig, T_Bc_Br, pos_prior_weight, rot_prior_weight);
    Transformation T_Bc_Br_estimate = T_Bc_Br_perturbed;

    ClamState state;
    state.at<0>() = T_Bc_Br_estimate;
    VectorX& inv_depths = state.at<1>();
    inv_depths.resize(landmarks.f_Br.cols());
    inv_depths.setConstant(1.0 / 1.5);
    optimizer.optimize(state);
    VLOG(1) << "optimization took " << t.stopAndGetMilliseconds() << " ms\n";

    // Compute error:
    T_Bc_Br_estimate = state.at<0>();
    Transformation T_err = T_Bc_Br * T_Bc_Br_estimate.inverse();
    real_t pos_error = T_err.getPosition().norm();
    real_t ang_error = T_err.getRotation().log().norm();
    CHECK_LT(pos_error, 0.005);
    CHECK_LT(ang_error, 0.005);
    VLOG(1) << "ang error = " << ang_error;
    VLOG(1) << "pos error = " << pos_error;
  }

  // Test Localization AND mapping without prior:
  {
    ClamFrameData data;
    data.f_C = f_Cc_noisy.leftCols<10>();
    data.p_Br = p_Br.leftCols<10>();
    data.landmark_measurements.reserve(n);
    for(size_t i = 0; i < n; ++i)
    {
      if(isVisible(rig->at(0).width(), rig->at(0).height(), px_Cc_noisy.col(i)))
      {
        data.landmark_measurements.push_back(std::make_pair(i, px_Cc_noisy.col(i)));
      }
    }
    VLOG(1) << "# Visible Landmarks = " << data.landmark_measurements.size();
    data.T_C_B = T_C_B;
    std::vector<ClamFrameData> data_vec = { data };

    ClamLandmarks landmarks;
    landmarks.f_Br = T_C_B.getRotation().inverse().rotateVectorized(f_Cr);
    Vector3 t_Br_Cr = T_C_B.inverse().getPosition();
    landmarks.origin_Br = t_Br_Cr.replicate(1, landmarks.f_Br.cols());

    real_t pos_prior_weight = 0.0;
    real_t rot_prior_weight = 0.0;
    ze::Timer t;
    Clam optimizer(
          landmarks, data_vec, *rig, T_Bc_Br, pos_prior_weight, rot_prior_weight);
    Transformation T_Bc_Br_estimate = T_Bc_Br_perturbed;

    ClamState state;
    state.at<0>() = T_Bc_Br_estimate;
    VectorX& inv_depths = state.at<1>();
    inv_depths.resize(landmarks.f_Br.cols());
    inv_depths.setConstant(1.0 / 1.5);
    optimizer.optimize(state);
    VLOG(1) << "optimization took " << t.stopAndGetMilliseconds() << " ms\n";

    // Compute error:
    T_Bc_Br_estimate = state.at<0>();
    Transformation T_err = T_Bc_Br * T_Bc_Br_estimate.inverse();
    real_t pos_error = T_err.getPosition().norm();
    real_t ang_error = T_err.getRotation().log().norm();
    EXPECT_LT(pos_error, 0.007);
    EXPECT_LT(ang_error, 0.005);
    VLOG(1) << "ang error = " << ang_error;
    VLOG(1) << "pos error = " << pos_error;
  }
}




ZE_UNITTEST_ENTRYPOINT
