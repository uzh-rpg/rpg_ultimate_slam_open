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
#include <ze/common/benchmark.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/matrix.hpp>
#include <ze/common/timer.hpp>
#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>
#include <ze/cameras/camera_utils.hpp>
#include <ze/cameras/camera_impl.hpp>
#include <ze/geometry/line.hpp>
#include <ze/geometry/pose_optimizer.hpp>
#include <ze/geometry/robust_cost.hpp>

namespace ze {

void testPoseOptimizer(
    const real_t pos_prior_weight,
    const real_t rot_prior_weight,
    const Transformation& T_B_W,
    const Transformation& T_B_W_perturbed,
    const PoseOptimizerFrameData& data,
    const std::string& description)
{
  SCOPED_TRACE(description);
  PoseOptimizerFrameDataVec data_vec = { data };
  Transformation T_B_W_estimate;
  auto fun = [&]()
  {
    PoseOptimizer optimizer(
          PoseOptimizer::getDefaultSolverOptions(),
          data_vec, T_B_W, pos_prior_weight, rot_prior_weight);
    T_B_W_estimate = T_B_W_perturbed;
    optimizer.optimize(T_B_W_estimate);
  };
  runTimingBenchmark(fun, 1, 10, description, true);

  // Compute error:
  Transformation T_err = T_B_W * T_B_W_estimate.inverse();
  real_t pos_error = T_err.getPosition().norm();
  real_t ang_error = T_err.getRotation().log().norm();
  EXPECT_LT(pos_error, 0.0075);
  EXPECT_LT(ang_error, 0.005);
  VLOG(1) << "ang error = " << ang_error;
  VLOG(1) << "pos error = " << pos_error;
}

} // namespace ze

TEST(PoseOptimizerTests, testSolver)
{
  using namespace ze;

  Transformation T_C_B, T_B_W;
  T_C_B.setRandom(); // Random camera to imu/body transformation.
  T_B_W.setRandom(); // Random body transformation.

  const size_t n = 120;
  PinholeCamera cam = createTestPinholeCamera();
  Keypoints px_true = generateRandomKeypoints(cam.size(), 10, n);

  Positions pos_C = cam.backProjectVectorized(px_true);

  // Obtain the 3D points by applying a random scaling between 1 and 3 meters.
  std::ranlux24 gen;
  std::uniform_real_distribution<double> scale(1.0, 3.0);
  for(size_t i = 0; i < n; ++i)
  {
    pos_C.col(i) *= scale(gen);
  }

  // Transform points to world coordinates.
  Positions pos_W = (T_B_W.inverse() * T_C_B.inverse()).transformVectorized(pos_C);

  // Apply some noise to the keypoints to simulate measurements.
  Keypoints px_noisy = px_true;
  VectorX pyr_scale(n);
  const double stddev = 1.0;
  std::normal_distribution<double> px_noise(0.0, stddev);
  for(size_t i = 0; i < n; ++i)
  {
    // Features distribute among all levels. features on higher levels have more
    // uncertainty.
    pyr_scale(i) = (1 << (i % 4));
    px_noisy(0,i) += pyr_scale(i) * px_noise(gen);
    px_noisy(1,i) += pyr_scale(i) * px_noise(gen);
  }
  Bearings bearings_noisy = cam.backProjectVectorized(px_noisy);

  // Perturb pose:
  Transformation T_B_W_perturbed =
      T_B_W * Transformation::exp((Vector6() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());

  // Optimize using bearing vectors:
  PoseOptimizerFrameData data;
  data.f = bearings_noisy;
  data.kp_idx = KeypointIndices(n, 1);
  data.p_W = pos_W;
  data.T_C_B = T_C_B;
  data.scale = pyr_scale;
  data.type = PoseOptimizerResidualType::UnitPlane;

  testPoseOptimizer(
        0.0, 0.0, T_B_W, T_B_W_perturbed, data, "UnitPlane, No Prior");
  testPoseOptimizer(
        10.0, 0.0, T_B_W, T_B_W_perturbed, data, "UnitPlane, Rotation Prior");
  testPoseOptimizer(
        10.0, 10.0, T_B_W, T_B_W_perturbed, data, "UnitPlane, Rotation and Position Prior");

  data.type = PoseOptimizerResidualType::Bearing;

  testPoseOptimizer(
        0.0, 0.0, T_B_W, T_B_W_perturbed, data, "Bearing, No Prior");
  testPoseOptimizer(
        10.0, 0.0, T_B_W, T_B_W_perturbed, data, "Bearing, Rotation Prior");
  testPoseOptimizer(
        10.0, 10.0, T_B_W, T_B_W_perturbed, data, "Bearing, Rotation and Position Prior");
}

TEST(PoseOptimizerTests, testSolver_withLines)
{
  using namespace ze;

  Transformation T_C_B, T_B_W;
  T_C_B.setRandom(); // Random camera to imu/body transformation.
  T_B_W.setRandom(); // Random body transformation.

  const size_t n = 130;
  PinholeCamera cam = createTestPinholeCamera();
  Keypoints endpoints_image;
  Bearings bearings_truth;
  Positions endpoints_C;
  std::tie(endpoints_image, bearings_truth, endpoints_C) =
      generateRandomVisible3dPoints(cam, 2 * n, 10, 1.0, 3.0);

  Positions endpoints_W =
      (T_B_W.inverse() * T_C_B.inverse()).transformVectorized(endpoints_C);

  Lines lines_W = generateLinesFromEndpoints(endpoints_W.block(0, 0, 3, n),
                                             endpoints_W.block(0, n, 3, n));

  // Apply some noise to the endpoints to simulate measurements.
  Keypoints endpoints_noisy = endpoints_image;
  const double stddev = 1.0;
  std::ranlux24 gen;
  std::normal_distribution<double> endpoints_noise(0.0, stddev);
  for (size_t i = 0; i < 2 * n; ++i)
  {
    endpoints_noisy(0, i) += endpoints_noise(gen);
    endpoints_noisy(1, i) += endpoints_noise(gen);
  }
  Bearings bearings_noisy = cam.backProjectVectorized(endpoints_noisy);
  LineMeasurements line_measurements_noisy(3, n);
  LineMeasurements line_measurements_truth(3, n);
  for (size_t i = 0; i < n; ++i)
  {
    line_measurements_noisy.col(i) =
        lineMeasurementFromBearings(bearings_noisy.col(i), bearings_noisy.col(n + i));
    line_measurements_truth.col(i) =
        lineMeasurementFromBearings(bearings_truth.col(i), bearings_truth.col(n + i));
  }

  // Check if error for truth is zero.
  PoseOptimizerFrameData data;
  data.line_measurements_C = line_measurements_truth;
  data.lines_W = lines_W;
  data.T_C_B = T_C_B;
  data.type = PoseOptimizerResidualType::Line;

  PoseOptimizerFrameDataVec data_vec = { data };
  PoseOptimizer optimizer(
        PoseOptimizer::getDefaultSolverOptions(),
        data_vec, T_B_W, 0.0, 0.0);
  real_t error = optimizer.evaluateError(T_B_W, nullptr, nullptr);
  EXPECT_NEAR(error, 0.0, 1e-5);

  // Perturb pose:
  Transformation T_B_W_perturbed =
      T_B_W * Transformation::exp((Vector6() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());

  // Optimize using noisy lines:
  data.line_measurements_C = line_measurements_noisy;
  testPoseOptimizer(
        0.0, 0.0, T_B_W, T_B_W_perturbed, data, "Line, No Prior");
}

ZE_UNITTEST_ENTRYPOINT
