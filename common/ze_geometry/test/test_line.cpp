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

#include <functional>

#include <ze/cameras/camera_impl.hpp>
#include <ze/common/numerical_derivative.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/geometry/line.hpp>

TEST(LineTests, testLineGeneration)
{
  using namespace ze;
  // Create lines.
  size_t n = 100;
  Lines lines;
  Positions start, end;
  PinholeCamera cam = createTestPinholeCamera();
  Transformation T_W_C;
  T_W_C.setRandom();
  std::tie(start, end) = generateRandomVisibleLines(cam, T_W_C, n, lines);

  for (size_t i = 0; i < n; ++i)
  {
    Vector3 direction = lines[i].direction();
    Position anchor_point = lines[i].anchorPoint();
    // Check whether anchor point is really the closest point available.
    EXPECT_NEAR(direction.dot(anchor_point), 0.0, 1e-10);
    // Direction should be a normalized vector.
    EXPECT_NEAR(direction.norm(), 1.0, 1e-12);
    // Anchor, start and end point should be on the line.
    EXPECT_NEAR(lines[i].distanceToLine(anchor_point), 0.0, 1e-12);
    EXPECT_NEAR(lines[i].distanceToLine(start.col(i)), 0.0, 1e-12);
    EXPECT_NEAR(lines[i].distanceToLine(end.col(i)), 0.0, 1e-12);
  }
}

TEST(LineTests, testJacobian)
{
  using namespace ze;
  Transformation T_B_W, T_C_B;
  T_B_W.setRandom();
  T_C_B.setRandom();
  const Transformation T_C_W = T_C_B * T_B_W;
  const Transformation T_W_C = T_C_W.inverse();

  size_t num_tests = 100;
  Lines lines_W;
  Positions start_W, end_W;
  ze::PinholeCamera cam = createTestPinholeCamera();
  std::tie(start_W, end_W) = generateRandomVisibleLines(cam, T_W_C, num_tests, lines_W);

  // Create measurements.
  LineMeasurements measurements_C(3, num_tests);
  size_t i;
  for (i = 0; i < num_tests; ++i)
  {
    measurements_C.col(i) =
        lineMeasurementFromBearings(T_C_W * start_W.col(i),
                                    T_C_W * end_W.col(i));
  }

  auto measurementError = [&](const Transformation& T_B_W_in_lambda) {
    Transformation T_W_C_in_lambda = (T_C_B * T_B_W_in_lambda).inverse();
    Position camera_pos_W = T_W_C_in_lambda.getPosition();
    Vector3 measurement_W = T_W_C_in_lambda.getRotation().rotate(measurements_C.col(i));
    return lines_W[i].calculateMeasurementError(measurement_W, camera_pos_W);
  };

  LineMeasurements measurements_W = T_W_C.getRotationMatrix() * measurements_C;
  for (i = 0; i < num_tests; ++i)
  {
    Matrix26 J_numeric =
        numericalDerivative<Vector2, Transformation>(measurementError, T_B_W);
    Matrix26 J_analytic =
        dLineMeasurement_dPose(T_B_W, T_C_B, measurements_W.col(i),
                               lines_W[i].anchorPoint(), lines_W[i].direction());
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(J_numeric, J_analytic, 1e-9));
  }

}

ZE_UNITTEST_ENTRYPOINT
