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

#include <ze/geometry/line.hpp>

#include <ze/cameras/camera_utils.hpp>

namespace ze {

// -----------------------------------------------------------------------------
Vector2 Line::calculateMeasurementError(const Vector3& measurement_W,
                                        const Vector3& camera_position_W) const
{
  const Vector3 anchor_to_camera = camera_position_W - anchorPoint();
  DEBUG_CHECK_GT(anchor_to_camera.norm(), 0.0);
  return Vector2(measurement_W.dot(direction()),
                 measurement_W.dot(anchor_to_camera) / anchor_to_camera.norm());
}

// -----------------------------------------------------------------------------
Matrix26 dLineMeasurement_dPose(const Transformation& T_B_W,
                                const Transformation& T_C_B,
                                const LineMeasurement& measurement_W,
                                const Position& line_anchor,
                                const Vector3& line_direction)
{
  //! @todo Can be optimized.
  const Transformation T_C_W = T_C_B * T_B_W;
  const Matrix13 measurement_W_transpose = measurement_W.transpose();
  const Vector3 anchor_to_cam = T_C_W.inverse().getPosition() - line_anchor;
  DEBUG_CHECK_NE(anchor_to_cam.norm(), 0.0);
  const real_t inverse_distance = 1.0 / anchor_to_cam.norm();
  const Vector3 anchor_to_cam_normalized = anchor_to_cam * inverse_distance;
  const Matrix3 d_anchor_to_cam_normalized_d_campos =
      inverse_distance * (Matrix3::Identity() -
                          anchor_to_cam_normalized *
                          anchor_to_cam_normalized.transpose());

  Matrix26 J;
  J.block<1, 3>(0, 0).setZero();
  J.block<1, 3>(0, 3) =
      -measurement_W_transpose * skewSymmetric(line_direction);

  J.block<1, 3>(1, 0) =
      -measurement_W_transpose * d_anchor_to_cam_normalized_d_campos;
  J.block<1, 3>(1, 3) =
      -measurement_W_transpose * (
        d_anchor_to_cam_normalized_d_campos *
        skewSymmetric(T_C_W.getRotation().inverse().rotate(T_C_B.getPosition()) +
         T_B_W.getRotation().inverse().rotate(T_B_W.getPosition()))
        + skewSymmetric(anchor_to_cam_normalized));

  return J;
}

// -----------------------------------------------------------------------------
std::pair<Positions, Positions> generateRandomVisibleLines(
    const Camera& cam, const Transformation& T_W_C,
    size_t num_lines, Lines& lines_W)
{
  auto visible_points_C =
      generateRandomVisible3dPoints(cam, num_lines * 2, 10, 1.0, 20.0);

  Positions start_W =
      T_W_C.transformVectorized(
        std::get<2>(visible_points_C).topLeftCorner(3, num_lines));
  Positions end_W =
      T_W_C.transformVectorized(
        std::get<2>(visible_points_C).topRightCorner(3, num_lines));

  lines_W = generateLinesFromEndpoints(start_W, end_W);

  return std::make_pair(start_W, end_W);
}

// -----------------------------------------------------------------------------
Lines generateLinesFromEndpoints(const Positions& startpoints,
                                const Positions& endpoints)
{
  CHECK_EQ(startpoints.cols(), endpoints.cols());
  const size_t n = startpoints.cols();
  Lines lines;
  lines.reserve(n);
  for (size_t i = 0; i < n; ++i)
  {
    const Vector3& start = startpoints.col(i);
    const Vector3& end = endpoints.col(i);
    Vector3 direction = (end - start).normalized();
    const double anchor_point_distance = (start.squaredNorm() - start.dot(end)) /
                                         (end - start).norm();
    const Vector3 anchor_point = start + anchor_point_distance * direction;
    const Vector3 anchor_point_direction = anchor_point.normalized();

    Matrix3 rotation;
    rotation << direction, anchor_point_direction.cross(direction), anchor_point_direction;
    const Quaternion orientation(rotation);
    lines.emplace_back(orientation, anchor_point.norm());
  }
  return lines;
}

} // namespace ze
