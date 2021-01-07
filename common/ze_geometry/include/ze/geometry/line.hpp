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

#pragma once

#include <ze/common/transformation.hpp>
#include <ze/common/types.hpp>

namespace ze {

// fwd
class Camera;
class Line;

// Convenience typedefs:
using Lines = std::vector<Line>;

inline real_t distanceToLine(const Position& pos,
                                const Position& line_anchor,
                                const Vector3& line_direction)
{
  return (pos - line_anchor).cross(line_direction).norm();
}

inline LineMeasurement lineMeasurementFromBearings(const Vector3& bearing_1,
                                                   const Vector3& bearing_2)
{
  return bearing_1.cross(bearing_2).normalized();
}

Matrix26 dLineMeasurement_dPose(const Transformation& T_B_W,
                                const Transformation& T_C_B,
                                const LineMeasurement& measurement_W,
                                const Position& line_anchor,
                                const Vector3& line_direction);


std::pair<Positions, Positions> generateRandomVisibleLines(
    const Camera& cam, const Transformation& T_W_C,
    size_t num_lines, Lines& lines_W);

Lines generateLinesFromEndpoints(const Positions& startpoints,
                                const Positions& endpoints);

class Line
{
public:
  Line() = default;

  Line(Quaternion orientation, real_t distance)
  : orientation_(orientation)
  , distance_(distance) {}


  inline Vector3 direction() const
  {
    return orientation_.rotate(Vector3::UnitX());
  }

  inline Position anchorPoint() const
  {
    return distance_ * orientation_.rotate(Vector3::UnitZ());
  }

  inline real_t distanceToLine(const Position& pos) const
  {
    return ze::distanceToLine(pos, anchorPoint(), direction());
  }

  Vector2 calculateMeasurementError(const Vector3& measurement_W,
                                    const Vector3& camera_position_W) const;

private:
  Quaternion orientation_;
  real_t distance_ = 0.0;
};

} // namespace ze
