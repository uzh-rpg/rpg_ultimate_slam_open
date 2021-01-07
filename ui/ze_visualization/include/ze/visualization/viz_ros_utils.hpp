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

#include <visualization_msgs/Marker.h>
#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>

namespace ze {

inline std_msgs::ColorRGBA getRosColor(const Color& color)
{
  std_msgs::ColorRGBA c;
  c.r = color.r;
  c.g = color.g;
  c.b = color.b;
  c.a = color.a;
  return c;
}

inline geometry_msgs::Point getRosPoint(const Eigen::Ref<const Position>& point)
{
  geometry_msgs::Point p;
  p.x = point(0);
  p.y = point(1);
  p.z = point(2);
  return p;
}

inline geometry_msgs::Quaternion getRosQuaternion(const Quaternion& rot)
{
  geometry_msgs::Quaternion q;
  q.x = rot.toImplementation().x();
  q.y = rot.toImplementation().y();
  q.z = rot.toImplementation().z();
  q.w = rot.toImplementation().w();
  return q;
}

inline geometry_msgs::Pose getRosPose(const Transformation& pose)
{
  geometry_msgs::Pose T;
  T.position = getRosPoint(pose.getPosition());
  T.orientation = getRosQuaternion(pose.getRotation());
  return T;
}

} // namespace ze
