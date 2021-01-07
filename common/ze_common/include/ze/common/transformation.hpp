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

#pragma once

#include <limits>
#include <vector>
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Eigen 3.2.7 uses std::binder1st and std::binder2nd which are deprecated since c++11
// Fix is in 3.3 devel (http://eigen.tuxfamily.org/bz/show_bug.cgi?id=872).
#include <kindr/minimal/quat-transformation.h>
#pragma diagnostic pop
#include <ze/common/types.hpp>
#include <ze/common/matrix.hpp>

namespace ze {

using Transformation = kindr::minimal::QuatTransformationTemplate<real_t>;
using Quaternion = kindr::minimal::RotationQuaternionTemplate<real_t>;
using AngleAxis = kindr::minimal::AngleAxisTemplate<real_t>;

using TransformationVector = std::vector<Transformation, Eigen::aligned_allocator<Transformation>>;
using QuaternionVector = std::vector<Quaternion, Eigen::aligned_allocator<Quaternion>>;

using StampedTransformation = std::pair<int64_t, Transformation>;
using StampedTransformationVector = std::vector<StampedTransformation,
                                                Eigen::aligned_allocator<StampedTransformation>>;


//------------------------------------------------------------------------------
// Odometry state containers.
struct Odometry
{
  int64_t stamp;
  Transformation T_W_B;
  Vector3 v_W;
  Vector3 omega_B;
};

// -----------------------------------------------------------------------------
// Transformation utils

// Right Jacobian for Exponential map in SO(3)
inline Matrix3 expmapDerivativeSO3(const Vector3& omega)
{
  real_t theta2 = omega.dot(omega);
  if (theta2 <= std::numeric_limits<real_t>::epsilon())
  {
    return I_3x3;
  }
  real_t theta = std::sqrt(theta2);  // rotation angle
  // element of Lie algebra so(3): X = omega^, normalized by normx
  const Matrix3 Y = skewSymmetric(omega) / theta;
  return I_3x3 - ((real_t{1} - std::cos(theta)) / (theta)) * Y
               + (real_t{1} - std::sin(theta) / theta) * Y * Y;
}

// Right Jacobian for Log map in SO(3)
inline Matrix3 logmapDerivativeSO3(const Vector3& omega)
{
  real_t theta2 = omega.dot(omega);
  if (theta2 <= std::numeric_limits<real_t>::epsilon())
  {
    return I_3x3;
  }
  real_t theta = std::sqrt(theta2);  // rotation angle
  const Matrix3 X = skewSymmetric(omega); // element of Lie algebra so(3): X = omega^
  return I_3x3 + real_t{0.5} * X
               + (real_t{1} / (theta * theta)
               - (real_t{1} + std::cos(theta)) / (real_t{2} * theta * std::sin(theta))) * X * X;
}

// -----------------------------------------------------------------------------
// Quaternion utils

//! Plus matrix for a quaternion. q_AB x q_BC = plus(q_AB) * q_BC.coeffs().
inline Matrix4 quaternionPlusMatrix(const Eigen::Quaternion<real_t>& q_AB)
{
  const Vector4& q = q_AB.coeffs();
  Matrix4 Q;
  Q(0,0) =  q[3]; Q(0,1) = -q[2]; Q(0,2) =  q[1]; Q(0,3) =  q[0];
  Q(1,0) =  q[2]; Q(1,1) =  q[3]; Q(1,2) = -q[0]; Q(1,3) =  q[1];
  Q(2,0) = -q[1]; Q(2,1) =  q[0]; Q(2,2) =  q[3]; Q(2,3) =  q[2];
  Q(3,0) = -q[0]; Q(3,1) = -q[1]; Q(3,2) = -q[2]; Q(3,3) =  q[3];
  return Q;
}

//! Opposite-Plus matrix for a quaternion q_AB x q_BC = oplus(q_BC) * q_AB.coeffs().
inline Matrix4 quaternionOplusMatrix(const Eigen::Quaternion<real_t>& q_BC)
{
  const Vector4& q = q_BC.coeffs();
  Matrix4 Q;
  Q(0,0) =  q[3]; Q(0,1) =  q[2]; Q(0,2) = -q[1]; Q(0,3) =  q[0];
  Q(1,0) = -q[2]; Q(1,1) =  q[3]; Q(1,2) =  q[0]; Q(1,3) =  q[1];
  Q(2,0) =  q[1]; Q(2,1) = -q[0]; Q(2,2) =  q[3]; Q(2,3) =  q[2];
  Q(3,0) = -q[0]; Q(3,1) = -q[1]; Q(3,2) = -q[2]; Q(3,3) =  q[3];
  return Q;
}

} // namespace ze
