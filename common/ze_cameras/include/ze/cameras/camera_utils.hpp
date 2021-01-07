// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.

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

#include <tuple>

#include <ze/common/types.hpp>
#include <imp/core/size.hpp>

namespace ze {

// fwd
class Camera;
class CameraRig;

// -----------------------------------------------------------------------------
// Generate visible keypoints and landmarks.

//! Generate random visible keypoints.
Keypoints generateRandomKeypoints(
    const Size2u image_size,
    const uint32_t margin,
    const uint32_t num_keypoints);

//! Generate count random visible keypoints.
Keypoints generateUniformKeypoints(
    const Size2u image_size,
    const uint32_t margin,
    const uint32_t num_keypoints);

//! Generate random visible 3d points.
std::tuple<Keypoints, Bearings, Positions> generateRandomVisible3dPoints(
    const Camera& cam,
    const uint32_t num_points,
    const uint32_t margin = 10u,
    const real_t min_depth = 1.0,
    const real_t max_depth = 3.0);

// -----------------------------------------------------------------------------
// Check overlapping field of view.

//! Check if two cameras in a rig have an overlapping field of view.
//! @return Approximate percentage of overlapping field of view between cameras.
real_t overlappingFieldOfView(
    const CameraRig& rig,
    const uint32_t cam_a,
    const uint32_t cam_b);

// -----------------------------------------------------------------------------
// Check landmark visiblity.

//! Return if pixel u is within image boundaries.
template<typename DerivedKeyPoint>
bool isVisible(
    const Size2u image_size,
    const Eigen::MatrixBase<DerivedKeyPoint>& px)
{
  return px[0] >= 0u
      && px[1] >= 0u
      && px[0] <  image_size.width()
      && px[1] <  image_size.height();
}

//! Return if pixel u is within image boundaries.
template<typename DerivedKeyPoint>
bool isVisible(
    const typename DerivedKeyPoint::Scalar image_width,
    const typename DerivedKeyPoint::Scalar image_height,
    const Eigen::MatrixBase<DerivedKeyPoint>& px)
{
  return px[0] >= 0
      && px[1] >= 0
      && px[0] <  image_width
      && px[1] <  image_height;
}

//! Return if pixel px is within image boundaries with margin.
template<typename DerivedKeyPoint>
bool isVisibleWithMargin(
    const typename DerivedKeyPoint::Scalar image_width,
    const typename DerivedKeyPoint::Scalar image_height,
    const Eigen::MatrixBase<DerivedKeyPoint>& px,
    const typename DerivedKeyPoint::Scalar margin)
{
  return px[0] >= margin
      && px[1] >= margin
      && px[0] < (image_width - margin)
      && px[1] < (image_height - margin);
}

//! Return if pixel px is within image boundaries with margin.
template<typename DerivedKeyPoint>
bool isVisibleWithMargin(
    const Size2u image_size,
    const Eigen::MatrixBase<DerivedKeyPoint>& px,
    const typename DerivedKeyPoint::Scalar margin)
{
  return px[0] >= margin
      && px[1] >= margin
      && px[0] < (static_cast<typename DerivedKeyPoint::Scalar>(image_size.width()) - margin)
      && px[1] < (static_cast<typename DerivedKeyPoint::Scalar>(image_size.height()) - margin);
}

//! Return if pixel px is within image boundaries with margin.
inline bool isVisibleWithMargin(
    const int image_width, const int image_height, const int x, const int y, const int margin)
{
  return x >= margin
      && y >= margin
      && x < (image_width - margin)
      && y < (image_height - margin);
}

} // namespace ze
