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

#include <ze/cameras/camera_utils.hpp>

#include <random>
#include <ze/common/config.hpp>
#include <ze/common/logging.hpp>
#include <ze/common/random_matrix.hpp>

#include <ze/cameras/camera_rig.hpp>

namespace ze {

// -----------------------------------------------------------------------------
Keypoints generateRandomKeypoints(
    const Size2u size,
    const uint32_t margin,
    const uint32_t num_keypoints)
{
  DEBUG_CHECK_GT(size.width(), margin + 1u);
  DEBUG_CHECK_GT(size.height(), margin + 1u);

  Keypoints kp(2, num_keypoints);
  for(uint32_t i = 0u; i < num_keypoints; ++i)
  {
    kp(0,i) = sampleUniformRealDistribution<real_t>(false, margin, size.width() - 1 - margin);
    kp(1,i) = sampleUniformRealDistribution<real_t>(false, margin, size.height() - 1 - margin);
  }
  return kp;
}

// -----------------------------------------------------------------------------
Keypoints generateUniformKeypoints(
    const Size2u size,
    const uint32_t margin,
    const uint32_t num_cols)
{
  DEBUG_CHECK_GT(size.width(), margin + 1u);
  DEBUG_CHECK_GT(size.height(), margin + 1u);
  const uint32_t num_rows = num_cols * size.height() / size.width();

  // Compute width and height of a cell:
  real_t w = (static_cast<real_t>(size.width() - 0.01)  - 2.0 * margin) / (num_cols - 1);
  real_t h = (static_cast<real_t>(size.height() - 0.01) - 2.0 * margin) / (num_rows - 1);

  // Sample keypoints:
  Keypoints kp(2, num_rows * num_cols);
  for (uint32_t y = 0u; y < num_rows; ++y)
  {
    for (uint32_t x = 0u; x < num_cols; ++x)
    {
      uint32_t i = y * num_cols + x;
      kp(0,i) = margin + x * w;
      kp(1,i) = margin + y * h;
    }
  }
  return kp;
}

// -----------------------------------------------------------------------------
std::tuple<Keypoints, Bearings, Positions> generateRandomVisible3dPoints(
    const Camera& cam,
    const uint32_t num_points,
    const uint32_t margin,
    const real_t min_depth,
    const real_t max_depth)
{
  Keypoints px = generateRandomKeypoints(cam.size(), margin, num_points);
  Bearings f = cam.backProjectVectorized(px);
  Positions pos = f;
  for(uint32_t i = 0u; i < num_points; ++i)
  {
    pos.col(i) *= sampleUniformRealDistribution<real_t>(false, min_depth, max_depth);
  }
  return std::make_tuple(px, f, pos);
}

// -----------------------------------------------------------------------------
real_t overlappingFieldOfView(
    const CameraRig& rig,
    const uint32_t cam_A,
    const uint32_t cam_B)
{
  DEBUG_CHECK_LT(cam_A, rig.size());
  DEBUG_CHECK_LT(cam_B, rig.size());

  // We sample uniformly keypoints in camera a and project them into camera b,
  // assuming the landmark is at infinity (i.e. only rotate).
  Keypoints px_A = generateUniformKeypoints(rig.at(cam_A).size(), 0u, 20u);
  Bearings f_A = rig.at(cam_A).backProjectVectorized(px_A);
  Transformation T_B_A = rig.T_C_B(cam_B) * rig.T_C_B(cam_A).inverse();
  Positions p_B = T_B_A.getRotation().rotateVectorized(f_A);
  Keypoints px_B = rig.at(cam_B).projectVectorized(p_B);

  uint32_t num_visible = 0u;
  for (int i = 0; i < px_B.cols(); ++i)
  {
    //! @todo: Omnidirectional cameras: Improve check.
    if (p_B.col(i)(2) > 0 && isVisible(rig.at(cam_B).size(), px_B.col(i)))
    {
      ++num_visible;
    }
  }

  return static_cast<real_t>(num_visible) / px_B.cols();
}

} // namespace ze
