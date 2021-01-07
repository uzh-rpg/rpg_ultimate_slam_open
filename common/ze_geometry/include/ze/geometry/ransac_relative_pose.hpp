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

#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>

namespace ze {

// fwd
class Camera;

using BearingsVector =
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

// utility
inline BearingsVector bearingsVectorFromBearings(const Bearings& f)
{
  BearingsVector v;
  v.reserve(f.cols());
  for (int i = 0; i < f.cols(); ++i)
  {
    v.push_back(f.col(i).cast<double>());
  }
  return v;
}

enum class RelativePoseAlgorithm {
  FivePoint,               //!< Nister 5-point relative pose.
  TwoPointTranslationOnly, //!< 2-point relative pose, assumes known rotation between frames.
  TwoPointRotationOnly     //!< 2-point relative pose, assumes no translation between frames.
};

class RansacRelativePose
{
public:
  RansacRelativePose() = delete;

  RansacRelativePose(
      const Camera& cam,
      const real_t& reprojection_threshold_px);

  bool solve(
      const Bearings& f_ref,
      const Bearings& f_cur,
      const RelativePoseAlgorithm method,
      Transformation& T_cur_ref);

  bool solve(
      const BearingsVector& f_ref,
      const BearingsVector& f_cur,
      const RelativePoseAlgorithm method,
      Transformation& T_cur_ref);

  bool solveRelativePose(
      const BearingsVector& f_ref,
      const BearingsVector& f_cur,
      Transformation& T_cur_ref);

  bool solveTranslationOnly(
      const BearingsVector& f_ref,
      const BearingsVector& f_cur,
      Transformation& T_cur_ref);

  bool solveRotationOnly(
      const BearingsVector& f_ref,
      const BearingsVector& f_cur,
      Transformation& T_cur_ref);

  inline uint32_t numIterations() const { return num_iterations_; }

  inline const std::vector<int>& inliers() const { return inliers_; }

  std::vector<int> outliers();

  //! @name: OpenGV settings.
  //! @{
  real_t ogv_threshold_;
  uint32_t ogv_max_iterations_ = 100;
  real_t ogv_init_probability_ = 0.999;
  uint32_t ogv_verbosity_level_ = 0u;
  //! @}

private:
  uint32_t num_measurements_ = 0u;
  uint32_t num_iterations_ = 0u;
  real_t result_probability_;
  std::vector<int> inliers_;
};

} // namespace ze
