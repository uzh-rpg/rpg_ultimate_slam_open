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
#include <ze/common/matrix.hpp>
#include <ze/common/timer.hpp>
#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>
#include <ze/cameras/camera_impl.hpp>
#include <ze/cameras/camera_utils.hpp>
#include <ze/geometry/triangulation.hpp>

namespace ze {

#ifndef ZE_SINGLE_PRECISION_FLOAT
  real_t tol = 1e-10;
#else
  real_t tol = 3e-6;
#endif

std::tuple<Position, TransformationVector, Bearings>
generateObservingCameras()
{
  Transformation T_W_C;
  T_W_C.setRandom(); // Random camera to world transformation.

  PinholeCamera cam = createPinholeCamera(640, 480, 329.11, 329.11, 320.0, 240.0);
  Keypoint px(349, 210);
  Position p_W_true = T_W_C * (cam.backProject(px) * 2.0);

  TransformationVector T_C_W_vec;
  Bearings f_C(3, 10);
  int n = 0;
  std::ranlux24 gen;
  std::normal_distribution<real_t> noise_rot(0, 0.1);
  std::normal_distribution<real_t> noise_pos(0, 0.5);
  for(int i = 0; i < 10; ++i)
  {
    // Perturb pose:
    Vector6 pert;
    pert.head<3>() = Vector3::Constant(noise_pos(gen));
    pert.tail<3>() = Vector3::Constant(noise_rot(gen));
    Transformation T_C_W_perturbed = (T_W_C * Transformation::exp(pert)).inverse();

    Position p_C = T_C_W_perturbed * p_W_true;
    Keypoint px = cam.project(p_C);
    if(isVisible(cam.size(), px))
    {
      T_C_W_vec.push_back(T_C_W_perturbed);
      f_C.col(n++) = p_C.normalized();
    }
  }
  f_C.conservativeResize(3, T_C_W_vec.size());
  CHECK_GE(T_C_W_vec.size(), 2u);
  return std::make_tuple(p_W_true, T_C_W_vec, f_C);
}

} // namespace ze

TEST(TriangulationTests, testSolver)
{
  using namespace ze;

  // Generate data.
  Position p_W_true;
  TransformationVector T_C_W_vec;
  Bearings f_C;
  std::tie(p_W_true, T_C_W_vec, f_C) = ze::generateObservingCameras();

  // Triangulate.
  Vector4 p_W_homogeneous;
  bool success;
  std::tie(p_W_homogeneous, success) = triangulateHomogeneousDLT(T_C_W_vec, f_C);
  Vector3 p_W_estimated = p_W_homogeneous.head<3>() / p_W_homogeneous(3);

  // Compare error.
  EXPECT_LT((p_W_estimated - p_W_true).norm(), tol);
}

TEST(TriangulationTests, testNonlinearRefinement)
{
  using namespace ze;

  // Generate data.
  Position p_W_true;
  TransformationVector T_C_W_vec;
  Bearings f_C;
  std::tie(p_W_true, T_C_W_vec, f_C) = ze::generateObservingCameras();

  // Triangulate.
  Position p_W_estimated = p_W_true + Vector3(0.1, 0.05, 0.1);
  triangulateGaussNewton(T_C_W_vec, f_C, p_W_estimated);

  // Compare error.
  EXPECT_LT((p_W_estimated - p_W_true).norm(), tol);
}

ZE_UNITTEST_ENTRYPOINT
