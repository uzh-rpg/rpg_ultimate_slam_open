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

#include <ze/geometry/ransac_relative_pose.hpp>

#include <glog/logging.h>

#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/TranslationOnlySacProblem.hpp>
#include <opengv/sac_problems/relative_pose/RotationOnlySacProblem.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/triangulation/methods.hpp>

#include <ze/cameras/camera.hpp>
#include <ze/common/combinatorics.hpp>

namespace ze {

RansacRelativePose::RansacRelativePose(
    const Camera& cam,
    const real_t& reprojection_threshold_px)
  : ogv_threshold_(
      1.0 - std::cos(cam.getApproxAnglePerPixel() * reprojection_threshold_px))
{
  VLOG(3) << "RANSAC THRESHOLD = " << cam.getApproxAnglePerPixel() * reprojection_threshold_px;
}

bool RansacRelativePose::solve(
      const Bearings& f_ref,
      const Bearings& f_cur,
      const RelativePoseAlgorithm method,
      Transformation& T_cur_ref)
{
  BearingsVector f_ref_v = bearingsVectorFromBearings(f_ref);
  BearingsVector f_cur_v = bearingsVectorFromBearings(f_cur);
  return solve(f_ref_v, f_cur_v, method, T_cur_ref);
}

bool RansacRelativePose::solve(
    const BearingsVector& f_ref,
    const BearingsVector& f_cur,
    const RelativePoseAlgorithm method,
    Transformation& T_cur_ref)
{
  CHECK_EQ(f_ref.size(), f_cur.size());
  switch(method)
  {
    case RelativePoseAlgorithm::FivePoint:
      return solveRelativePose(f_ref, f_cur, T_cur_ref);
      break;
    case RelativePoseAlgorithm::TwoPointTranslationOnly:
      return solveTranslationOnly(f_ref, f_cur, T_cur_ref);
      break;
    case RelativePoseAlgorithm::TwoPointRotationOnly:
      return solveRotationOnly(f_ref, f_cur, T_cur_ref);
      break;
    default:
      LOG(FATAL) << "Algorithm not implemented";
      break;
  }
  return false;
}

// -----------------------------------------------------------------------------
bool RansacRelativePose::solveRelativePose(
    const BearingsVector& f_ref,
    const BearingsVector& f_cur,
    Transformation& T_cur_ref)
{
  // Setup problem.
  //! @todo: Unify all the repetitive code.
  using Problem = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
  using Adapter = opengv::relative_pose::CentralRelativeAdapter;
  Adapter adapter(f_cur, f_ref);
  std::shared_ptr<Problem> problem(new Problem(adapter, Problem::NISTER));
  opengv::sac::Ransac<Problem> ransac;
  ransac.sac_model_ = problem;
  ransac.threshold_ = ogv_threshold_;
  ransac.max_iterations_ = ogv_max_iterations_;
  ransac.probability_ = ogv_init_probability_;

  // Solve.
  if(!ransac.computeModel(ogv_verbosity_level_))
  {
    VLOG(1) << "5Pt RANSAC could not find a solution";
    return false;
  }
  VLOG(10) << "5Pt RANSAC:"
           << ", #iter = " << ransac.iterations_
           << ", #inliers = " << ransac.inliers_.size();

  // Process results.
  Matrix3 R = ransac.model_coefficients_.leftCols<3>().cast<real_t>();
  Vector3 t = ransac.model_coefficients_.rightCols<1>().cast<real_t>();
  T_cur_ref = Transformation(Eigen::Quaternion<real_t>(R).normalized(), t);

  result_probability_ = ransac.probability_;
  num_iterations_ = ransac.iterations_;
  inliers_ = ransac.inliers_;
  num_measurements_ = f_ref.size();
  return true;
}

// -----------------------------------------------------------------------------
bool RansacRelativePose::solveTranslationOnly(
    const BearingsVector& f_ref,
    const BearingsVector& f_cur,
    Transformation& T_cur_ref)
{
  // Setup Problem.
  //! @todo: Unify all the repetitive code.
  using Problem = opengv::sac_problems::relative_pose::TranslationOnlySacProblem;
  using Adapter = opengv::relative_pose::CentralRelativeAdapter;
  Adapter adapter(f_cur, f_ref, T_cur_ref.getRotationMatrix().cast<double>());
  std::shared_ptr<Problem> problem(new Problem(adapter));
  opengv::sac::Ransac<Problem> ransac;
  ransac.sac_model_ = problem;
  ransac.threshold_ = ogv_threshold_;
  ransac.max_iterations_ = ogv_max_iterations_;
  ransac.probability_ = ogv_init_probability_;

  // Solve.
  if(!ransac.computeModel(ogv_verbosity_level_))
  {
    VLOG(1) << "2Pt RANSAC could not find a solution";
    return false;
  }
  VLOG(10) << "2pt RANSAC:"
           << ", #iter = " << ransac.iterations_
           << ", #inliers = " << ransac.inliers_.size();

  // Process results.
  Matrix3 R = ransac.model_coefficients_.leftCols<3>().cast<real_t>();
  Vector3 t = ransac.model_coefficients_.rightCols<1>().cast<real_t>();
  T_cur_ref = Transformation(Eigen::Quaternion<real_t>(R).normalized(), t);

  result_probability_ = ransac.probability_;
  num_iterations_ = ransac.iterations_;
  inliers_ = ransac.inliers_;
  num_measurements_ = f_ref.size();
  return true;
}

// -----------------------------------------------------------------------------
bool RansacRelativePose::solveRotationOnly(
    const BearingsVector& f_ref,
    const BearingsVector& f_cur,
    Transformation& T_cur_ref)
{
  // Setup Problem.
  //! @todo: Unify all the repetitive code.
  using Problem = opengv::sac_problems::relative_pose::RotationOnlySacProblem;
  opengv::relative_pose::CentralRelativeAdapter adapter(f_cur, f_ref);
  std::shared_ptr<Problem> problem(new Problem(adapter));
  opengv::sac::Ransac<Problem> ransac;
  ransac.sac_model_ = problem;
  ransac.threshold_ = ogv_threshold_;
  ransac.max_iterations_ = ogv_max_iterations_;
  ransac.probability_ = ogv_init_probability_;

  // Solve.
  if(!ransac.computeModel(ogv_verbosity_level_))
  {
    VLOG(1) << "2Pt RANSAC could not find a solution";
    return false;
  }
  VLOG(10) << "2pt Rotation-Only RANSAC:"
           << ", #iter = " << ransac.iterations_
           << ", #inliers = " << ransac.inliers_.size();

  // Process results.
  Matrix3 R = ransac.model_coefficients_.leftCols<3>().cast<real_t>();
  Vector3 t = ransac.model_coefficients_.rightCols<1>().cast<real_t>();
  T_cur_ref = Transformation(Eigen::Quaternion<real_t>(R).normalized(), t);

  result_probability_ = ransac.probability_;
  num_iterations_ = ransac.iterations_;
  inliers_ = ransac.inliers_;
  num_measurements_ = f_ref.size();
  return true;
}

// -----------------------------------------------------------------------------
std::vector<int> RansacRelativePose::outliers()
{
  return getOutlierIndicesFromInlierIndices<int>(inliers_, num_measurements_);
}

} // namespace ze
