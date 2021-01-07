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

#include <ze/cameras/camera.hpp>
#include <ze/cameras/camera_models.hpp>
#include <ze/cameras/camera_utils.hpp>

namespace ze {

template<class Distortion>
class PinholeProjection : public Camera
{
public:

  static constexpr DistortionType distortion_type = Distortion::type;

  //! Default constructor.
  using Camera::Camera;

  virtual ~PinholeProjection() = default;

  virtual Keypoint project(
      const Eigen::Ref<const Bearing>& bearing) const override
  {
    // Unit coordinates -> distortion -> pinhole, offset and scale.
    Keypoint px = bearing.head<2>() / bearing(2);
    Distortion::distort(this->distortion_params_.data(), px.data());
    PinholeGeometry::project(this->projection_params_.data(), px.data());
    return px;
  }

  virtual std::pair<Keypoint, bool> projectWithCheck(
      const Eigen::Ref<const Position>& pos,
      real_t border_margin) const override
  {
    if (pos[2] < 0.0)
    {
      return std::make_pair(Keypoint(), false);
    }
    Keypoint px = project(pos);
    return std::make_pair(px, isVisibleWithMargin(size(), px, border_margin));
  }

  virtual Bearing backProject(
      const Eigen::Ref<const Keypoint>& px) const override
  {
    Bearing bearing;
    bearing << px(0), px(1), 1.0;
    PinholeGeometry::backProject(this->projection_params_.data(), bearing.data());
    Distortion::undistort(this->distortion_params_.data(), bearing.data());
    return bearing.normalized();
  }

  virtual Matrix23 dProject_dLandmark(
      const Eigen::Ref<const Position>& pos) const override
  {
    Matrix22 J_dist;
    real_t z_inv = 1.0 / pos.z();
    real_t z_inv_sq = z_inv * z_inv;
    Keypoint px_unitplane = pos.head<2>() * z_inv;
    Distortion::distort(
          this->distortion_params_.data(), px_unitplane.data(), J_dist.data());
    const real_t fx = this->projection_params_[0];
    const real_t fy = this->projection_params_[1];
    Matrix23 J;
    J(0, 0) = fx * J_dist(0, 0) * z_inv;
    J(0, 1) = fx * J_dist(0, 1) * z_inv;
    J(0, 2) = -fx * (pos.x() * J_dist(0, 0) + pos.y() * J_dist(0, 1)) * z_inv_sq;
    J(1, 0) = fy * J_dist(1, 0) * z_inv;
    J(1, 1) = fy * J_dist(1, 1) * z_inv;
    J(1, 2) = -fy * (pos.x() * J_dist(1, 0) + pos.y() * J_dist(1, 1)) * z_inv_sq;
    return J;
  }

  std::pair<Keypoint, Matrix23> projectWithJacobian(
        const Eigen::Ref<const Position>& pos) const
  {
    //! @todo: project and jacobian computation do many duplicate things.
    Keypoint px = project(pos);
    Matrix23 J = dProject_dLandmark(pos);
    return std::make_pair(px, J);
  }

  virtual real_t getApproxAnglePerPixel() const override
  {
    //! @todo: Is this correct? And if yes, this is costlty to compute often!
    //!        replace with acos and a dot product between the bearing vectors.
    // abs() because ICL-NUIM has negative focal length.
    return std::atan(1.0 / (2.0 * std::abs(this->projection_params_[0])))
         + std::atan(1.0 / (2.0 * std::abs(this->projection_params_[1])));
  }

  virtual real_t getApproxBearingAngleFromPixelDifference(real_t px_diff) const override
  {
    //! @todo: Is this correct? And if yes, this is costlty to compute often!
    //!        acos and a dot product between the bearing vectors.
    // abs() because ICL-NUIM has negative focal length.
    return std::atan(px_diff / (2.0 * std::abs(this->projection_params_[0])))
         + std::atan(px_diff / (2.0 * std::abs(this->projection_params_[1])));
  }
};

//-----------------------------------------------------------------------------
// Convenience typedefs.
// (sync with explicit template class instantiations at the end of the cpp file)
typedef PinholeProjection<NoDistortion> PinholeCamera;
typedef PinholeProjection<FovDistortion> FovCamera;
typedef PinholeProjection<RadialTangentialDistortion> RadTanCamera;
typedef PinholeProjection<EquidistantDistortion> EquidistantCamera;

//-----------------------------------------------------------------------------
// Convenience factory functions.

inline PinholeCamera createPinholeCamera(
    int width, int height, real_t fx, real_t fy, real_t cx, real_t cy)
{
  return PinholeCamera(width, height, CameraType::Pinhole,
                       (Vector4() << fx, fy, cx, cy).finished(), VectorX());
}

inline FovCamera createFovCamera(
    int width, int height, real_t fx, real_t fy, real_t cx, real_t cy,
    real_t s)
{
  return FovCamera(width, height, CameraType::PinholeFov,
                   (Vector4() << fx, fy, cx, cy).finished(),
                   (Vector1() << s).finished());
}

inline RadTanCamera createRadTanCamera(
    int width, int height, real_t fx, real_t fy, real_t cx, real_t cy,
    real_t k1, real_t k2, real_t r1, real_t r2)
{
  return RadTanCamera(width, height, CameraType::PinholeRadialTangential,
                       (Vector4() << fx, fy, cx, cy).finished(),
                       (Vector4() << k1, k2, r1, r2).finished());
}

inline EquidistantCamera createEquidistantCamera(
    int width, int height, real_t fx, real_t fy, real_t cx, real_t cy,
    real_t k1, real_t k2, real_t k3, real_t k4)
{
  return EquidistantCamera(width, height, CameraType::PinholeEquidistant,
                           (Vector4() << fx, fy, cx, cy).finished(),
                           (Vector4() << k1, k2, k3, k4).finished());
}

inline Camera::Ptr createEquidistantCameraShared(
    int width, int height, real_t fx, real_t fy, real_t cx, real_t cy,
    real_t k1, real_t k2, real_t k3, real_t k4)
{
  return std::make_shared<EquidistantCamera>(
        width, height, CameraType::PinholeEquidistant,
        (Vector4() << fx, fy, cx, cy).finished(),
        (Vector4() << k1, k2, k3, k4).finished());
}

//! Returns camera with some reasonable parameters.
inline PinholeCamera createTestPinholeCamera()
{
  return createPinholeCamera(640, 480, 329.11, 329.11, 320.0, 240.0);
}

} // namespace ze
