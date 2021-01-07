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

#include <string>
#include <memory>
#include <ze/common/logging.hpp>
#pragma diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Eigen 3.2.7 uses std::binder1st and std::binder2nd which are deprecated since c++11
// Fix is in 3.3 devel (http://eigen.tuxfamily.org/bz/show_bug.cgi?id=872).
#include <Eigen/Core>
#pragma diagnostic pop

#include <imp/core/image.hpp>
#include <imp/core/size.hpp>
#include <ze/common/macros.hpp>
#include <ze/common/types.hpp>

namespace ze {

enum class CameraType {
  Pinhole = 0,
  PinholeFov = 1,
  PinholeEquidistant = 2,
  PinholeRadialTangential = 3
};

inline bool isPinholeType(CameraType type)
{
  switch (type)
  {
    case CameraType::Pinhole:
    case CameraType::PinholeFov:
    case CameraType::PinholeEquidistant:
    case CameraType::PinholeRadialTangential:
      return true;
      break;
    default:
      LOG(FATAL) << "Camera type not known.";
      break;
  }
  return false;
}

//! Wrapper class of various camera projection and distortion models.
class Camera
{
public:
  ZE_POINTER_TYPEDEFS(Camera);

  Camera() = delete;

  //! This constructor is used by the derived classes. If you want a camera,
  //! create one of the derived cameras in camera_impl.
  Camera(const uint32_t width, const uint32_t height, const CameraType type,
         const VectorX& projection_params, const VectorX& distortion_params);

  virtual ~Camera() = default;

  //! @name: Projection and back-projection operations. The main use of the camera.
  //! @{
  //! Bearing vector from pixel coordinates. Z-component of return value is 1.0.
  virtual Bearing backProject(const Eigen::Ref<const Keypoint>& px) const = 0;

  //! Computes pixel coordinates from 3D-point.
  virtual Keypoint project(const Eigen::Ref<const Position>& pos) const = 0;

  //! Returns true or false if 3D-point is visible (no occlusion check) and if
  //! visible also returns pixel coordinates.
  virtual std::pair<Keypoint, bool> projectWithCheck(
      const Eigen::Ref<const Position>& pos,
      real_t border_margin = 0.0) const = 0;

  //! Computes pixel coordinates from 3D-point in homogeneous coordinates.
  virtual Keypoint projectHomogeneous(const Eigen::Ref<const HomPosition>& pos_h) const;

  //! Returns true or false if homogeneous 3D-point is visible (no occlusion check)
  //! and if visible also returns the pixel coordinates.
  virtual std::pair<Keypoint, bool> projectHomogeneousWithCheck(
      const Eigen::Ref<const HomPosition>& pos_h,
      real_t border_margin = 0.0) const;

  //! Computes Jacobian of projection w.r.t. bearing vector.
  virtual Matrix23 dProject_dLandmark(const Eigen::Ref<const Position>& pos) const = 0;

  //! Computes Jacobian of homogeneous projection w.r.t. homogeneous landmark position.
  virtual Matrix24 dProjectHomogeneous_dLandmark(
      const Eigen::Ref<const HomPosition>& pos_h) const;

  //! Computes pixel coordinates from 3D point and corresponding Jacobian.
  virtual std::pair<Keypoint, Matrix23> projectWithJacobian(
      const Eigen::Ref<const Position>& pos) const = 0;

  //! Computes pixel coordinates from 3D homogeneous point and corresponding Jacobian.
  virtual std::pair<Keypoint, Matrix24> projectHomogeneousWithJacobian(
      const Eigen::Ref<const HomPosition>& pos_h) const;
  //! @}

  //! @name Block projection and back-projection. Always prefer to avoid cache misses.
  //! @{
  virtual Bearings backProjectVectorized(const Eigen::Ref<const Keypoints>& px_vec) const;
  virtual Keypoints projectVectorized(const Eigen::Ref<const Bearings>& bearing_vec) const;
  virtual Matrix6X dProject_dLandmarkVectorized(const Positions& pos_vec) const;
  //! @}

  //! @name Image dimension.
  //! @{
  inline uint32_t width() const { return size_.width(); }
  inline uint32_t height() const { return size_.height(); }
  inline Size2u size() const { return size_; }
  //! @}

  //! CameraType value representing the camera model used by the derived class.
  inline CameraType type() const { return type_; }

  //! CameraType as descriptive string.
  std::string typeAsString() const;

  //! Camera projection parameters.
  inline const VectorX& projectionParameters() const { return projection_params_; }

  //! Camera distortion parameters.
  inline const VectorX& distortionParameters() const { return distortion_params_; }

  //! Name of the camera.
  inline const std::string& label() const { return label_; }

  //! Set user-specific camera label.
  inline void setLabel(const std::string& label) { label_ = label; }

  //! Get angle corresponding to one pixel in image plane.
  //! @todo: make static cache.
  virtual real_t getApproxAnglePerPixel() const = 0;

  virtual real_t getApproxBearingAngleFromPixelDifference(real_t px_diff) const = 0;

  //! Set mask: 0 = masked, >0 = unmasked.
  void setMask(const Image8uC1::Ptr& mask);

  //! Get mask.
  inline Image8uC1::ConstPtr mask() const { return mask_; }

protected:
  Size2u size_;

  //! Camera projection parameters, e.g., (fx, fy, cx, cy).
  VectorX projection_params_;

  //! Camera distortion parameters, e.g., (k1, k2, r1, r2).
  VectorX distortion_params_;

  std::string label_;
  CameraType type_;
  Image8uC1::Ptr mask_ = nullptr;
};

//! Load a camera rig form a yaml file. Returns a nullptr if the loading fails.
Camera::Ptr cameraFromYaml(const std::string& path);

//! Print camera:
std::ostream& operator<<(std::ostream& out, const Camera& cam);

} // namespace ze
