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

#include <ze/cameras/camera.hpp>

#include <string>
#include <ze/cameras/camera_yaml_serialization.hpp>

namespace ze {

Camera::Camera(const uint32_t width, const uint32_t height, const CameraType type,
               const VectorX& projection_params, const VectorX& distortion_params)
  : size_(width, height)
  , projection_params_(projection_params)
  , distortion_params_(distortion_params)
  , type_(type)
{
  CHECK_EQ(projection_params_.size(), 4);
  switch (type_)
  {
    case CameraType::Pinhole:
      CHECK_EQ(distortion_params_.size(), 0);
      break;
    case CameraType::PinholeRadialTangential:
      CHECK_EQ(distortion_params_.size(), 4);
      break;
    case CameraType::PinholeEquidistant:
      CHECK_EQ(distortion_params_.size(), 4);
      break;
    case CameraType::PinholeFov:
    {
      CHECK_EQ(distortion_params_.size(), 1);
      // Pre-computations for improved speed.
      const real_t s = distortion_params_(0);
      const real_t tan_s_half_x2 = std::tan(s / 2.0) * 2.0;
      distortion_params_ = Vector2(s, tan_s_half_x2);
      break;
    }
    default:
      LOG(FATAL) << "Unknown camera model.";
      break;
  }
}

Keypoint Camera::projectHomogeneous(
    const Eigen::Ref<const HomPosition>& pos_h) const
{
  if (pos_h[3] < 0.0)
  {
    return this->project(-pos_h.head<3>());
  }
  else
  {
    return this->project(pos_h.head<3>());
  }
}

std::pair<Keypoint, bool> Camera::projectHomogeneousWithCheck(
    const Eigen::Ref<const HomPosition>& pos_h,
    real_t border_margin) const
{
  if (pos_h[3] < 0.0)
  {
    return this->projectWithCheck(-pos_h.head<3>(), border_margin);
  }
  else
  {
    return this->projectWithCheck(pos_h.head<3>(), border_margin);
  }
}

Matrix24 Camera::dProjectHomogeneous_dLandmark(
    const Eigen::Ref<const HomPosition>& pos_h) const
{
  Matrix24 J;
  if (pos_h[3] < 0.0)
  {
    J.topLeftCorner<2, 3>() = this->dProject_dLandmark(-pos_h.head<3>());
  }
  else
  {
    J.topLeftCorner<2, 3>() = this->dProject_dLandmark(pos_h.head<3>());
  }
  J.bottomRightCorner<2, 1>().setZero();
  return J;
}

std::pair<Keypoint, Matrix24> Camera::projectHomogeneousWithJacobian(
    const Eigen::Ref<const HomPosition>& pos_h) const
{
  return std::make_pair(projectHomogeneous(pos_h),
                        dProjectHomogeneous_dLandmark(pos_h));
}

Bearings Camera::backProjectVectorized(const Eigen::Ref<const Keypoints>& px_vec) const
{
  Bearings bearings(3, px_vec.cols());
  for(int i = 0; i < px_vec.cols(); ++i)
  {
    bearings.col(i) = this->backProject(px_vec.col(i));
  }
  return bearings;
}

Keypoints Camera::projectVectorized(const Eigen::Ref<const Bearings>& bearing_vec) const
{
  Keypoints px_vec(2, bearing_vec.cols());
  for(int i = 0; i < bearing_vec.cols(); ++i)
  {
    px_vec.col(i) = this->project(bearing_vec.col(i));
  }
  return px_vec;
}

Matrix6X Camera::dProject_dLandmarkVectorized(const Positions& pos_vec) const
{
  Matrix6X J_vec(6, pos_vec.cols());
  for(int i = 0; i < pos_vec.cols(); ++i)
  {
    J_vec.col(i) =
        Eigen::Map<Matrix61>(this->dProject_dLandmark(pos_vec.col(i)).data());
  }
  return J_vec;
}

std::string Camera::typeAsString() const
{
  switch (type_)
  {
    case CameraType::Pinhole: return "Pinhole";
    case CameraType::PinholeFov: return "PinholeFov";
    case CameraType::PinholeEquidistant: return "PinholeEquidistant";
    case CameraType::PinholeRadialTangential: return "PinholeRadialTangential";
    default:
      LOG(FATAL) << "Unknown parameter type";
  }
  return "";
}

void Camera::setMask(const Image8uC1::Ptr& mask)
{
  CHECK_NOTNULL(mask.get());
  CHECK_EQ(mask->size(), size_);
  mask_ = mask;
}

Camera::Ptr cameraFromYaml(const std::string& path)
{
  try
  {
    YAML::Node doc = YAML::LoadFile(path.c_str());
    return doc.as<Camera::Ptr>();
  }
  catch (const std::exception& ex)
  {
    LOG(ERROR) << "Failed to load Camera from file " << path << " with the error: \n"
               << ex.what();
  }
  return Camera::Ptr();
}

std::ostream& operator<<(std::ostream& out, const Camera& cam)
{
  out << "    Label = " << cam.label() << "\n"
      << "    Model = " << cam.typeAsString() << "\n"
      << "    Dimensions = " << cam.width() << "x" << cam.height() << "\n"
      << "    Proj. parameters = " << cam.projectionParameters().transpose() << "\n"
      << "    Dist. parameters = " << cam.distortionParameters().transpose() << "\n"
      << "    Masked = " << (cam.mask() ? "True" : "False");
  return out;
}

} // namespace ze
