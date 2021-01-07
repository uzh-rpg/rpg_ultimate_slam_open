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

#include <ze/common/transformation.hpp>
#include <ze/common/types.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <ze/common/matrix.hpp>

namespace ze {

// ----------------------------------------------------------------------------
//! Compute essential matrix from given camera transformation
inline Matrix3 essentialMatrix(const Transformation& T)
{
  return skewSymmetric(T.getPosition()) * T.getRotationMatrix();
}

// ----------------------------------------------------------------------------
//! Compute fundamental matrix from given camera transformation
inline Matrix3 fundamentalMatrix(const Transformation& T_cam0_cam1,
                                 const VectorX& projection_parameters0,
                                 const VectorX& projection_parameters1)
{
  CHECK_EQ(projection_parameters0.size(), 4u);
  CHECK_EQ(projection_parameters1.size(), 4u);

  Matrix3 K0, K1;
  K0 << projection_parameters0(0), 0, projection_parameters0(2),
      0, projection_parameters0(1), projection_parameters0(3),
      0, 0, 1;
  K1 << projection_parameters1(0), 0, projection_parameters1(2),
      0, projection_parameters1(1), projection_parameters1(3),
      0, 0, 1;

  return (K0.inverse().transpose() * essentialMatrix(T_cam0_cam1) * K1.inverse());
}

// ----------------------------------------------------------------------------
//! Stereo Rectification

using Rect = Roi<real_t, 2>;

//! \brief Compute inner and outer rectangles.
//!
//! The inner rectangle is inscribed in the undistorted-rectified image.
//! The outer rectangle is circumscribed about the undistorted-rectified image.
//! \param img_size The size of the original (distorted) image.
//! \param camera_parameters Vector of intrinsic parameters [fx, fy, cx, cy]
//! for the original image.
//! \param transformed_camera_parameters Vector of intrinsic parameters [fx', fy', cx', cy']
//! for the undistorted-rectified image.
//! \param distortion_coefficients Vector of distortion coefficients.
//! \param H Rectifying homography matrix.
//! \return A pair containing the rectangles inner (first) and outer (second).
template<typename CameraModel,
         typename DistortionModel>
inline std::pair<Rect, Rect> innerAndOuterRectangles(
    const Size2u& img_size,
    const Vector4& camera_parameters,
    const Vector4& transformed_camera_parameters,
    const Vector4& distortion_coefficients,
    const Matrix3& H)
{
  //! Sample the image in num_pts*num_pts locations
  constexpr int num_pts{9}; //!< Number of sampling point for each image dimension
  Matrix3X pts(3, num_pts*num_pts);     //!< Sampling points

  for (int y = 0, k = 0; y < num_pts; ++y)
  {
    for (int x = 0; x < num_pts; ++x)
    {
      pts.col(k++) =
          Vector3(
            static_cast<real_t>(x) *
            static_cast<real_t>(img_size[0]) /
          static_cast<real_t>(num_pts -1),
          static_cast<real_t>(y) *
          static_cast<real_t>(img_size[1]) /
          static_cast<real_t>(num_pts - 1),
          1);
    }
  }

  //! For every sampling point (u,v) compute the corresponding (u', v')
  //! in the undistorted-rectified image:
  //! x" = (u - cx)/fx
  //! y" = (v - cy)/fy
  //! (x', y') = undistort(distortion_coefficients, (x", y"))
  //! [X Y W]^T = H*[x' y' 1]^T
  //! x = X / W, y = Y / W
  //! u' = x * fx' + cx'
  //! v' = y * fy' + cy'
  for (int c = 0; c < num_pts * num_pts; ++c)
  {
    CameraModel::backProject(camera_parameters.data(),
                             pts.col(c).data());
    DistortionModel::undistort(distortion_coefficients.data(),
                               pts.col(c).data());
    pts.col(c) = H * pts.col(c);
    pts.col(c) /= pts.col(c)(2);
    CameraModel::project(transformed_camera_parameters.data(),
                         pts.col(c).data());
  }

  //! Rectangles are specified by two points.
  real_t inner_x_left{-std::numeric_limits<real_t>::max()};
  real_t inner_x_right{std::numeric_limits<real_t>::max()};
  real_t inner_y_top{-std::numeric_limits<real_t>::max()};
  real_t inner_y_bottom{std::numeric_limits<real_t>::max()};
  real_t outer_x_left{std::numeric_limits<real_t>::max()};
  real_t outer_x_right{-std::numeric_limits<real_t>::max()};
  real_t outer_y_top{std::numeric_limits<real_t>::max()};
  real_t outer_y_bottom{-std::numeric_limits<real_t>::max()};

  //! Iterate over the sampling points and adjust the rectangle bounds.
  for (int y = 0, k = 0; y < num_pts; y++)
  {
    for (int x = 0; x < num_pts; x++)
    {
      const Vector3& pt = pts.col(k++);
      outer_x_left = std::min(outer_x_left, pt.x());
      outer_x_right = std::max(outer_x_right, pt.x());
      outer_y_top = std::min(outer_y_top, pt.y());
      outer_y_bottom = std::max(outer_y_bottom, pt.y());

      if (x == 0)
      {
        inner_x_left = std::max(inner_x_left, pt.x());
      }
      if (x == num_pts - 1)
      {
        inner_x_right = std::min(inner_x_right, pt.x());
      }
      if (y == 0)
      {
        inner_y_top = std::max(inner_y_top, pt.y());
      }
      if (y == num_pts - 1)
      {
        inner_y_bottom = std::min(inner_y_bottom, pt.y());
      }
    }
  }

  //! Compute and return the rectangles.
  Rect inner(inner_x_left, inner_y_top,
             inner_x_right - inner_x_left,
             inner_y_bottom - inner_y_top);
  Rect outer(outer_x_left, outer_y_top,
             outer_x_right - outer_x_left,
             outer_y_bottom - outer_y_top);

  return std::pair<Rect, Rect>(inner, outer);
}

//!\brief Compute rectification parameters for a horizontal stereo pair
//!
//! The function is specific for the horizontal-stereo case.
//! \param img_size The size of the original (distorted) image.
//! \param cam0_parameters Vector of intrinsic parameters [fx, fy, cx, cy]
//! for the 0-th camera of the stereo pair.
//! \param cam0_distortion_coefficients Vector of distortion coefficients
//! for the 0-th camera of the stereo pair.
//! \param cam1_parameters Vector of intrinsic parameters [fx, fy, cx, cy]
//! for the 1st camera of the stereo pair.
//! \param cam1_distortion_coefficients Vector of distortion coefficients
//! for the 1st camera of the stereo pair.
//! \param T_cam0_cam1 Stereo extrinsic parameters, i.e. the transformation right-to-left.
//! \return cam0_H rectifying homography for the 0-th camera.
//! \return cam1_H rectifying homography for the 1st camera.
//! \return transformed_cam0_parameters Output transformed parameters for the 0-th camera.
//! \return transformed_cam1_parameters Output transformed parameters for the 1st camera.
//! \return horizontal_offset Output displacement for the rectified stereo pair.
template<typename CameraModel,
         typename DistortionModel>
inline std::tuple<Matrix3, Matrix3, Vector4, Vector4, real_t>
computeHorizontalStereoParameters(const Size2u& img_size,
                                  const Vector4& cam0_parameters,
                                  const Vector4& cam0_distortion_coefficients,
                                  const Vector4& cam1_parameters,
                                  const Vector4& cam1_distortion_coefficients,
                                  const Transformation& T_cam0_cam1)
{
  //! Compute the recification homographies as in
  //! Trucco, Verry: Introductory techniques for 3D computer vision,
  //! Prentice Hall 1998, page 160.
  const Quaternion avg_rotation =
      Quaternion::exp(-0.5*Quaternion::log(T_cam0_cam1.getRotation()));
  const Vector3 transformed_t = avg_rotation.rotate(-T_cam0_cam1.getPosition());
  const Vector3 e1 = transformed_t / transformed_t.norm();
  Vector3 e2(-transformed_t(1), transformed_t(0), 0);
  e2 = e2 / e2.norm();
  const Vector3 e3 = e1.cross(e2);
  Matrix3 rect_R;
  rect_R.row(0) = e1.transpose();
  rect_R.row(1) = e2.transpose();
  rect_R.row(2) = e3.transpose();

  //! Rotate both cameras according to the average rotation.
  const Matrix3 cam0_H = rect_R * avg_rotation.getRotationMatrix().transpose();
  const Matrix3 cam1_H = rect_R * avg_rotation.getRotationMatrix();

  //! The images rectified according to cam0_H and cam1_H will not be contained
  //! in the same region of the image plane as the original image.
  //! Here we alter the focal lengths and the principal points to keep
  //! all points within the original image size.
  const Vector4* const camera_parameter_ptrs[2] = {&cam0_parameters,
                                                   &cam1_parameters};
  const Vector4* const distortion_coefficient_ptrs[2] = {&cam0_distortion_coefficients,
                                                         &cam1_distortion_coefficients};
  const Matrix3* const homography_ptrs[2] = {&cam0_H,
                                             &cam1_H};

  const real_t nx = img_size[0];
  const real_t ny = img_size[1];

  real_t transformed_focal_length = std::numeric_limits<real_t>::max();
  for (int8_t i = 0; i < 2; ++i)
  {
    const Vector4& camera_parameters = *camera_parameter_ptrs[i];
    const Vector4& distortion_coefficients = *distortion_coefficient_ptrs[i];
    real_t focal_length = camera_parameters(1);
    if (distortion_coefficients(0) < 0)
    {
      focal_length *= 1 + distortion_coefficients(0)*
          (nx * nx + ny * ny) /
          (4 * focal_length * focal_length);
    }
    transformed_focal_length = std::min(transformed_focal_length, focal_length);
  }

  Matrix22 transformed_principal_point;
  for (int8_t i = 0; i < 2; ++i)
  {
    const Vector4& camera_parameters = *camera_parameter_ptrs[i];
    const Vector4& distortion_coefficients = *distortion_coefficient_ptrs[i];
    const Matrix3& H = *homography_ptrs[i];

    Matrix34 img_corners;
    img_corners << 0, nx, nx, 0,
        0, 0, ny, ny,
        1, 1, 1, 1;

    Vector4 temp_cam_params;
    temp_cam_params << transformed_focal_length, transformed_focal_length, 0, 0;
    for (int8_t c = 0; c < 4; ++c)
    {
      CameraModel::backProject(camera_parameters.data(), img_corners.col(c).data());
      DistortionModel::undistort(distortion_coefficients.data(), img_corners.col(c).data());
      img_corners.col(c) = H * img_corners.col(c);
      img_corners.col(c) /= img_corners.col(c)(2);
      CameraModel::project(temp_cam_params.data(), img_corners.col(c).data());
    }
    transformed_principal_point.col(i) = Vector2((nx - 1) / 2, (ny - 1) / 2);
    transformed_principal_point.col(i) -= img_corners.block(0, 0, 2, 4).rowwise().mean();
  }
  transformed_principal_point.col(0) =
      transformed_principal_point.col(1) =
      transformed_principal_point.rowwise().mean();

  Vector4 transformed_cam0_parameters;
  Vector4 transformed_cam1_parameters;
  transformed_cam0_parameters << transformed_focal_length,
      transformed_focal_length,
      transformed_principal_point(0, 0),
      transformed_principal_point(1, 0);
  transformed_cam1_parameters << transformed_focal_length,
      transformed_focal_length,
      transformed_principal_point(0, 1),
      transformed_principal_point(1, 1);

  std::pair<Rect, Rect> cam0_rects =
      innerAndOuterRectangles<CameraModel, DistortionModel>(
        img_size, cam0_parameters,
        transformed_cam0_parameters,
        cam0_distortion_coefficients,
        cam0_H);

  std::pair<Rect, Rect> cam1_rects =
      innerAndOuterRectangles<CameraModel, DistortionModel>(
        img_size, cam1_parameters,
        transformed_cam1_parameters,
        cam1_distortion_coefficients,
        cam1_H);

  //! Determine s0, i.e. the scaling factor based on inner rectangles from both images.
  //! @todo (MPI) support different scales in [0, 1].
  //! Currently only scale = 0 is supported (s0).
  //! Camera 0 image
  real_t s0 = std::max(
        transformed_principal_point(0, 0) / (transformed_principal_point(0, 0) - cam0_rects.first.x()),
        transformed_principal_point(1, 0) / (transformed_principal_point(1, 0) - cam0_rects.first.y()));

  s0 = std::max(s0,
                (nx - transformed_principal_point(0, 0)) /
                (cam0_rects.first.x() + cam0_rects.first.width() - transformed_principal_point(0, 0)));

  s0 = std::max(s0,
                (ny - transformed_principal_point(1, 0)) /
                (cam0_rects.first.y() + cam0_rects.first.height() - transformed_principal_point(1, 0)));

  //! Camera 1 image
  s0 = std::max(
        transformed_principal_point(0, 1) / (transformed_principal_point(0, 1) - cam1_rects.first.x()),
        transformed_principal_point(1, 1) / (transformed_principal_point(1, 1) - cam1_rects.first.y()));

  s0 = std::max(s0,
                (nx - transformed_principal_point(0, 1)) /
                (cam1_rects.first.x() + cam1_rects.first.width() - transformed_principal_point(0, 1)));

  s0 = std::max(s0,
                (ny - transformed_principal_point(1, 1)) /
                (cam1_rects.first.y() + cam1_rects.first.height() - transformed_principal_point(1, 1)));

  transformed_cam0_parameters(0) =
      transformed_cam0_parameters(1) =
      transformed_cam1_parameters(0) =
      transformed_cam1_parameters(1) = transformed_focal_length * s0;

  const real_t horizontal_offset = transformed_t(0) * s0;

  return std::make_tuple(cam0_H,
                         cam1_H,
                         transformed_cam0_parameters,
                         transformed_cam1_parameters,
                         horizontal_offset);
}

} // namespace ze
