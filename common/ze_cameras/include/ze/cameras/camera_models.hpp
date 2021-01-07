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

#include <iostream>
#include <cmath>

#ifdef WITH_CUDA
#  include<cuda_runtime_api.h>
#  define CUDA_HOST __host__
#  define CUDA_DEVICE  __device__
#else
#  define CUDA_HOST
#  define CUDA_DEVICE
#endif

namespace ze {

// Pure static camera projection and distortion models, intended to be used in
// both GPU and CPU code. Parameter checking should be performed in interface
// classes.

// Pinhole projection model.
struct PinholeGeometry
{
  template <typename T>
  CUDA_HOST CUDA_DEVICE
  static void project(const T* params, T* px)
  {
    const T fx = params[0];
    const T fy = params[1];
    const T cx = params[2];
    const T cy = params[3];
    px[0] = px[0] * fx + cx;
    px[1] = px[1] * fy + cy;
  }

  template <typename T>
  CUDA_HOST CUDA_DEVICE
  static void backProject(const T* params, T* px)
  {
    const T fx = params[0];
    const T fy = params[1];
    const T cx = params[2];
    const T cy = params[3];
    px[0] = (px[0] - cx) / fx;
    px[1] = (px[1] - cy) / fy;
  }

  template <typename T>
  CUDA_HOST CUDA_DEVICE
  static void dProject_dBearing(const T* bearing, const T* params, T* H)
  {
    const T fx = params[0];
    const T fy = params[1];
    const T z_sq = bearing[2] * bearing[2];
    const T z_inv = 1.0 / bearing[2];
    H[0] = fx * z_inv;
    H[1] = 0;
    H[2] = 0;
    H[3] = fy * z_inv;
    H[4] = - fx * bearing[0] / z_sq;
    H[5] = - fy * bearing[1] / z_sq;
  }
};

enum class DistortionType
{
  No,
  Fov,
  RadTan,
  Equidistant,
};

// -----------------------------------------------------------------------------
// Dummy distortion.
struct NoDistortion
{
  static constexpr DistortionType type = DistortionType::No;

  template <typename T>
  CUDA_HOST CUDA_DEVICE
  static void distort(const T* /*params*/, T* /*px*/, T* jac_colmajor = nullptr)
  {
    if (jac_colmajor)
    {
      T& J_00 = jac_colmajor[0];
      T& J_10 = jac_colmajor[1];
      T& J_01 = jac_colmajor[2];
      T& J_11 = jac_colmajor[3];
      J_00 = 1.0;
      J_01 = 0.0;
      J_10 = 0.0;
      J_11 = 1.0;
    }
  }

  template <typename T>
  CUDA_HOST CUDA_DEVICE
  static void undistort(const T* /*params*/, T* /*px*/)
  {}
};

// -----------------------------------------------------------------------------
// This class implements the FOV distortion model of Deverneay and Faugeras,
// Straight lines have to be straight, 2001. In PTAM this model is called ATAN.
struct FovDistortion
{
  static constexpr DistortionType type = DistortionType::Fov;

  template <typename T>
  CUDA_HOST CUDA_DEVICE
  static void distort(const T* params, T* px, T* jac_colmajor = nullptr)
  {
    const T x = px[0];
    const T y = px[1];
    const T s = params[0];
    const T tan_s_half_x2 = params[1];
    const T rad = std::sqrt(x * x + y * y);
    const T factor = (rad < 0.001) ? 1.0 : std::atan(rad * tan_s_half_x2) / (s * rad);
    px[0] *= factor;
    px[1] *= factor;

    if (jac_colmajor)
    {
      // no common factors
      const T xx = x * x;
      const T yy = y * y;
      const T rad_sq = xx + yy;
      T& J_00 = jac_colmajor[0];
      T& J_10 = jac_colmajor[1];
      T& J_01 = jac_colmajor[2];
      T& J_11 = jac_colmajor[3];
      if(s * s < 1e-5)
      {
        // Distortion parameter very small.
        J_00 = 1.0; J_01 = 0.0;
        J_10 = 0.0; J_11 = 1.0;
      }
      else if(rad_sq < 1e-5)
      {
        // Projection very close to image center
        J_00 = 2.0 * std::tan(s / 2.0) / s;
        J_11 = J_00;
        J_01 = 0.0;
        J_10 = 0.0;
      }
      else
      {
        // Standard case
        const T xy = x * y;
        const T rad = std::sqrt(rad_sq);
        const T nominator = std::atan(tan_s_half_x2 * rad);
        const T scale =
            tan_s_half_x2 / (s * (xx + yy) * (tan_s_half_x2 * tan_s_half_x2 * (xx + yy) + 1.0))
            - factor * 1.0  / rad_sq;
        J_00 = xx * scale + factor;
        J_11 = yy * scale + factor;
        J_01 = xy * scale;
        J_10 = J_01;
      }
    }
  }

  template <typename T>
  CUDA_HOST CUDA_DEVICE
  static void undistort(const T* params, T* px)
  {
    const T s = params[0];
    const T tan_s_half_x2 = params[1];
    const T rad = std::sqrt(px[0] * px[0] + px[1] * px[1]);
    const T factor = (rad < 0.001) ? 1.0 : (std::tan(rad * s) / tan_s_half_x2) / rad;
    px[0] *= factor;
    px[1] *= factor;
  }
};

// -----------------------------------------------------------------------------
// This class implements the radial and tangential distortion model used by
// OpenCV and ROS. Reference:
// docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
struct RadialTangentialDistortion
{
  static constexpr DistortionType type = DistortionType::RadTan;

  template <typename T>
  CUDA_HOST CUDA_DEVICE
  static void distort(const T* params, T* px, T* jac_colmajor = nullptr)
  {
    const T x = px[0];
    const T y = px[1];
    const T k1 = params[0];
    const T k2 = params[1];
    const T p1 = params[2];
    const T p2 = params[3];
    const T xx = x * x;
    const T yy = y * y;
    const T xy = x * y;
    const T r2 = xx + yy;
    const T cdist = (k1 + k2 * r2) * r2;
    px[0] += px[0] * cdist + p1 * 2.0 * xy + p2 * (r2 + 2.0 * xx);
    px[1] += px[1] * cdist + p2 * 2.0 * xy + p1 * (r2 + 2.0 * yy);

    if (jac_colmajor)
    {
      const T k2_r2_x4 = k2 * r2 * 4.0;
      const T cdist_p1 = cdist + 1.0;
      T& J_00 = jac_colmajor[0];
      T& J_10 = jac_colmajor[1];
      T& J_01 = jac_colmajor[2];
      T& J_11 = jac_colmajor[3];
      J_00 = cdist_p1 + k1 * 2.0 * xx + k2_r2_x4 * xx + 2.0 * p1 * y + 6.0 * p2 * x;
      J_11 = cdist_p1 + k1 * 2.0 * yy + k2_r2_x4 * yy + 2.0 * p2 * x + 6.0 * p1 * y;
      J_10 = 2.0 * k1 * xy + k2_r2_x4 * xy + 2.0 * p1 * x + 2.0 * p2 * y;
      J_01 = J_10;
    }
  }

  template <typename T>
  CUDA_HOST CUDA_DEVICE
  static void undistort(const T* params, T* px)
  {
    T jac_colmajor[4];
    T x[2];
    T x_tmp[2];
    x[0] = px[0]; x[1]= px[1];
    for(int i = 0; i < 30; ++i)
    {
      x_tmp[0] = x[0]; x_tmp[1] = x[1];
      distort(params, x_tmp, jac_colmajor);

      const T e_u = px[0] - x_tmp[0];
      const T e_v = px[1] - x_tmp[1];

      const T a = jac_colmajor[0];
      const T b = jac_colmajor[1];
      const T d = jac_colmajor[3];

      // direct gauss newton step
      const T a_sqr = a * a;
      const T b_sqr = b * b;
      const T d_sqr = d * d;
      const T abbd = a * b + b * d;
      const T abbd_sqr = abbd * abbd;
      const T a2b2 = a_sqr + b_sqr;
      const T a2b2_inv = 1.0/a2b2;
      const T adabdb = a_sqr * d_sqr - 2 * a * b_sqr * d + b_sqr * b_sqr;
      const T adabdb_inv = 1.0 / adabdb;
      const T c1 = abbd * adabdb_inv;

      x[0] += e_u * (a * (abbd_sqr * a2b2_inv * adabdb_inv + a2b2_inv) - b * c1) + e_v * (b * (abbd_sqr * a2b2_inv * adabdb_inv + a2b2_inv) - d * c1);
      x[1] += e_u * (-a * c1 + b * a2b2 * adabdb_inv) + e_v * (-b * c1 + d * a2b2 * adabdb_inv);

      if ((e_u * e_u + e_v * e_v) < 1e-8)
      {
        break;
      }
    }

    px[0] = x[0];
    px[1] = x[1];
  }
};

// -----------------------------------------------------------------------------
// This class implements the distortion model described in the paper:
// "A Generic Camera Model and Calibration Method for Conventional, Wide-Angle,
// and Fish-Eye Lenses" by Juho Kannala and Sami S. Brandt, PAMI.
struct EquidistantDistortion
{
  static constexpr DistortionType type = DistortionType::Equidistant;

  template <typename T>
  CUDA_HOST CUDA_DEVICE
  static void distort(const T* params, T* px, T* jac_colmajor = nullptr)
  {
    const T x = px[0];
    const T y = px[1];
    const T k1 = params[0];
    const T k2 = params[1];
    const T k3 = params[2];
    const T k4 = params[3];
    const T r_sqr = x * x + y * y;
    const T r = std::sqrt(r_sqr);
    const T theta = std::atan(r);
    const T theta2 = theta * theta;
    const T theta4 = theta2 * theta2;
    const T theta6 = theta4 * theta2;
    const T theta8 = theta4 * theta4;
    const T thetad = theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
    const T scaling = (r > 1e-8) ? thetad / r : 1.0;
    px[0] *= scaling;
    px[1] *= scaling;

    if (jac_colmajor)
    {
      T& J_00 = jac_colmajor[0];
      T& J_10 = jac_colmajor[1];
      T& J_01 = jac_colmajor[2];
      T& J_11 = jac_colmajor[3];

      if(r < 1e-7)
      {
        J_00 = 1.0; J_01 = 0.0;
        J_10 = 0.0; J_11 = 1.0;
      }
      else
      {
        T xx = x * x;
        T yy = y * y;
        T xy = x * y;
        T theta_inv_r = theta / r;
        T theta_sqr = theta * theta;
        T theta_four = theta_sqr * theta_sqr;

        T t1 = 1.0 / (xx + yy + 1.0);
        T t2 = k1 * theta_sqr
             + k2 * theta_four
             + k3 * theta_four * theta_sqr
             + k4 * (theta_four * theta_four) + 1.0;
        T t3 = t1 * theta_inv_r;

        T offset = t2 * theta_inv_r;
        T scale  = t2 * (t1 / r_sqr - theta_inv_r / r_sqr)
            + theta_inv_r * t3 * (
                  2.0 * k1
                + 4.0 * k2 * theta_sqr
                + 6.0 * k3 * theta_four
                + 8.0 * k4 * theta_four * theta_sqr);

        J_11 = yy * scale + offset;
        J_00 = xx * scale + offset;
        J_01 = xy * scale;
        J_10 = J_01;
      }
    }
  }

  template <typename T>
  CUDA_HOST CUDA_DEVICE
  static void undistort(const T* params, T* px)
  {
    T jac_colmajor[4];
    T x[2];
    T x_tmp[2];
    x[0] = px[0]; x[1]= px[1];
    for(int i = 0; i < 30; ++i)
    {
      x_tmp[0] = x[0]; x_tmp[1] = x[1];
      distort(params, x_tmp, jac_colmajor);

      const T e_u = px[0] - x_tmp[0];
      const T e_v = px[1] - x_tmp[1];

      const T a = jac_colmajor[0];
      const T b = jac_colmajor[1];
      const T d = jac_colmajor[3];

      // direct gauss newton step
      const T a_sqr = a * a;
      const T b_sqr = b * b;
      const T d_sqr = d * d;
      const T abbd = a * b + b * d;
      const T abbd_sqr = abbd * abbd;
      const T a2b2 = a_sqr + b_sqr;
      const T a2b2_inv = 1.0 / a2b2;
      const T adabdb = a_sqr * d_sqr - 2 * a * b_sqr * d + b_sqr * b_sqr;
      const T adabdb_inv = 1.0 / adabdb;
      const T c1 = abbd * adabdb_inv;

      x[0] += e_u * (a * (abbd_sqr * a2b2_inv * adabdb_inv + a2b2_inv) - b * c1) + e_v * (b * (abbd_sqr * a2b2_inv * adabdb_inv + a2b2_inv) - d * c1);
      x[1] += e_u * (-a * c1 + b * a2b2 * adabdb_inv) + e_v * (-b * c1 + d * a2b2 * adabdb_inv);

      if ((e_u * e_u + e_v * e_v) < 1e-8)
      {
        break;
      }
    }

    px[0] = x[0];
    px[1] = x[1];
  }
};

} // namespace ze
