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

#pragma once

#include <cstdint>
#pragma diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Eigen 3.2.7 uses std::binder1st and std::binder2nd which are deprecated since c++11
// Fix is in 3.3 devel (http://eigen.tuxfamily.org/bz/show_bug.cgi?id=872).
#include <Eigen/Core>
#pragma diagnostic pop
#include <Eigen/StdVector>
#include <ze/common/config.hpp>
#include <dvs_msgs/EventArray.h>

namespace ze {

//------------------------------------------------------------------------------
// Scalars and fp precision.
using size_t    = std::size_t;
using int8_t    = std::int8_t;
using int16_t   = std::int16_t;
using int64_t   = std::int64_t;
using uint8_t   = std::uint8_t;
using uint16_t  = std::uint16_t;
using uint32_t  = uint32_t;
using uint64_t  = std::uint64_t;
#ifdef ZE_SINGLE_PRECISION_FLOAT
using real_t = float;
#else
using real_t = double;
#endif

//------------------------------------------------------------------------------
// Typedefs of commonly used Eigen matrices and vectors.

// MatrixMN, MatrixN = MatrixNN, I_NxN, and Z_NxN, for M,N=1..9.
#define ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(SIZE, SUFFIX)            \
  using Matrix##SUFFIX = Eigen::Matrix<real_t, SIZE, SIZE>; \
  using Matrix1##SUFFIX = Eigen::Matrix<real_t, 1, SIZE>;   \
  using Matrix2##SUFFIX = Eigen::Matrix<real_t, 2, SIZE>;   \
  using Matrix3##SUFFIX = Eigen::Matrix<real_t, 3, SIZE>;   \
  using Matrix4##SUFFIX = Eigen::Matrix<real_t, 4, SIZE>;   \
  using Matrix5##SUFFIX = Eigen::Matrix<real_t, 5, SIZE>;   \
  using Matrix6##SUFFIX = Eigen::Matrix<real_t, 6, SIZE>;   \
  using Matrix7##SUFFIX = Eigen::Matrix<real_t, 7, SIZE>;   \
  using Matrix8##SUFFIX = Eigen::Matrix<real_t, 8, SIZE>;   \
  using Matrix9##SUFFIX = Eigen::Matrix<real_t, 9, SIZE>;   \
  using Matrix##SUFFIX##X = Eigen::Matrix<real_t, SIZE, Eigen::Dynamic>; \
  using MatrixX##SUFFIX = Eigen::Matrix<real_t, Eigen::Dynamic, SIZE>;   \
  static const Eigen::MatrixBase<Matrix##SUFFIX>::IdentityReturnType I_##SUFFIX##x##SUFFIX = Matrix##SUFFIX::Identity(); \
  static const Eigen::MatrixBase<Matrix##SUFFIX>::ConstantReturnType Z_##SUFFIX##x##SUFFIX = Matrix##SUFFIX::Zero()

ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(1,1);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(2,2);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(3,3);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(4,4);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(5,5);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(6,6);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(7,7);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(8,8);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(9,9);

// Typedef arbitary length vector and arbitrary sized matrix.
using VectorX = Eigen::Matrix<real_t, Eigen::Dynamic, 1>;
using MatrixX = Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic>;
using VectorXi = Eigen::VectorXi;

// Commonly used fixed size vectors.
using Vector1 = Eigen::Matrix<real_t, 1, 1>;
using Vector2 = Eigen::Matrix<real_t, 2, 1>;
using Vector3 = Eigen::Matrix<real_t, 3, 1>;
using Vector4 = Eigen::Matrix<real_t, 4, 1>;
using Vector5 = Eigen::Matrix<real_t, 5, 1>;
using Vector6 = Eigen::Matrix<real_t, 6, 1>;
using Vector7 = Eigen::Matrix<real_t, 7, 1>;
using Vector8 = Eigen::Matrix<real_t, 8, 1>;
using Vector9 = Eigen::Matrix<real_t, 9, 1>;
using Vector2i = Eigen::Vector2i;

//------------------------------------------------------------------------------
// Feature containers.
using Keypoint    = Vector2;
using Bearing     = Vector3;
using Position    = Vector3;
using HomPosition = Vector4;
using Gradient    = Vector2;
using Seed        = Vector4;
using LineMeasurement = Vector3;
using Keypoints   = Matrix2X;
using Bearings    = Matrix3X;
using Positions   = Matrix3X;
using HomPositions = Matrix4X;
using Gradients   = Matrix2X;
using Seeds       = Matrix4X;
//! Normal vector on line end-points bearings, as explained in
//! ze_geometry/doc/line_parametrization.pdf
using LineMeasurements = Matrix3X;
using KeypointLevel = int8_t;
using KeypointType  = int8_t;
using KeypointIndex = uint16_t;
using KeypointLevels = Eigen::Matrix<KeypointLevel, Eigen::Dynamic, 1>;
using KeypointTypes  = Eigen::Matrix<KeypointType, Eigen::Dynamic, 1>;
using KeypointAngles = VectorX;
using KeypointScores = VectorX;
using KeypointSizes  = VectorX;
using KeypointIndices  = Eigen::Matrix<KeypointIndex, Eigen::Dynamic, 1>;
using Descriptors = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

//------------------------------------------------------------------------------
// Inertial containers.
using ImuStamps = Eigen::Matrix<int64_t, Eigen::Dynamic, 1>;
using ImuAccGyrContainer = Matrix6X;
// Order: Accelerometer, Gyroscope
using ImuAccGyr = Vector6;


//------------------------------------------------------------------------------
// Event camera containers.
using EventArray = std::vector<dvs_msgs::Event>;
using EventQueue = EventArray;
using EventArrayPtr = std::shared_ptr<EventArray>;
using StampedEventArray = std::pair<int64_t, EventArrayPtr>;
using StampedEventArrays = std::vector<StampedEventArray>;


} // namespace ze
