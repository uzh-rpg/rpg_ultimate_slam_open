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

#include <ze/imu/imu_intrinsic_model.hpp>
#include <ze/imu/imu_noise_model.hpp>
#include <ze/common/macros.hpp>
#include <ze/common/types.hpp>
namespace ze {

//! Gyroscope Model
class GyroscopeModel
{
public:
  ZE_POINTER_TYPEDEFS(GyroscopeModel);

  typedef VectorX measurement_t;

  GyroscopeModel() = delete;

  GyroscopeModel(ImuIntrinsicModel::Ptr intrinsicModel,
                 ImuNoiseModel::Ptr noiseModel);

  //! These models may depend on both angular and linear quantities as well as
  //! higher order time derivatives of the quantities. A measurement is
  //! composed exclusively of either angular or linear quantities and features
  //! time derivatives in increasing order starting from 0.
  Vector3 distort(const Eigen::Ref<const measurement_t>& w,
                  const Eigen::Ref<const measurement_t>& a) const;
  Vector3 undistort(const Eigen::Ref<const measurement_t>& w,
                    const Eigen::Ref<const measurement_t>& a) const;

  // getters
  inline const ImuNoiseModel::Ptr noiseModel() const { return noiseModel_; }
  inline const ImuIntrinsicModel::Ptr intrinsicModel() const { return intrinsicModel_; }

private:
  const ImuIntrinsicModel::Ptr intrinsicModel_;
  const ImuNoiseModel::Ptr noiseModel_;
};

} // namespace ze
