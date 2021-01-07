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

#include <string>

#include <ze/imu/accelerometer_model.hpp>
#include <ze/imu/gyroscope_model.hpp>
#include <ze/imu/imu_yaml_serialization.hpp>
#include <ze/common/macros.hpp>
#include <ze/common/types.hpp>

namespace ze {

//! Imu Model
class ImuModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuModel);

  typedef VectorX measurement_t;

  ImuModel() = delete;

  ImuModel(const AccelerometerModel::Ptr accelerometer,
           const GyroscopeModel::Ptr gyroscope);

  //! Load an imu from a yaml file. Returns a nullptr if the loading fails.
  static Ptr loadFromYaml(const std::string& path);

  void setLabel(const std::string& label) { label_ = label; }
  void setId(const std::string& id) { id_ = id; }
  std::string label() const { return label_; }
  std::string id() const { return id_; }

  Vector6 distort(const Eigen::Ref<const measurement_t>& primary,
                  const Eigen::Ref<const measurement_t>& secondary) const;
  Vector6 undistort(const Eigen::Ref<const measurement_t>& primary,
                    const Eigen::Ref<const measurement_t>& secondary) const;

  // getters
  inline const AccelerometerModel::Ptr accelerometerModel() const
  {
    return accelerometerModel_;
  }
  inline const GyroscopeModel::Ptr gyroscopeModel() const
  {
    return gyroscopeModel_;
  }

private:
  std::string id_;
  std::string label_;

  const AccelerometerModel::Ptr accelerometerModel_;
  const GyroscopeModel::Ptr gyroscopeModel_;
};

} // namespace ze
