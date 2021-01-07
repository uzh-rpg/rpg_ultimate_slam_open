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

#include <ze/imu/accelerometer_model.hpp>
#include <ze/imu/gyroscope_model.hpp>
#include <ze/imu/imu_intrinsic_model.hpp>
#include <ze/imu/imu_model.hpp>
#include <ze/imu/imu_noise_model.hpp>
#include <ze/imu/imu_rig.hpp>
#include <ze/imu/imu_yaml_serialization.hpp>
#include <ze/common/types.hpp>
#include <ze/common/yaml_serialization.hpp>

using ze::Matrix3;
using ze::Vector3;
using ze::real_t;

namespace YAML {

//------------------------------------------------------------------------------
// IMU loading.
bool convert<std::shared_ptr<ze::ImuModel>>::decode(
    const Node& node, std::shared_ptr<ze::ImuModel>& imu)
{
  imu.reset();
  try {
    if(!node.IsMap())
    {
      LOG(ERROR) << "Unable to get parse the imu because the node is not a map.";
      return false;
    }

    const YAML::Node gyroscopes = node["gyroscopes"];
    const YAML::Node accelerometers = node["accelerometers"];

    if (!gyroscopes || !accelerometers)
    {
      throw std::runtime_error("Missing Gyroscopes / Accelerometer keys.");
    }

    // get the types
    const YAML::Node g_noise_node = gyroscopes["noise_model"];
    const YAML::Node g_intrinsic_node = gyroscopes["intrinsic_model"];
    const YAML::Node a_noise_node = accelerometers["noise_model"];
    const YAML::Node a_intrinsic_node = accelerometers["intrinsic_model"];

    std::string g_noise_type = extractChild<std::string>(g_noise_node, "type");
    std::string g_intrinsic_type = extractChild<std::string>(g_intrinsic_node, "type");
    std::string a_noise_type = extractChild<std::string>(a_noise_node, "type");
    std::string a_intrinsic_type = extractChild<std::string>(a_intrinsic_node, "type");

    if (!internal::validateNoise(g_noise_type) ||
        !internal::validateNoise(a_noise_type) ||
        !internal::validateIntrinsic(g_intrinsic_type) ||
        !internal::validateIntrinsic(a_intrinsic_type))
    {
      throw std::runtime_error("Invalid Intrinsic or Noise type.");
    }

    ze::ImuNoiseModel::Ptr g_noise = internal::decodeNoise(g_noise_node);
    ze::ImuNoiseModel::Ptr a_noise = internal::decodeNoise(a_noise_node);

    ze::ImuIntrinsicModel::Ptr g_intrinsic_model =
        internal::decodeIntrinsics(g_intrinsic_node);
    ze::ImuIntrinsicModel::Ptr a_intrinsic_model =
        internal::decodeIntrinsics(a_intrinsic_node);

    ze::GyroscopeModel::Ptr gyro =
        std::make_shared<ze::GyroscopeModel>(g_intrinsic_model, g_noise);
    ze::AccelerometerModel::Ptr accel =
        std::make_shared<ze::AccelerometerModel>(a_intrinsic_model, a_noise);

    imu = std::make_shared<ze::ImuModel>(accel, gyro);

    if(node["label"])
    {
      imu->setLabel(node["label"].as<std::string>());
    }
    if(node["id"])
    {
      imu->setId(node["id"].as<std::string>());
    }
  }
  catch(const std::exception& e)
  {
    LOG(ERROR) << "YAML exception during parsing: " << e.what();
    imu.reset();
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
// Validation of types

bool internal::validateNoise(const std::string& value)
{
  if ("white-brownian" == value)
  {
    return true;
  }

  return false;
}

bool internal::validateIntrinsic(const std::string& value)
{
  if ("calibrated" == value ||
      "scale-misalignment" == value ||
      "scale-misalignment-gsensitivity" == value ||
      "scale-misalignment-size-effect" == value)
  {
    return true;
  }

  return false;
}

//------------------------------------------------------------------------------
// Noise Model loading.
std::shared_ptr<ze::ImuNoiseModel> internal::decodeNoise(const Node& node)
{
  if(!node.IsMap())
  {
    LOG(ERROR) << "Unable to get parse the imu because the node is not a map.";
    return std::shared_ptr<ze::ImuNoiseModel>();
  }

  // gyroscopes:
  std::string noise_type = extractChild<std::string>(node, "type");
  if (noise_type == "white-brownian")
  {
    real_t noise_density = extractChild<real_t>(node, "noise_density");
    real_t bandwidth = extractChild<real_t>(node, "bandwidth");
    real_t bias_noise_density = extractChild<real_t>(node, "bias_noise_density");
    return std::make_shared<ze::ImuNoiseWhiteBrownian>(
          noise_density, bandwidth, bias_noise_density);
  }
  else
  {
    throw std::runtime_error("Unsupported Noise Model.");
  }
}

//------------------------------------------------------------------------------
// Intrinsic Model loading.
typename std::shared_ptr<ze::ImuIntrinsicModel> internal::decodeIntrinsics(
    const Node& node)
{
  if(!node.IsMap())
  {
    LOG(ERROR) << "Unable to get parse intrinsic model because the node is not a map.";
    return std::shared_ptr<ze::ImuIntrinsicModel>();
  }

  // gyroscopes:
  std::string type = extractChild<std::string>(node, "type");

  if (type == "calibrated")
  {
    return std::make_shared<ze::ImuIntrinsicModelCalibrated>();
  }
  else if (type == "scale-misalignment" ||
           type == "scale-misalignment-gsensitivity" ||
           type == "scale-misalignment-size-effect")
  {
    real_t delay = extractChild<real_t>(node, "delay");
    real_t range = extractChild<real_t>(node, "range");
    Vector3 b = extractChild<Vector3>(node, "b");
    Matrix3 M = extractChild<Matrix3>(node, "M");

    if ("scale-misalignment" == type)
    {
      return std::make_shared<ze::ImuIntrinsicModelScaleMisalignment>(
                         delay, range, b, M);
    }
    else if ("scale-misalignment-gsensitivity" == type)
    {
      Matrix3 Ma = extractChild<Matrix3>(node, "Ma");
      return std::make_shared<
                       ze::ImuIntrinsicModelScaleMisalignmentGSensitivity>(
                         delay, range, b, M, Ma);
    }
    else if ("scale-misalignment-size-effect" == type)
    {
      Matrix3 R = extractChild<Matrix3>(node, "R");
      return std::make_shared<
                       ze::ImuIntrinsicModelScaleMisalignmentSizeEffect>(
                         delay, range, b, M, R);
    }
  }
  else
  {
    throw std::runtime_error("Unsupported Intrinsic Model.");
  }

  return nullptr;
}

//------------------------------------------------------------------------------
// ImuRig loading.
bool convert<std::shared_ptr<ze::ImuRig>>::decode(
    const Node& node, std::shared_ptr<ze::ImuRig>& imu_rig)
{
  imu_rig.reset();
  try {
    if (!node.IsMap())
    {
      LOG(ERROR) << "Parsing ImuRig failed because node is not a map.";
      return false;
    }

    std::string label = extractChild<std::string>(node, "label");

    const Node& imus_node = node["imus"];
    if (!imus_node.IsSequence())
    {
      LOG(ERROR) << "Parsing ImuRig failed because 'imus' is not a sequence.";
      return false;
    }

    size_t num_imus = imus_node.size();
    if (num_imus == 0)
    {
      LOG(ERROR) << "Parsing ImuRig failed. Number of imus is 0.";
      return false;
    }

    ze::TransformationVector T_B_Si;
    ze::ImuVector imus;
    for (size_t i = 0; i < num_imus; ++i)
    {
      const Node& imu_node = imus_node[i];
      if (!imu_node)
      {
        LOG(ERROR) << "Unable to get imu node for imu " << i;
        return false;
      }
      if (!imu_node.IsMap())
      {
        LOG(ERROR) << "Imu node for imu " << i << " is not a map.";
        return false;
      }

      ze::ImuModel::Ptr imu = extractChild<ze::ImuModel::Ptr>(imu_node, "imu");

      ze::Matrix4 T_B_S = extractChild<ze::Matrix4>(imu_node, "T_B_S");

      imus.push_back(imu);
      T_B_Si.push_back(ze::Transformation(T_B_S).inverse());
    }

    imu_rig.reset(new ze::ImuRig(T_B_Si, imus, label));
  }
  catch (const std::exception& ex)
  {
    LOG(ERROR) << "YAML exception during parsing: " << ex.what();
    imu_rig.reset();
    return false;
  }
  return true;
}

} // namespace YAML
