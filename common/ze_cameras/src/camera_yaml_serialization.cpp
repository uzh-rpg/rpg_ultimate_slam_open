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

#include <ze/cameras/camera_yaml_serialization.hpp>
#include <ze/cameras/camera_impl.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <ze/common/yaml_serialization.hpp>
#include <ze/common/types.hpp>

namespace YAML {

//------------------------------------------------------------------------------
// Camera loading.
bool convert<std::shared_ptr<ze::Camera>>::decode(
    const Node& node, ze::Camera::Ptr& camera)
{
  camera.reset();
  if (!node.IsMap())
  {
    LOG(ERROR) << "Parsing Camera failed because node is not a map.";
    return false;
  }
  try
  {
    std::string camera_type = extractChild<std::string>(node, "type");
    int width = extractChild<int>(node, "image_width");
    int height = extractChild<int>(node, "image_height");
    ze::VectorX intrinsics = extractChild<ze::VectorX>(node, "intrinsics");
    const YAML::Node distortion_config = node["distortion"];
    std::string distortion_type = extractChild<std::string>(distortion_config, "type");
    ze::VectorX distortion_parameters = extractChild<ze::VectorX>(distortion_config, "parameters");

    if(camera_type == "pinhole" && distortion_type == "none")
    {
      camera = std::make_shared<ze::PinholeCamera>(
            width, height, ze::CameraType::Pinhole, intrinsics,
            distortion_parameters);
    }
    else if(camera_type == "pinhole" && distortion_type == "radial-tangential")
    {
      camera = std::make_shared<ze::RadTanCamera>(
            width, height, ze::CameraType::PinholeRadialTangential, intrinsics,
            distortion_parameters);
    }
    else if(camera_type == "pinhole" && distortion_type == "equidistant")
    {
      camera = std::make_shared<ze::EquidistantCamera>(
            width, height, ze::CameraType::PinholeEquidistant, intrinsics,
            distortion_parameters);
    }
    else if(camera_type == "pinhole" && distortion_type == "fisheye")
    {
      camera = std::make_shared<ze::FovCamera>(
            width, height, ze::CameraType::PinholeFov, intrinsics,
            distortion_parameters);
    }
    else
    {
      LOG(FATAL) << "Camera model not yet supported.";
    }

    if(node["label"])
    {
      camera->setLabel(node["label"].as<std::string>());
    }
  }
  catch(const std::exception& e)
  {
    LOG(ERROR) << "YAML exception during parsing: " << e.what();
    camera.reset();
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
// Camera writing.
Node convert<ze::Camera::Ptr>::encode(const ze::Camera::Ptr& camera)
{
  LOG(FATAL) << "Not implemented!";
  Node camera_node;
  return camera_node;
}

//------------------------------------------------------------------------------
// CameraRig loading.
bool convert<std::shared_ptr<ze::CameraRig>>::decode(
    const Node& node, ze::CameraRig::Ptr& camera_rig)
{
  camera_rig.reset();
  try {
    if (!node.IsMap())
    {
      LOG(ERROR) << "Parsing CameraRig failed because node is not a map.";
      return false;
    }

    std::string label = extractChild<std::string>(node, "label");

    const Node cameras_node = node["cameras"];
    if (!cameras_node.IsSequence())
    {
      LOG(ERROR) << "Parsing CameraRig failed because 'cameras' is not a sequence.";
      return false;
    }

    size_t num_cameras = cameras_node.size();
    if (num_cameras == 0)
    {
      LOG(ERROR) << "Parsing CameraRig failed. Number of cameras is 0.";
      return false;
    }

    ze::TransformationVector T_Ci_B;
    ze::CameraVector cameras;
    for (size_t i = 0; i < num_cameras; ++i)
    {
      const Node& camera_node = cameras_node[i];
      if (!camera_node)
      {
        LOG(ERROR) << "Unable to get camera node for camera " << i;
        return false;
      }
      if (!camera_node.IsMap())
      {
        LOG(ERROR) << "Camera node for camera " << i << " is not a map.";
        return false;
      }

      ze::Camera::Ptr camera = extractChild<ze::Camera::Ptr>(camera_node, "camera");
      cameras.push_back(camera);

      if (camera_node["T_B_C"])
      {
        ze::Matrix4 T_B_C = extractChild<ze::Matrix4>(camera_node, "T_B_C");
        T_Ci_B.push_back(ze::Transformation(
                           ze::Quaternion::fromApproximateRotationMatrix(T_B_C.block<3,3>(0,0)),
                           T_B_C.block<3,1>(0,3)).inverse());
      }
      else if (camera_node["T_C_B"])
      {
        ze::Matrix4 T_C_B = extractChild<ze::Matrix4>(camera_node, "T_C_B");
        T_Ci_B.push_back(ze::Transformation(
                           ze::Quaternion::fromApproximateRotationMatrix(T_C_B.block<3,3>(0,0)),
                           T_C_B.block<3,1>(0,3)));
      }
      else
      {
        LOG(ERROR) << "Unable to get extrinsic transformation T_B_C or T_C_B "
                   << "for camera " << i;
        return false;
      }
    }

    camera_rig.reset(new ze::CameraRig(T_Ci_B, cameras, label));
  }
  catch (const std::exception& ex)
  {
    LOG(ERROR) << "YAML exception during parsing: " << ex.what();
    camera_rig.reset();
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
// CameraRig writing.
Node convert<std::shared_ptr<ze::CameraRig> >::encode(
    const std::shared_ptr<ze::CameraRig>& /*camera_rig*/)
{
  LOG(FATAL) << "Not implemented!";
  Node camera_rig_node;
  return camera_rig_node;
}

}  // namespace YAML

