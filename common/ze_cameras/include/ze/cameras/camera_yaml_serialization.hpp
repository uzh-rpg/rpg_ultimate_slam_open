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

#include <memory>

#include <ze/common/logging.hpp>
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// The yaml-cpp version in yaml_cpp_catkin uses auto_ptr which is deprecated.
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

namespace ze {
class Camera;
class CameraRig;
}  // namespace ze

namespace YAML {

template<>
struct convert<std::shared_ptr<ze::Camera>>
{
  static bool decode(const Node& node, std::shared_ptr<ze::Camera>& camera);
  static Node encode(const std::shared_ptr<ze::Camera>& camera);
};

template<>
struct convert<std::shared_ptr<ze::CameraRig>>
{
  static bool decode(const Node& node, std::shared_ptr<ze::CameraRig>& camera);
  static Node encode(const std::shared_ptr<ze::CameraRig>& camera_rig);
};

}  // namespace YAML
