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

#include <ze/imu/imu_rig.hpp>
#include <ze/imu/imu_yaml_serialization.hpp>

namespace ze {

ImuRig::ImuRig(
    const TransformationVector& T_B_S,
    const ImuVector& imus,
    const std::string& label)
  : T_B_S_(T_B_S)
  , imus_(imus)
  , label_(label)
{
  CHECK_EQ(T_B_S_.size(), imus_.size());
  for(size_t i = 0; i < size(); ++i)
  {
    CHECK_NOTNULL(imus_[i].get());
  }

  // set inverse transformations
  for(size_t i = 0; i < size(); ++i)
  {
    T_S_B_.push_back(T_B_S_[i].inverse());
  }
}

ImuRig::Ptr ImuRig::loadFromYaml(const std::string& yaml_file)
{
  try
  {
    YAML::Node doc = YAML::LoadFile(yaml_file.c_str());
    return doc.as<ImuRig::Ptr>();
  }
  catch (const std::exception& ex)
  {
    LOG(ERROR) << "Cannot load ImuRig from file:" << yaml_file << "\n"
               << ex.what();
  }
  return nullptr;
}
} // namespace ze
