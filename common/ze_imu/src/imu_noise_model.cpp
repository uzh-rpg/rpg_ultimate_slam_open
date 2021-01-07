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

#include <ze/imu/imu_noise_model.hpp>

namespace ze {

//------------------------------------------------------------------------------
// Noise model base class

ImuNoiseModel::ImuNoiseModel(ImuNoiseType type)
  : type_(type)
{
}

std::string ImuNoiseModel::typeAsString() const
{
  switch (type())
  {
    case ImuNoiseType::WhiteBrownian: return "White Brownian";
    case ImuNoiseType::None: return "No Noise";
    default:
      LOG(FATAL) << "Unknown noise model";
  }
}

//------------------------------------------------------------------------------
// No Noise model
ImuNoiseNone::ImuNoiseNone(): ImuNoiseModel(Type)
{
}

//------------------------------------------------------------------------------
// White brownian noise model
ImuNoiseWhiteBrownian::ImuNoiseWhiteBrownian(real_t noise_density,
           real_t bandwidth,
           real_t bias_noise_density)
  : ImuNoiseModel(Type)
  , noise_density_(noise_density)
  , bandwidth_(bandwidth)
  , bias_noise_density_(bias_noise_density)
{
  CHECK(bandwidth > 0) << "Bandwidth must be >0'";
}

} // namespace ze
