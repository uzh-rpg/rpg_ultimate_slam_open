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

#include <ze/data_provider/data_provider_base.hpp>

namespace ze {

DataProviderBase::DataProviderBase(DataProviderType type)
  : type_(type)
  , signal_handler_(running_)
{}

void DataProviderBase::spin()
{
  while (ok())
  {
    spinOnce();
  }
}

void DataProviderBase::pause()
{
  running_ = false;
}

void DataProviderBase::shutdown()
{
  running_ = false;
}

void DataProviderBase::registerImuCallback(const ImuCallback& imu_callback)
{
  imu_callback_ = imu_callback;
}

void DataProviderBase::registerGyroCallback(const GyroCallback& gyro_callback)
{
  gyro_callback_ = gyro_callback;
}

void DataProviderBase::registerAccelCallback(const AccelCallback& accel_callback)
{
  accel_callback_ = accel_callback;
}

void DataProviderBase::registerCameraCallback(const CameraCallback& camera_callback)
{
  camera_callback_ = camera_callback;
}

void DataProviderBase::registerCameraCallback(const DVSCallback& dvs_callback)
{
  dvs_callback_ = dvs_callback;
}

} // namespace ze
