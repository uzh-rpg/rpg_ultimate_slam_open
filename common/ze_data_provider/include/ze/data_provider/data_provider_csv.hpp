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

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ze/common/macros.hpp>
#include <ze/common/types.hpp>
#include <ze/data_provider/data_provider_base.hpp>


namespace ze {

//fwd
namespace internal {
struct MeasurementBase;
}

class DataProviderCsv : public DataProviderBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using StampMeasurementPair = std::pair<int64_t, std::shared_ptr<internal::MeasurementBase>> ;
  using DataBuffer = std::multimap<int64_t, std::shared_ptr<internal::MeasurementBase>> ;

  DataProviderCsv(
      const std::string& csv_directory,
      const std::map<std::string, size_t>& imu_topics,
      const std::map<std::string, size_t>& camera_topics);

  virtual ~DataProviderCsv() = default;

  virtual bool spinOnce() override;

  virtual bool ok() const override;

  virtual size_t imuCount() const;

  virtual size_t cameraCount() const;

  inline size_t size() const
  {
    return buffer_.size();
  }

private:
  void loadImuData(
      const std::string data_dir,
      const size_t imu_index,
      const int64_t playback_delay);

  void loadCameraData(
      const std::string& data_dir,
      const size_t camera_index,
      int64_t playback_delay);

  //! Buffer to chronologically sort the data.
  DataBuffer buffer_;

  //! Points to the next published buffer value. Buffer can't change once loaded!
  DataBuffer::const_iterator buffer_it_;

  std::map<std::string, size_t> imu_topics_;
  std::map<std::string, size_t> camera_topics_;

  size_t imu_count_ = 0u;
};

} // namespace ze
