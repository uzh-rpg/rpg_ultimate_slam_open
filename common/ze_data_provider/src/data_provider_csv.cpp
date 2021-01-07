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

#include <ze/data_provider/data_provider_csv.hpp>

#include <fstream>
#include <iostream>
#include <ze/common/logging.hpp>

#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/core/image_base.hpp>
#include <ze/common/time_conversions.hpp>
#include <ze/common/string_utils.hpp>
#include <ze/common/file_utils.hpp>

namespace ze {
namespace internal {

enum class MeasurementType
{
  Imu,
  Camera,
  FeatureTrack,
};

struct MeasurementBase
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ZE_POINTER_TYPEDEFS(MeasurementBase);

public:
  MeasurementBase() = delete;
  virtual ~MeasurementBase() = default;

protected:
  MeasurementBase(int64_t stamp_ns, MeasurementType type)
    : stamp_ns(stamp_ns)
    , type(type)
  {}

public:
  const int64_t stamp_ns;
  const MeasurementType type;
};

struct ImuMeasurement : public MeasurementBase
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ZE_POINTER_TYPEDEFS(ImuMeasurement);

  ImuMeasurement() = delete;
  ImuMeasurement(int64_t stamp_ns, const size_t imu_idx,
                 const Vector3& acc, const Vector3& gyr)
    : MeasurementBase(stamp_ns, MeasurementType::Imu)
    , acc(acc)
    , gyr(gyr)
    , imu_index(imu_idx)
  {}
  virtual ~ImuMeasurement() = default;

  const Vector3 acc;
  const Vector3 gyr;
  const size_t imu_index;
};

struct CameraMeasurement : public MeasurementBase
{
  ZE_POINTER_TYPEDEFS(CameraMeasurement);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraMeasurement() = delete;
  CameraMeasurement(int64_t stamp_ns, size_t cam_idx, const std::string& img_path)
    : MeasurementBase(stamp_ns, MeasurementType::Camera)
    , camera_index(cam_idx)
    , image_filename(img_path)
  {}
  virtual ~CameraMeasurement() = default;

  inline ImageBase::Ptr loadImage() const
  {
    //! @todo: Make an option which pixel-type to load.
    ImageCv8uC1::Ptr img;
    cvBridgeLoad<Pixel8uC1>(img, image_filename, PixelOrder::gray);
    CHECK_NOTNULL(img.get());
    CHECK(img->numel() > 0);
    return img;
  }

  const size_t camera_index;
  const std::string image_filename;
};

struct FeatureTrackMeasurement : public MeasurementBase
{
  ZE_POINTER_TYPEDEFS(FeatureTrackMeasurement);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FeatureTrackMeasurement() = delete;
  FeatureTrackMeasurement(int64_t stamp_ns, size_t cam_idx, int track_id,
                          const Eigen::Vector2d& keypoint_measurement,
                          double keypoint_std_dev)
    : MeasurementBase(stamp_ns, MeasurementType::FeatureTrack)
    , camera_index(cam_idx)
    , track_id(track_id)
    , keypoint_measurement(keypoint_measurement)
    , keypoint_std_dev(keypoint_std_dev)
  {}
  virtual ~FeatureTrackMeasurement() = default;

  const size_t camera_index;
  const int track_id;
  const Eigen::Vector2d keypoint_measurement;
  const double keypoint_std_dev;
};

} // namespace internal

DataProviderCsv::DataProviderCsv(
    const std::string& csv_directory,
    const std::map<std::string, size_t>& imu_topics,
    const std::map<std::string, size_t>& camera_topics)
  : DataProviderBase(DataProviderType::Csv)
  , imu_topics_(imu_topics)
  , camera_topics_(camera_topics)
{
  VLOG(1) << "Loading .csv dataset from directory \"" << csv_directory << "\".";

  for (auto it : imu_topics)
  {
    std::string dir = joinPath(csv_directory, it.first);
    loadImuData(dir, it.second, 0u);
  }

  for (auto it : camera_topics)
  {
    std::string dir = joinPath(csv_directory, it.first);
    loadCameraData(dir, it.second, millisecToNanosec(100));
  }

  buffer_it_ = buffer_.cbegin();
  VLOG(1) << "done.";
}

bool DataProviderCsv::spinOnce()
{
  if (buffer_it_ != buffer_.cend())
  {
    const internal::MeasurementBase::Ptr& data = buffer_it_->second;
    switch (data->type)
    {
    case internal::MeasurementType::Camera:
    {
      if (camera_callback_)
      {
        internal::CameraMeasurement::ConstPtr cam_data =
            std::dynamic_pointer_cast<const internal::CameraMeasurement>(data);
        camera_callback_(cam_data->stamp_ns,
                         cam_data->loadImage(),
                         cam_data->camera_index);
      }
      else
      {
        LOG_FIRST_N(WARNING, 1) << "No camera callback registered but measurements available.";
      }
      break;
    }
    case internal::MeasurementType::Imu:
    {
      if (imu_callback_)
      {
        internal::ImuMeasurement::ConstPtr imu_data =
            std::dynamic_pointer_cast<const internal::ImuMeasurement>(data);
        imu_callback_(imu_data->stamp_ns,
                      imu_data->acc,
                      imu_data->gyr,
                      imu_data->imu_index);
      }
      else
      {
        LOG_FIRST_N(WARNING, 1) << "No IMU callback registered but measurements available";
      }
      break;
    }
    default:
    {
      LOG(FATAL) << "Unhandled message type: " << static_cast<int>(data->type);
      break;
    }
    }
    ++buffer_it_;
    return true;
  }
  return false;
}

bool DataProviderCsv::ok() const
{
  if (!running_)
  {
    VLOG(1) << "Data Provider was paused/terminated.";
    return false;
  }
  if (buffer_it_ == buffer_.cend())
  {
    VLOG(1) << "All data processed.";
    return false;
  }
  return true;
}

size_t DataProviderCsv::cameraCount() const
{
  return camera_topics_.size();
}

size_t DataProviderCsv::imuCount() const
{
  return imu_topics_.size();
}

void DataProviderCsv::loadImuData(
    const std::string data_dir,
    const size_t imu_index,
    const int64_t playback_delay)
{
  const std::string kHeader = "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]";
  std::ifstream fs;
  openFileStreamAndCheckHeader(data_dir+"/data.csv", kHeader, &fs);
  std::string line;
  size_t i = 0;
  while (std::getline(fs, line))
  {
    std::vector<std::string> items = splitString(line, ',');
    CHECK_EQ(items.size(), 7u);
    Eigen::Vector3d acc, gyr;
    acc << std::stod(items[4]), std::stod(items[5]), std::stod(items[6]);
    gyr << std::stod(items[1]), std::stod(items[2]), std::stod(items[3]);
    auto imu_measurement =
        std::make_shared<internal::ImuMeasurement>(
          std::stoll(items[0]), imu_index,
          acc.cast<real_t>(), gyr.cast<real_t>());

    buffer_.insert(std::make_pair(
                     imu_measurement->stamp_ns + playback_delay,
                     imu_measurement));
    ++i;
  }
  VLOG(2) << "Loaded " << i << " IMU measurements.";
  fs.close();
}

void DataProviderCsv::loadCameraData(
    const std::string& data_dir,
    const size_t camera_index,
    int64_t playback_delay)
{
  const std::string kHeader = "#timestamp [ns],filename";
  std::ifstream fs;
  openFileStreamAndCheckHeader(data_dir+"/data.csv", kHeader, &fs);
  std::string line;
  size_t i = 0;
  while (std::getline(fs, line))
  {
    std::vector<std::string> items = splitString(line, ',');
    CHECK_EQ(items.size(), 2u);
    auto camera_measurement =
        std::make_shared<internal::CameraMeasurement>(
          std::stoll(items[0]), camera_index, data_dir + "/data/" + items[1]);

    buffer_.insert(std::make_pair(
                     camera_measurement->stamp_ns + playback_delay,
                     camera_measurement));
    ++i;
  }
  VLOG(2) << "Loaded " << i << " camera measurements.";
  fs.close();
}

} // namespace ze
