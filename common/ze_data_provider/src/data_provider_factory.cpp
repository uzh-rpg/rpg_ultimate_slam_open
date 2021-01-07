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

#include <ze/common/logging.hpp>
#include <ze/data_provider/data_provider_factory.hpp>
#include <ze/data_provider/data_provider_base.hpp>
#include <ze/data_provider/data_provider_csv.hpp>
#include <ze/data_provider/data_provider_rosbag.hpp>
#include <ze/data_provider/data_provider_rostopic.hpp>

DEFINE_string(bag_filename, "dataset.bag", "Name of bagfile in data_dir.");

DEFINE_string(topic_cam0, "/cam0/image_raw", "");
DEFINE_string(topic_cam1, "/cam1/image_raw", "");
DEFINE_string(topic_cam2, "/cam2/image_raw", "");
DEFINE_string(topic_cam3, "/cam2/image_raw", "");

DEFINE_string(topic_imu0, "/imu0", "");
DEFINE_string(topic_imu1, "/imu1", "");
DEFINE_string(topic_imu2, "/imu2", "");
DEFINE_string(topic_imu3, "/imu3", "");

DEFINE_string(topic_dvs0, "/dvs0/events", "");
DEFINE_string(topic_dvs1, "/dvs1/events", "");
DEFINE_string(topic_dvs2, "/dvs2/events", "");
DEFINE_string(topic_dvs3, "/dvs3/events", "");

DEFINE_string(topic_acc0, "/acc0", "");
DEFINE_string(topic_acc1, "/acc1", "");
DEFINE_string(topic_acc2, "/acc2", "");
DEFINE_string(topic_acc3, "/acc3", "");

DEFINE_string(topic_gyr0, "/gyr0", "");
DEFINE_string(topic_gyr1, "/gyr1", "");
DEFINE_string(topic_gyr2, "/gyr2", "");
DEFINE_string(topic_gyr3, "/gyr3", "");

DEFINE_int32(data_source, 1, " 0: CSV, 1: Rosbag, 2: Rostopic");
DEFINE_string(data_dir, "", "Directory for csv dataset.");
DEFINE_uint64(num_imus, 1, "Number of IMUs used in the pipeline.");
DEFINE_uint64(num_cams, 1, "Number of normal cameras used in the pipeline.");
DEFINE_uint64(num_dvs, 1, "Number of event cameras used in the pipeline.");
DEFINE_uint64(num_accels, 0, "Number of Accelerometers used in the pipeline.");
DEFINE_uint64(num_gyros, 0, "Number of Gyroscopes used in the pipeline.");
DEFINE_double(timeshift_cam_imu, 0., "Delay between cam and IMU timestamps.");

namespace ze {

DataProviderBase::Ptr loadDataProviderFromGflags(const uint32_t num_cams)
{
  CHECK_GT(FLAGS_num_cams, 0u);
  CHECK_LE(FLAGS_num_cams, 4u);
  CHECK_LE(FLAGS_num_imus, 4u);

  // Fill camera topics.
  std::map<std::string, size_t> cam_topics;
  if (FLAGS_num_cams >= 1) cam_topics[FLAGS_topic_cam0] = 0;
  if (FLAGS_num_cams >= 2) cam_topics[FLAGS_topic_cam1] = 1;
  if (FLAGS_num_cams >= 3) cam_topics[FLAGS_topic_cam2] = 2;
  if (FLAGS_num_cams >= 4) cam_topics[FLAGS_topic_cam3] = 3;

  // Fill imu topics.
  std::map<std::string, size_t> imu_topics;
  if (FLAGS_num_imus >= 1) imu_topics[FLAGS_topic_imu0] = 0;
  if (FLAGS_num_imus >= 2) imu_topics[FLAGS_topic_imu1] = 1;
  if (FLAGS_num_imus >= 3) imu_topics[FLAGS_topic_imu2] = 2;
  if (FLAGS_num_imus >= 4) imu_topics[FLAGS_topic_imu3] = 3;

  // Fill event camera topics.
  std::map<std::string, size_t> dvs_topics;
  if (FLAGS_num_dvs >= 1) dvs_topics[FLAGS_topic_dvs0] = 0;
  if (FLAGS_num_dvs >= 2) dvs_topics[FLAGS_topic_dvs1] = 1;
  if (FLAGS_num_dvs >= 3) dvs_topics[FLAGS_topic_dvs2] = 2;
  if (FLAGS_num_dvs >= 4) dvs_topics[FLAGS_topic_dvs3] = 3;

  // Fill accelerometer topics.
  std::map<std::string, size_t> acc_topics;
  if (FLAGS_num_accels >= 1) acc_topics[FLAGS_topic_acc0] = 0;
  if (FLAGS_num_accels >= 2) acc_topics[FLAGS_topic_acc1] = 1;
  if (FLAGS_num_accels >= 3) acc_topics[FLAGS_topic_acc2] = 2;
  if (FLAGS_num_accels >= 4) acc_topics[FLAGS_topic_acc3] = 3;

  // Fill gyroscope topics.
  std::map<std::string, size_t> gyr_topics;
  if (FLAGS_num_gyros >= 1) gyr_topics[FLAGS_topic_gyr0] = 0;
  if (FLAGS_num_gyros >= 2) gyr_topics[FLAGS_topic_gyr1] = 1;
  if (FLAGS_num_gyros >= 3) gyr_topics[FLAGS_topic_gyr2] = 2;
  if (FLAGS_num_gyros >= 4) gyr_topics[FLAGS_topic_gyr3] = 3;

  // Create data provider.
  ze::DataProviderBase::Ptr data_provider;
  LOG(INFO) << "FLAGS_data_source = " << FLAGS_data_source;
  switch (FLAGS_data_source)
  {
    case 0: // CSV
    {
      LOG(FATAL) << "data_provider_csv not supported";
      break;
    }
    case 1: // Rosbag
    {
      // Use the split imu dataprovider
      if (FLAGS_num_accels != 0 && FLAGS_num_gyros != 0)
      {
        LOG(FATAL) << "data_provider_rosbag with split IMU not supported";
      }
      else
      {
        data_provider.reset(new DataProviderRosbag(FLAGS_bag_filename,
                                                   imu_topics,
                                                   cam_topics,
                                                   dvs_topics));
      }
      break;
    }
    case 2: // Rostopic
    {
      if (FLAGS_num_accels != 0 && FLAGS_num_gyros != 0)
      {
        LOG(FATAL) << "Unsupported data provider";
      }
      else
      {
        data_provider.reset(new DataProviderRostopic(imu_topics,
                                                     cam_topics,
                                                     dvs_topics,
                                                     1000u,
                                                     1000u,
                                                     10000u,
                                                     100u));
      }

      break;
    }
    default:
    {
      LOG(FATAL) << "Data source not known.";
      break;
    }
  }

  return data_provider;
}

} // namespace ze
