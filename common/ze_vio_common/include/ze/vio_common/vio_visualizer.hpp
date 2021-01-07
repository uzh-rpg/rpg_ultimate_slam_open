// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <condition_variable>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <gflags/gflags.h>

#include <ze/common/types.hpp>
#include <ze/vio_common/landmark_types.hpp>
#include <ze/vio_common/landmark_table.hpp>
#include <ze/vio_common/nframe_table.hpp>
#include <ze/vio_common/localization.hpp>

#include <opencv2/core/core.hpp>

DECLARE_int32(vio_viz_sleep);

// fwd
namespace ros {
class Publisher;
class NodeHandle;
}

namespace image_transport {
class ImageTransport;
class Publisher;
}

namespace cv {
class Mat;
}

namespace ze {

// fwd
class CameraRig;
class Frame;
class NFrame;
class Visualizer;

class VioVisualizer
{
public:
  ZE_POINTER_TYPEDEFS(VioVisualizer);

  VioVisualizer() = delete;
  VioVisualizer(
      const std::shared_ptr<Visualizer>& viz,
      const StereoIndexPairs& stereo_pairs);
  ~VioVisualizer();

  void displayStereoMatches(
      const NFrame& nframe,
      const StereoIndexPair& stereo_pair);

  void displayFeatureTracks(
      const Frame& frame_cur,
      const Frame& frame_ref);

  void displayAllLandmarks(
      const LandmarkTable& landmarks,
      bool draw_only_persistent = true);

  void displayFrames(
      const NFrameTable& states);

  void displayLandmarksVisibleInFrame(
      const Frame& frame,
      const LandmarkTable& landmarks);

  void displayNFrameImages(
      const NFrame& nframe,
      const std::vector<uint32_t>& frame_indices,
      const NFrameTable& states,
      const LandmarkTable& landmarks,
      const uint32_t level = 0u);

  void displayFrame(
      const Frame& frame,
      const LandmarkTable& landmarks,
      cv::Mat& img_rgb,
      const uint32_t level);

  void displayFrameImageWithStatistics(
      const Frame& frame,
      const uint32_t frame_idx,
      const NFrameTable& states,
      const LandmarkTable& landmarks,
      cv::Mat& img_rgb,
      const uint32_t level);

  void publishToRos();
  void publishOdometry(const Odometry& state);

  void traceStateEstimate(
      const int64_t& stamp,
      const Transformation& T_Bk_W,
      const Vector3& acc_bias,
      const Vector3& gyr_bias);

  //! @name Multi-threaded visualization.
  //! @{
  void startThread();

  void stopThread();

  void copyDataAndVisualizeConcurrently(
      const NFrameTable& nframe_states,
      const LandmarkTable& landmarks,
      const LocalizationInformation& localization_information);
  //! @}

private:
  std::shared_ptr<Visualizer> viz_;
  std::ofstream ofs_;
  StereoIndexPairs stereo_pairs_;

  // History to be visualized.
  std::vector<Position> trajectory_;

  std::vector<cv::Scalar> colors_;

  // Copy of data for concurrent visualization.
  Transformation T_Bk_M_;
  Transformation T_G_M_;
  Vector3 acc_bias_;
  Vector3 gyr_bias_;
  Vector3 v_W_;
  int64_t nframe_k_stamp_ = -1;
  std::shared_ptr<const NFrame> nframe_k_;
  LandmarkTable landmarks_;
  NFrameTable states_;
  NFrameHandle last_traced_handle_;
  LocalizationInformation localization_information_;

  // Multithreading.
  mutable std::condition_variable wait_condition_;
  mutable std::mutex mutex_;
  std::unique_ptr<std::thread> thread_;
  volatile bool stop_thread_ = false;
  int64_t last_visualized_nframe_stamp_ = -1;

  // ROS
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<ros::Publisher> pub_T_M_B_;
  std::shared_ptr<ros::Publisher> pub_T_G_M_;
  std::shared_ptr<ros::Publisher> pub_T_G_B_;
  std::shared_ptr<ros::Publisher> pub_localization_state_;
  std::shared_ptr<ros::Publisher> pub_biases_;
  std::shared_ptr<ros::Publisher> pub_odom_;
  std::shared_ptr<image_transport::Publisher> pub_event_imgs_;

  void visualizationLoop();
  void visualizeCopiedData();
};

} // namespace ze
