// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_common/vio_visualizer.hpp>

#include <ze/common/logging.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt32.h>

#include <imp/bridge/opencv/image_cv.hpp>
#include <ze/cameras/camera.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <ze/common/combinatorics.hpp>
#include <ze/common/file_utils.hpp>
#include <ze/common/stl_utils.hpp>
#include <ze/vio_common/frame.hpp>
#include <ze/vio_common/nframe.hpp>
#include <ze/vio_common/nframe_table.hpp>
#include <ze/vio_common/landmark_table.hpp>
#include <ze/visualization/viz_interface.hpp>
#include <ze/ros/pose_msg_bridge.hpp>
#include <imp/bridge/ros/ros_bridge.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>

DEFINE_int32(vio_viz_sleep, 1, "sleep");
DEFINE_int32(vio_viz_level, 0, "pyramid level to visualize");
DEFINE_int32(vio_viz_skip_rate, 1, "skip images for visualization");
DEFINE_bool(vio_viz_show_stereo_matches, false, "");
DEFINE_bool(vio_viz_show_image, true, "");
DEFINE_bool(vio_viz_show_image_statistics, true, "");
DEFINE_bool(vio_viz_feature_tracks, false, "");
DEFINE_int32(vio_viz_feature_tracks_colormap, 1,
             "OpenCV colormap for feature tracks");
DEFINE_uint64(vio_viz_feature_tracks_length, 0,
             "Limit length of feature track");
DEFINE_double(vio_viz_marker_scale, 1.0, "");
DEFINE_bool(vio_trace_pose, true, "");
DEFINE_bool(vio_viz_publish_T_G_B, false, "");

namespace ze {

// -----------------------------------------------------------------------------
VioVisualizer::VioVisualizer(
    const std::shared_ptr<Visualizer>& viz,
    const StereoIndexPairs& stereo_pairs)
  : viz_(viz)
  , stereo_pairs_(stereo_pairs)
{
  // Inititialize ROS if it was not initialized before.
  if(!ros::isInitialized())
  {
    VLOG(1) << "Initializting ROS";
    int argc = 0;
    ros::init(argc, nullptr, std::string("ze_vio"));
  }

  // Generate colors for feature tracks
  if (FLAGS_vio_viz_feature_tracks_colormap != -1) {
    cv::Mat cmap(1, landmarks_.c_capacity_, CV_8U,
                   cv::Scalar(0, 255, 255));
    for (int i=0; i != cmap.cols; ++i)
      cmap.at<uchar>(0, i) = (unsigned char) rand();

    cv::applyColorMap(cmap, cmap, FLAGS_vio_viz_feature_tracks_colormap);

    for (int i=0; i != cmap.cols; ++i)
    {
      const cv::Vec3b& c = cmap.at<cv::Vec3b>(0, i);
      colors_.push_back(cv::Scalar(c[0], c[1], c[2]));
    }
  }

  // Create node and subscribe.
  nh_.reset(new ros::NodeHandle("ze_vio"));
  it_.reset(new image_transport::ImageTransport(*nh_));
  pub_T_M_B_.reset(new ros::Publisher(nh_->advertise<geometry_msgs::PoseStamped>("T_M_B", 10)));
  pub_T_G_M_.reset(new ros::Publisher(nh_->advertise<geometry_msgs::PoseStamped>("T_G_M", 10)));
  pub_T_G_B_.reset(new ros::Publisher(nh_->advertise<geometry_msgs::PoseStamped>("T_G_B", 10)));
  pub_localization_state_.reset(new ros::Publisher(nh_->advertise<std_msgs::UInt32>("localization_state", 10)));
  pub_biases_.reset(new ros::Publisher(nh_->advertise<geometry_msgs::TwistStamped>("biases", 10)));
  pub_odom_.reset(new ros::Publisher(nh_->advertise<nav_msgs::Odometry>("odometry", 10)));
  pub_event_imgs_.reset(new image_transport::Publisher(it_->advertise("event_img", 10)));
}

// -----------------------------------------------------------------------------
VioVisualizer::~VioVisualizer()
{
  ofs_.close();
  stopThread();
}

// -----------------------------------------------------------------------------
void VioVisualizer::displayStereoMatches(
    const NFrame& nframe,
    const StereoIndexPair& stereo_pair)
{
  const Frame& frame_a = nframe.at(stereo_pair.second);
  const Frame& frame_b = nframe.at(stereo_pair.first);
  auto indices_a_b = getMatchIndices<LandmarkHandle::value_t>(
        frame_a.getLandmarkHandlesAsVector(),
        frame_b.getLandmarkHandlesAsVector(),
        std::bind(&isValidLandmarkHandleType, std::placeholders::_1));

  cv::Mat img_a = ImageCv8uC1(frame_a.pyr_->at(0)).cvMat();
  cv::Mat img_a_rgb = cv::Mat(img_a.size(), CV_8UC3);
  cv::cvtColor(img_a, img_a_rgb, cv::COLOR_GRAY2RGB);
  cv::Mat img_b = ImageCv8uC1(frame_b.pyr_->at(0)).cvMat();
  cv::Mat img_b_rgb = cv::Mat(img_b.size(), CV_8UC3);
  cv::cvtColor(img_b, img_b_rgb, cv::COLOR_GRAY2RGB);

  cv::Mat img_rgb(img_a.rows, 2 * img_a.cols, CV_8UC3);
  cv::Mat left_roi(img_rgb, cv::Rect(0, 0, img_a.cols, img_a.rows));
  img_a_rgb.copyTo(left_roi);
  cv::Mat right_roi(img_rgb, cv::Rect(img_a.cols, 0, img_a.cols, img_a.rows));
  img_b_rgb.copyTo(right_roi);

  for (const auto ab : indices_a_b)
  {
    auto pt_a = frame_a.px_vec_.col(ab.first);
    auto pt_b = frame_b.px_vec_.col(ab.second);
    cv::line(img_rgb,
             cv::Point2f(pt_a(0), pt_a(1)),
             cv::Point2f(pt_b(0) + img_a.cols, pt_b(1)), cv::Scalar(0,255,0), 1);
    cv::circle(img_rgb, cv::Point2f(pt_a(0), pt_a(1)), 3, cv::Scalar(0,0,255), 1);
    cv::circle(img_rgb, cv::Point2f(pt_b(0) + img_a.cols, pt_b(1)), 3, cv::Scalar(0,0,255), 1);
  }

  cv::imshow("stereo_matches", img_rgb);
}

// -----------------------------------------------------------------------------
void VioVisualizer::displayFeatureTracks(const Frame& frame_cur, const Frame& frame_ref)
{
  auto indices_cur_ref = getMatchIndices<LandmarkHandle::value_t>(
        frame_cur.getLandmarkHandlesAsVector(),
        frame_ref.getLandmarkHandlesAsVector(),
        std::bind(&isValidLandmarkHandleType, std::placeholders::_1));

  cv::Mat img = ImageCv8uC1(frame_cur.pyr_->at(0)).cvMat();
  cv::Mat img_rgb = cv::Mat(img.size(), CV_8UC3);
  cv::cvtColor(img, img_rgb, cv::COLOR_GRAY2RGB);

  for (const auto cur_ref : indices_cur_ref)
  {
    auto pt_cur = frame_cur.px_vec_.col(cur_ref.first);
    auto pt_ref = frame_ref.px_vec_.col(cur_ref.second);
    cv::line(img_rgb,
             cv::Point2f(pt_cur(0), pt_cur(1)),
             cv::Point2f(pt_ref(0), pt_ref(1)), cv::Scalar(0,255,0), 1);
  }

  cv::imshow("feature_tracks", img_rgb);
}

// -----------------------------------------------------------------------------
void VioVisualizer::displayLandmarksVisibleInFrame(
    const Frame& frame,
    const LandmarkTable& landmarks)
{
  int n_not_found = 0;
  for (size_t i = 0; i < frame.num_features_; ++i)
  {
    const LandmarkHandle lm_h = frame.landmark_handles_[i];
    if (landmarks.isStored(lm_h, true))
    {
      ++n_not_found;
      continue;
    }

    if (landmarks.numSuccessfulProjections(lm_h) > 0)
    {
      viz_->drawPoint("vio", lm_h.handle, landmarks.p_W(lm_h), Colors::Yellow, 0.05);
    }
  }
  LOG_IF(WARNING, n_not_found) << "Could not visualize " << n_not_found << " points"
                               << " because not in landmarks table.";
}

// -----------------------------------------------------------------------------
void VioVisualizer::displayAllLandmarks(
    const LandmarkTable& landmarks,
    bool draw_only_persistent)
{
  {
    Positions points(3, landmarks.numLandmarks());
    uint32_t num_points = 0u;
    for (uint32_t i = 0u; i < landmarks.numLandmarks(); ++i)
    {
      if (landmarks.typeAtSlot(i) == LandmarkType::Opportunistic
          || landmarks.typeAtSlot(i) == LandmarkType::Persistent)
      {
        points.col(num_points++) = landmarks.p_W_atSlot(i);
      }
    }
    points.conservativeResize(3, num_points);
    viz_->drawPoints("landmarks", 0, points, ze::Colors::LightGray,
                     0.05 * FLAGS_vio_viz_marker_scale);

    static int count = 0;
    if (count++ % 10 == 0)
    {
      for (uint32_t i = 0u; i < landmarks.numLandmarks(); ++i)
      {
        if (landmarks.typeAtSlot(i) == LandmarkType::Opportunistic
            || landmarks.typeAtSlot(i) == LandmarkType::Persistent)
        {
          viz_->drawPoint("all_landmarks", landmarks.getHandleAtSlot(i).handle,
                          landmarks.p_W_atSlot(i), ze::Colors::White,
                          0.05 * FLAGS_vio_viz_marker_scale);
        }
      }
    }
  }

  {
    Positions points(3, landmarks.numLandmarks());
    uint32_t num_points = 0u;
    for (uint32_t i = 0u; i < landmarks.numLandmarks(); ++i)
    {
      if (landmarks.typeAtSlot(i) == LandmarkType::SeedConverged)
      {
        points.col(num_points++) = landmarks.p_W_atSlot(i);
      }
    }
    points.conservativeResize(3, num_points);
    viz_->drawPoints("seed_converged", 0, points, ze::Colors::Yellow,
                     0.05 * FLAGS_vio_viz_marker_scale);
  }

  if (draw_only_persistent)
  {
    return;
  }

  {
    Positions points(3, landmarks.numLandmarks());
    uint32_t num_points = 0u;
    for (uint32_t i = 0u; i < landmarks.numLandmarks(); ++i)
    {
      if (landmarks.typeAtSlot(i) == LandmarkType::Seed)
      {
        points.col(num_points++) = landmarks.p_W_atSlot(i);
      }
    }
    points.conservativeResize(3, num_points);
    viz_->drawPoints("seeds", 1, points, ze::Colors::Red,
                     0.05 * FLAGS_vio_viz_marker_scale);
  }
}

// -----------------------------------------------------------------------------
void VioVisualizer::displayFrames(const NFrameTable& states)
{
  if (FLAGS_vio_viz_publish_T_G_B)
  {
    int n = 0;
    for (const Transformation& T_B_M : states.T_B_W_backend())
    {
      viz_->drawCoordinateFrame("frame", n++, T_G_M_ * T_B_M.inverse(),
                                0.2 * FLAGS_vio_viz_marker_scale);
    }
    std::vector<Position> trajectory_G;
    trajectory_G.reserve(trajectory_.size());
    for (const Position& p_M : trajectory_)
    {
      trajectory_G.push_back(T_G_M_ * p_M);
    }
    viz_->drawTrajectory("traj", 0, trajectory_G, ze::Colors::Green,
                         1.0 * FLAGS_vio_viz_marker_scale);
  }
  else
  {
    int n = 0;
    for (const Transformation& T_B_W : states.T_B_W_backend())
    {
      viz_->drawCoordinateFrame("frame", n++, T_B_W.inverse(),
                                0.2 * FLAGS_vio_viz_marker_scale);
    }
    viz_->drawTrajectory("traj", 0, trajectory_, ze::Colors::Green,
                         1.0 * FLAGS_vio_viz_marker_scale);
  }
}

// -----------------------------------------------------------------------------
void VioVisualizer::displayNFrameImages(
    const NFrame& nframe,
    const std::vector<uint32_t>& frame_indices,
    const NFrameTable& states,
    const LandmarkTable& landmarks,
    const uint32_t level)
{
  int vertical_padding = 40;
  if (!FLAGS_vio_viz_show_image_statistics)
  {
    vertical_padding = 0;
  }
  Size2u img_size = nframe.at(0).pyr_->at(level).size();
  cv::Mat img_rgb_all(img_size.height() + vertical_padding,
                      img_size.width() * frame_indices.size(),
                      CV_8UC3, cv::Scalar(0,0,0));
  uint32_t n = 0u;
  for(uint32_t frame_idx : frame_indices)
  {
    const Frame& frame = nframe.at(frame_idx);
    cv::Mat img_rgb(img_rgb_all, cv::Rect(n * img_size.width(), 0,
                                          img_size.width(), img_size.height()));
    CHECK(img_size == frame.pyr_->at(level).size());
    cv::Mat img = ImageCv8uC1(frame.pyr_->at(level)).cvMat();
    cv::cvtColor(img, img_rgb, cv::COLOR_GRAY2RGB);
    if (FLAGS_vio_viz_show_image_statistics)
    {
      cv::Mat img_rgb_with_padding(
            img_rgb_all, cv::Rect(n * img_size.width(), 0, img_size.width(),
                                  img_size.height() + vertical_padding));
      displayFrameImageWithStatistics(frame, frame_idx, states, landmarks, img_rgb_with_padding, level);
    }
    else
    {
      displayFrame(frame, landmarks, img_rgb, level);
    }
    ++n;
  }

  if (FLAGS_vio_viz_show_image)
  {
    cv::namedWindow("img", cv::WINDOW_NORMAL);
    cv::imshow("img", img_rgb_all);
    cv::waitKey(FLAGS_vio_viz_sleep);
  }

  if (pub_event_imgs_->getNumSubscribers() > 0)
  {
    sensor_msgs::Image img_msg = ze::cvMatToImageMsg(img_rgb_all, ros::Time(nframe.timestamp()*1e-9), "bgr8");
    pub_event_imgs_->publish(img_msg);
  }
}

// -----------------------------------------------------------------------------
void VioVisualizer::displayFrame(
    const Frame& frame,
    const LandmarkTable& landmarks,
    cv::Mat& img_rgb,
    const uint32_t level)
{
  real_t scale = 1.0 / (1 << level);
  for(size_t i = 0; i < frame.num_features_; ++i)
  {
    const LandmarkHandle lm_h = frame.landmark_handles_[i];
    if (isDeletedLandmarkHandle(lm_h))
    {
      continue;
    }

    if (isOutlierLandmarkHandle(lm_h))
    {
      continue;
    }

    if (!landmarks.isStored(lm_h, true))
    {
      continue;
    }

    const LandmarkType lm_type = landmarks.type(lm_h);
    if (isLandmarkInactive(lm_type))
    {
      continue;
    }
    cv::Scalar color(0,255,0);
    Keypoint px = frame.px_vec_.col(i) * scale;
    cv::circle(img_rgb, cv::Point2f(px(0), px(1)), 4, color, 1);
  }
}


// -----------------------------------------------------------------------------
void VioVisualizer::displayFrameImageWithStatistics(
    const Frame& frame,
    const uint32_t frame_idx,
    const NFrameTable& states,
    const LandmarkTable& landmarks,
    cv::Mat& img_rgb,
    const uint32_t level)
{
  int n_seed = 0;
  int n_opportunistic = 0;
  int n_opportunistic_aborbed = 0;
  int n_persistent = 0;
  int n_persistent_terminated = 0;
  int n_outliers = 0;
  int n_outliers_2 = 0;
  int n_not_found = 0;
  int n_localization_frame = 0;
  real_t scale = 1.0 / (1 << level);
  for(size_t i = 0; i < frame.num_features_; ++i)
  {
    const LandmarkHandle lm_h = frame.landmark_handles_[i];
    if (isDeletedLandmarkHandle(lm_h))
    {
      continue;
    }

    Keypoint px = frame.px_vec_.col(i) * scale;
    if (isOutlierLandmarkHandle(lm_h))
    {
      cv::circle(img_rgb, cv::Point2f(px(0), px(1)), 3, cv::Scalar(0, 0, 255), 2);
      ++n_outliers;
      continue;
    }

    if (!landmarks.isStored(lm_h, true))
    {
      ++n_not_found;
      continue;
    }

    if (landmarks.isLocalizationLandmark(lm_h))
    {
      ++n_localization_frame;
      continue;
    }

    const LandmarkType lm_type = landmarks.type(lm_h);
    if (lm_type == LandmarkType::Outlier)
    {
      ++n_outliers_2;
      continue;
    }
    cv::Scalar color = FLAGS_vio_viz_feature_tracks_colormap != -1 ?
                       colors_[lm_h.slot] : cv::Scalar(0, 255, 0);


    if (FLAGS_vio_viz_feature_tracks
        && lm_type == LandmarkType::Persistent)
    {
      const KeypointsVec& track = landmarks_.track(lm_h);
      size_t i = 1;
      if (FLAGS_vio_viz_feature_tracks_length > 0
          && track.size() > FLAGS_vio_viz_feature_tracks_length)
        i = track.size() - FLAGS_vio_viz_feature_tracks_length;
      for (; i < track.size(); ++i)
      {
        Keypoint px0 = track[i] * scale;
        Keypoint px1 = track[i-1] * scale;
        cv::line(img_rgb,
                 cv::Point2f(px0(0), px0(1)),
                 cv::Point2f(px1(0), px1(1)), color, 1, cv::LINE_AA);
      }
    }

    if (lm_type == LandmarkType::Seed || lm_type == LandmarkType::SeedConverged)
    {
      color = cv::Scalar(255,255,0);
      cv::circle(img_rgb, cv::Point2f(px(0), px(1)), 3, color, 1, cv::LINE_AA);
      ++n_seed;
    }
    else if (lm_type == LandmarkType::Opportunistic
             && landmarks.numSuccessfulProjections(lm_h) > 0)
    {
      color = cv::Scalar(0,255,255); // Yellow
      cv::circle(img_rgb, cv::Point2f(px(0), px(1)), 3, color, 1, cv::LINE_AA);
      ++n_opportunistic;
    }
    else if (lm_type == LandmarkType::OpportunisticAbsorbed)
    {
      color = cv::Scalar(255,0,255); // Magenta.
      cv::circle(img_rgb, cv::Point2f(px(0), px(1)), 3, color, 1, cv::LINE_AA);
      ++n_opportunistic;
    }
    else if (lm_type == LandmarkType::Persistent)
    {
      cv::circle(img_rgb, cv::Point2f(px(0), px(1)), 3, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
      ++n_persistent;
    }
  }

  Size2u img_size = frame.pyr_->at(level).size();
  cv::putText(img_rgb, "Candidates: "+std::to_string(n_seed),
              cv::Point(5, img_rgb.rows - 22),
              cv::FONT_HERSHEY_PLAIN, .8, cv::Scalar(255, 255, 0));
  cv::putText(img_rgb, "Persistent: "+std::to_string(n_persistent),
              cv::Point(5, img_rgb.rows - 7),
              cv::FONT_HERSHEY_PLAIN, .8, cv::Scalar(0, 255, 0));
  cv::line(img_rgb,
           cv::Point(180, img_rgb.rows - 27),
           cv::Point(180 + n_seed, img_rgb.rows - 27),
           cv::Scalar(255, 255, 0), 3);
  cv::line(img_rgb,
           cv::Point(180, img_rgb.rows - 12),
           cv::Point(180 + n_persistent, img_rgb.rows - 12),
           cv::Scalar(0, 255, 0), 3);

  // Localization state.
  size_t n_localization = localization_information_.n_localization_landmarks;
//  cv::putText(img_rgb, "Localiz.: " + std::to_string(n_localization_frame) +
//              " (" + std::to_string(n_localization) + ") ",
//              cv::Point(20, img_rgb.rows - 3),
//              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
//  cv::line(img_rgb,
//           cv::Point(180, img_rgb.rows - 7),
//           cv::Point(180 + n_localization_frame, img_rgb.rows - 7),
//           cv::Scalar(0, 255, 0), 3);

  std::string localization_state;
  cv::Scalar loc_state_color(0, 0, 255);
  switch (localization_information_.localization_state)
  {
    case LocalizationState::kUninitialized:
      localization_state = "Loc. Uninitialized";
      break;
    case LocalizationState::kNotLocalized:
      localization_state = "Not Localized";
      break;
    case LocalizationState::kLocalized:
      localization_state = "Localized";
      loc_state_color = cv::Scalar(0, 255, 0);
      break;
    case LocalizationState::kMapTracking:
      localization_state = "Map Tracking";
      loc_state_color = cv::Scalar(0, 255, 0);
      break;
    default:
      localization_state = "Unkown";
  }

//  cv::putText(img_rgb, localization_state,
//              cv::Point(img_rgb.cols - 150, img_rgb.rows - 10),
//              cv::FONT_HERSHEY_PLAIN, 1, loc_state_color);

  VLOG_IF(40, n_seed) << "Cam-" << frame_idx << ": #Seeds = " << n_seed;
  VLOG_IF(40, n_opportunistic) << "Cam-" << frame_idx << ": #Opportunistic = " << n_opportunistic;
  VLOG_IF(40, n_opportunistic_aborbed) << "Cam-" << frame_idx << ": #OpportunisticAbsorbed = " << n_opportunistic_aborbed;
  VLOG_IF(40, n_persistent) << "Cam-" << frame_idx << ": #Persistent = " << n_persistent;
  VLOG_IF(40, n_persistent_terminated) << "Cam-" << frame_idx << ": #PersistentTerminated = " << n_persistent_terminated;
  VLOG_IF(40, n_outliers) << "Cam-" << frame_idx << ": #Outliers = " << n_outliers;
  VLOG_IF(40, n_outliers_2) << "Cam-" << frame_idx << ": #Outliers-2 = " << n_outliers_2;
  VLOG_IF(40, n_not_found) << "Cam-" << frame_idx << ": #NotFound (probably absorbed) = " << n_not_found;
  VLOG_IF(40, n_localization) << "Cam-" << frame_idx << ": #Localization = " << n_localization;

}

// -----------------------------------------------------------------------------
void VioVisualizer::publishToRos()
{

  geometry_msgs::PoseStamped T_M_B =
      transformationToPoseStampedMsg(T_Bk_M_.inverse(), nframe_k_stamp_);
  // @todo: should be mission T_M_B.header.frame_id="mission";
  // But the tf from map to mission is not published, actually missing all tfs.
  T_M_B.header.frame_id="map";
  pub_T_M_B_->publish(T_M_B);

  if (localization_information_.localization_state == LocalizationState::kMapTracking
      || localization_information_.localization_state == LocalizationState::kLocalized)
  {
    geometry_msgs::PoseStamped T_G_M =
                    transformationToPoseStampedMsg(T_Bk_M_.inverse(), nframe_k_stamp_);
    T_G_M.header.frame_id="map";
    pub_T_G_M_->publish(transformationToPoseStampedMsg(
                                localization_information_.T_G_M, nframe_k_stamp_));
    geometry_msgs::PoseStamped T_G_B =
                    transformationToPoseStampedMsg(T_Bk_M_.inverse(), nframe_k_stamp_);
    T_G_B.header.frame_id="map";
    pub_T_G_B_->publish(transformationToPoseStampedMsg(
                          localization_information_.T_G_M * T_Bk_M_.inverse(),
                          nframe_k_stamp_));
    T_G_M_ = localization_information_.T_G_M;
  }
  std_msgs::UInt32 state_msg;
  state_msg.data = static_cast<int>(localization_information_.localization_state);
  pub_localization_state_->publish(state_msg);

  if (pub_biases_->getNumSubscribers() != 0)
  {
    geometry_msgs::TwistStamped biases;
    biases.header.stamp = ros::Time().fromNSec(nframe_k_stamp_);
    biases.twist.linear.x = acc_bias_[0];
    biases.twist.linear.y = acc_bias_[1];
    biases.twist.linear.z = acc_bias_[2];
    biases.twist.angular.x = gyr_bias_[0];
    biases.twist.angular.y = gyr_bias_[1];
    biases.twist.angular.z = gyr_bias_[2];
    pub_biases_->publish(biases);
  }
}

void VioVisualizer::publishOdometry(const Odometry &state)
{
  if (stop_thread_)
    return;

  if (pub_odom_->getNumSubscribers() > 0)
  {
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time().fromNSec(state.stamp);
    msg.header.frame_id = "map";
    msg.child_frame_id = "body";

    const Position& p = state.T_W_B.getPosition();
    msg.pose.pose.position.x = p[0];
    msg.pose.pose.position.y = p[1];
    msg.pose.pose.position.z = p[2];

    const Quaternion& q = state.T_W_B.getRotation();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    const Vector3& v = state.v_W;
    msg.twist.twist.linear.x = v[0];
    msg.twist.twist.linear.y = v[1];
    msg.twist.twist.linear.z = v[2];

    const Vector3& omega = state.omega_B;
    msg.twist.twist.angular.x = omega[0];
    msg.twist.twist.angular.y = omega[1];
    msg.twist.twist.angular.z = omega[2];

    pub_odom_->publish(msg);
  }

  viz_->drawRobot("body", state.T_W_B, ros::Time().fromNSec(state.stamp));
}

// -----------------------------------------------------------------------------
void VioVisualizer::traceStateEstimate(
    const int64_t& stamp,
    const Transformation& T_Bk_W,
    const Vector3& acc_bias,
    const Vector3& gyr_bias)
{
  // Get latest state.
  const Transformation T_W_B =  T_Bk_W.inverse();

  // Trace state.
  if(!ofs_.is_open())
  {
    openOutputFileStream(joinPath(FLAGS_log_dir, "traj_es.csv"), &ofs_);
    ofs_ << "timestamp, x, y, z, qx, qy, qz, qw, vx, vy, vz, bgx, bgy, bgz, bax, bay, baz\n";
  }
  Vector3 p = T_W_B.getPosition();
  Quaternion q = T_W_B.getRotation();
  Vector3 v = Vector3::Zero();
  ofs_ << stamp << ", "
       << p.x() << ", " << p.y() << ", " << p.z() << ", "
       << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << ", "
       << v.x() << ", " << v.y() << ", " << v.z() << ", "
       << gyr_bias.x() << ", " << gyr_bias.y() << ", " << gyr_bias.z() << ", "
       << acc_bias.x() << ", " << acc_bias.y() << ", " << acc_bias.z() << "\n";
}

//------------------------------------------------------------------------------
// Concurrent Visualization:

void VioVisualizer::startThread()
{
  CHECK(!thread_) << "Thread was already started.";
  thread_.reset(new std::thread(&VioVisualizer::visualizationLoop, this));
}

void VioVisualizer::stopThread()
{
  if (thread_)
  {
    VLOG(1) << "Interrupt and stop thread.";
    stop_thread_ = true;
    wait_condition_.notify_all();
    thread_->join();
    thread_.reset();
  }
  VLOG(1) << "Thread stopped and joined.";
}

void VioVisualizer::visualizationLoop()
{
  VLOG(1) << "Started visualization thread.";
  while (!stop_thread_)
  {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      while (!stop_thread_ && last_visualized_nframe_stamp_ == nframe_k_stamp_)
      {
        this->wait_condition_.wait(lock);
      }
      if (stop_thread_)
      {
        return;
      }
      last_visualized_nframe_stamp_ = nframe_k_stamp_;
    }
    visualizeCopiedData();
  }
}

void VioVisualizer::copyDataAndVisualizeConcurrently(
    const NFrameTable& states,
    const LandmarkTable& landmarks,
    const LocalizationInformation& localization_information)
{
  if (!states.nframeK())
  {
    return;
  }

  {
    std::unique_lock<std::mutex> lock(mutex_);
    T_Bk_M_ = states.T_Bk_W();
    trajectory_.push_back(T_Bk_M_.inverse().getPosition());
    acc_bias_ = states.accBias();
    gyr_bias_ = states.gyrBias();
    v_W_ = states.v_W(states.nframeHandleK());
    nframe_k_ = states.nframeK();
    nframe_k_stamp_ = nframe_k_->timestamp();
    states_ = states;
    landmarks_.setNumLandmark(landmarks.numLandmarks());
    landmarks_.p_W() = landmarks.p_W();
    landmarks_.types() = landmarks.types();
    landmarks_.infos() = landmarks.infos();
    landmarks_.versions() = landmarks.versions();
    landmarks_.obs() = landmarks.obs();
    landmarks_.tracks() = landmarks.tracks();
    localization_information_ = localization_information;
    publishToRos();
  }

  if (FLAGS_vio_trace_pose)
  {
    traceStateEstimate(nframe_k_stamp_, T_Bk_M_, acc_bias_, gyr_bias_);
  }

  if ((nframe_k_->seq() % FLAGS_vio_viz_skip_rate) == 0)
  {
    if (thread_)
    {
      wait_condition_.notify_one();
    }
    else
    {
      visualizeCopiedData();
    }
  }

}

void VioVisualizer::visualizeCopiedData()
{
  NFrame::ConstPtr nframe;
  {
    std::unique_lock<std::mutex> lock(this->mutex_);
    displayFrames(states_);
    CHECK(nframe_k_);
    nframe = nframe_k_;
  }

  // landmarks_ may be accessed from two threads concurrently but we don't care.
  // it should be safe since the landmarks_ datastructure is fixed size and will
  // stay at the same memory location. The reason is that copyData would otherwise
  // have to wait for too long for the mutex which slows the tracker thread down.

  if (FLAGS_vio_viz_show_stereo_matches
      && stereo_pairs_.size() > 0u)
  {
    for (const StereoIndexPair p : stereo_pairs_)
    {
      displayStereoMatches(*nframe, p);
    }
  }

  displayNFrameImages(*nframe, range(nframe->size()), states_, landmarks_,
                      FLAGS_vio_viz_level);

  displayAllLandmarks(landmarks_, false);
}

} // namespace ze
