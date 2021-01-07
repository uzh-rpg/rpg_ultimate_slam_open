// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_frontend/frontend_vio.hpp>

#include <ze/cameras/camera_rig.hpp>
#include <ze/common/combinatorics.hpp>
#include <ze/common/logging.hpp>
#include <ze/common/file_utils.hpp>
#include <ze/common/statistics.hpp>
#include <ze/geometry/triangulation.hpp>
#include <ze/geometry/pose_optimizer.hpp>
#include <ze/vio_common/frame.hpp>
#include <ze/vio_common/imu_integrator.hpp>
#include <ze/vio_common/landmark_triangulation.hpp>
#include <ze/vio_common/landmark_utils.hpp>
#include <ze/vio_common/nframe.hpp>
#include <ze/vio_common/nframe_utils.hpp>
#include <ze/vio_frontend/feature_tracker.hpp>
#include <ze/vio_frontend/feature_initializer.hpp>
#include <ze/vio_frontend/frontend_gflags.hpp>
#include <ze/vio_frontend/stereo_matcher.hpp>
#include <ze/vio_frontend/track_extractor.hpp>
#include <ze/vio_frontend/keyframe_selection.hpp>
#include <ze/vio_frontend/refinement.hpp>
#include <ze/vio_frontend/landmarks_reprojector.hpp>
#include <imp/features/feature_detector.hpp>


namespace ze {

FrontendVio::FrontendVio()
  : FrontendBase()
{
  init();
}

FrontendVio::FrontendVio(const CameraRig::Ptr& rig)
  : FrontendBase(rig)
{
  init();
}

void FrontendVio::init()
{
  reprojector_ = std::make_shared<LandmarksReprojector>(
                   *rig_, FLAGS_imp_detector_border_margin,
                   FLAGS_imp_detector_grid_size, landmarks_, states_);
}

// -----------------------------------------------------------------------------
void FrontendVio::processData(const Transformation& T_Bkm1_Bk)
{
  NFrame::Ptr nframe_k   = states_.nframeK();
  NFrame::Ptr nframe_lkf = states_.nframeLkf();
  bool add_frame_to_backend = (frame_count_ % FLAGS_vio_add_every_nth_frame_to_backend == 0);

  switch(stage_)
  {
    //--------------------------------------------------------------------------
    case FrontendStage::Running:
    {
      CHECK_NOTNULL(nframe_lkf.get());

      // KLT-Tracking of features and RANSAC:
      std::vector<real_t> disparities_sq;
      uint32_t num_outliers;
      std::tie(disparities_sq, num_outliers) = trackFrameKLT();
      uint32_t num_tracked = disparities_sq.size();

      motion_type_ = classifyMotion(disparities_sq, num_outliers);

      makeKeyframeIfNecessary(num_tracked);
      break;
    }
    //--------------------------------------------------------------------------
    case FrontendStage::AttitudeEstimation:
    {
      LOG(WARNING) << "Stage = AttitudeEstimation";
      motion_type_ = VioMotionType::GeneralMotion;
      if (add_frame_to_backend)
      {
        states_.setKeyframe(nframe_k->handle());
      }
      break;
    }
    //--------------------------------------------------------------------------
    case FrontendStage::Initializing:
    {
      LOG(WARNING) << "Stage = Initializing";
      motion_type_ = VioMotionType::GeneralMotion;
      CHECK(add_frame_to_backend);
      for (size_t i = 0u; i < nframe_k->size(); ++i)
      {
        Frame& frame = nframe_k->at(i);
        frame.min_depth_ = FLAGS_vio_min_depth;
        frame.max_depth_ = FLAGS_vio_max_depth;
        frame.median_depth_ = FLAGS_vio_median_depth;
      }
      feature_initializer_->detectAndInitializeNewFeatures(
            *nframe_k, range(rig_->size()));

      //if (rig_->size() > 1u)
        //stereo_matcher_->matchStereoAndRejectOutliers(*nframe_k, states_.T_Bk_W());

      // Add observations to landmarks.
      addLandmarkObservations(*nframe_k, landmarks_);

      // Switch state:
      stage_ = FrontendStage::Running;
      break;
    }
    default:
      LOG(FATAL) << "Case not implemented.";
      break;
  }
}

void FrontendVio::makeKeyframeIfNecessary(const uint32_t num_tracked)
{
  NFrame::Ptr nframe_k   = states_.nframeK();
  NFrame::Ptr nframe_lkf = states_.nframeLkf();

  // Set last observation in landmarks:
  setLandmarksLastObservationInNFrame(*nframe_k, landmarks_);

  // Remove old landmarks.
  {
    auto t = timers_[Timer::remove_old_landmarks].timeScope();
    removeOldLandmarks(FLAGS_vio_max_landmarks, nframe_k->seq(), landmarks_);
  }

  // Compute scene depth statistics.
  {
    auto t = timers_[Timer::scene_depth].timeScope();
    setSceneDepthInNFrame(*nframe_k, landmarks_, states_.T_Bk_W(), T_C_B_,
                          FLAGS_vio_min_depth, FLAGS_vio_max_depth,
                          FLAGS_vio_median_depth);
  }

  // Select new keyframe.
  if (!needNewKeyframe(
        *nframe_k, *nframe_lkf, states_, states_.T_Bk_W(), T_C_B_, num_tracked))
  {
    return; // No new keyframe.
  }

  // Don't detect features close to existing features.
  feature_initializer_->setOccupancyGrid(reprojector_->gridVec());

  // Mark all features in keyframe as opportunistic.
  setTypeOfConvergedSeedsInNFrame(
        *nframe_k, LandmarkType::Opportunistic, landmarks_);

  // Detect new features.
  if (num_tracked < FLAGS_vio_min_tracked_features_total)
  {
    auto t = timers_[Timer::initialize_features].timeScope();

    // If critical, we extract in all frames features and match stereo.
    // otherwise, just detect features in camera with least features.
    if (num_tracked < FLAGS_vio_kfselect_numfts_lower_thresh
        && rig_->stereoPairs().size() > 0u)
    {
      LOG(WARNING) << "Critical: Force stereo triangulation";
      std::vector<uint32_t> frame_idx_vec = range(rig_->size());
      feature_initializer_->detectAndInitializeNewFeatures(*nframe_k, frame_idx_vec);

      std::vector<std::pair<uint32_t, std::vector<uint32_t>>> stereo_matches =
          stereo_matcher_->matchStereoAndRejectOutliers(*nframe_k, states_.T_Bk_W());

      // Set all stereo observations to opportunistic.
      for (const std::pair<uint32_t, std::vector<uint32_t>>& it : stereo_matches)
      {
        const Frame& ref_frame = nframe_k->at(it.first);
        for (const uint32_t i : it.second)
        {
          DEBUG_CHECK_LT(i, ref_frame.landmark_handles_.size());
          const LandmarkHandle lm_h = ref_frame.landmark_handles_[i];
          if (isValidLandmarkHandle(lm_h)
              && landmarks_.type(lm_h) == LandmarkType::Seed)
          {
            landmarks_.type(lm_h) = LandmarkType::Opportunistic;
          }
        }
      }

      uint32_t num_inliers = 0u;
      for (auto it : stereo_matches)
      {
        num_inliers += it.second.size();
      }
      VLOG(3) << "Upgraded " << num_inliers << " stereo landmarks to opportunistic.";
    }
    else
    {
      auto t = timers_[Timer::detect_features].timeScope();
      for (size_t i = 0u; i < nframe_k->size(); ++i)
      {
        Frame& frame = nframe_k->at(i);
        frame.min_depth_ = FLAGS_vio_min_depth;
        frame.max_depth_ = FLAGS_vio_max_depth;
        frame.median_depth_ = scene_depth_ > FLAGS_vio_min_depth ?
                              scene_depth_ : FLAGS_vio_median_depth;
      }
      feature_initializer_->detectAndInitializeNewFeatures(
            *nframe_k, range(rig_->size()));

      //if (rig_->size() > 1u)
        //stereo_matcher_->matchStereoAndRejectOutliers(*nframe_k, states_.T_Bk_W());
    }
  }

  // Store frame as keyframe. Must be before seed update to do inter-frame updates.
  states_.setKeyframe(states_.nframeHandleK());
  addLandmarkObservations(*nframe_k, landmarks_);

  // Optimize landmarks.
  {
    auto t = timers_[Timer::optimize_landmarks].timeScope();
    optimizeLandmarks(*rig_, states_, *nframe_k, landmarks_);
  }
}


} // namespace ze
