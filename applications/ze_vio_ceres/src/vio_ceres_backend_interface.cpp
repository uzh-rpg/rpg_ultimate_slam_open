// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <iomanip>

#include <ze/vio_ceres/vio_ceres_backend_interface.hpp>

#include <ze/cameras/camera_rig.hpp>
#include <ze/common/timer.hpp>
#include <ze/vio_common/imu_integrator.hpp>
#include <ze/vio_common/landmark_table.hpp>
#include <ze/vio_common/nframe_table.hpp>
#include <ze/vio_frontend/frontend_base.hpp>

DEFINE_double(swe_imu_rate, 200, "IMU Rate [Hz]");
DEFINE_double(vio_ceres_accel_saturation, 176.0,
              "Acceleration saturation [m/s^2]");
DEFINE_double(vio_ceres_gyro_saturation, 7.8, "Gyro saturation [rad/s]");
DEFINE_double(gyro_noise_density, 12.0e-4,
              "Gyro noise density [rad/s/sqrt(Hz)]");
DEFINE_double(gyro_bias_random_walk, 0.03,
              "Gyro bias prior noise density [rad/s]");
DEFINE_double(acc_noise_density, 8.0e-3,
              "Accelerometer noise density [m/s^2/sqrt(Hz)]");
DEFINE_double(acc_bias_random_walk, 0.1,
              "Accelerometer bias prior noise density [m/s^2]");
DEFINE_double(vio_ceres_sigma_gyro_drift, 4.0e-6,
              "Gyro drift noise density [rad/s^2/sqrt(Hz)]");
DEFINE_double(vio_ceres_sigma_accel_drift, 4.0e-5,
              "Accelerometer drift noise density [m/s^3/sqrt(Hz)]");
DEFINE_double(vio_ceres_gravity, 9.81,
              "Earth graviational acceleration [m/s^2]");
DEFINE_bool(vio_ceres_add_velocity_prior, false,
            "Add velocity priors. [Experimental]");
DEFINE_double(extrinsics_sigma_abs_translation, 0.0,
              "Absolute translation sigma of camera extrinsics w.r.t to IMU Frame");
DEFINE_double(extrinsics_sigma_abs_orientation, 0.0,
              "Absolute orientation sigma of camera extrinsics w.r.t to IMU Frame");
DEFINE_double(extrinsics_sigma_rel_translation, 0.0,
              "Relative translation sigma (temporal) of camera extrinsics");
DEFINE_double(extrinsics_sigma_rel_orientation, 0.0,
              "Relative translation sigma (temporal) of camera extrinsics");
DEFINE_double(vio_ceres_max_optimization_time, -1.0,
              "Maximum time used to optimize [s]. Set negative to always do the "
              "maximum number of iterations.");

namespace ze {

CeresBackendInterface::CeresBackendInterface(
    LandmarkTable& landmarks_,
    NFrameTable& states,
    const std::shared_ptr<const CameraRig>& cam_rig,
    std::shared_ptr<ImuIntegrator> imu_integrator,
    const CeresBackendOptions& settings)
  : settings_(settings)
  , num_cameras_(cam_rig->size())
  , imu_integrator_(imu_integrator)
  , landmarks_(landmarks_)
  , states_(states)
{
  // Cameras -------------------------------------------------------------------
  // For now do not estimate extrinsics.
  ExtrinsicsEstimationParametersVec
      extrinsics_estimation_parameters(
        cam_rig->size(),
        ExtrinsicsEstimationParameters(FLAGS_extrinsics_sigma_abs_translation,
                                       FLAGS_extrinsics_sigma_abs_orientation,
                                       FLAGS_extrinsics_sigma_rel_translation,
                                       FLAGS_extrinsics_sigma_rel_orientation));
  backend_.addCameraRig(extrinsics_estimation_parameters, cam_rig);

  // IMU -----------------------------------------------------------------------
  ImuParameters imu_parameters;
  //! @todo set some default values in ImuParameters class.
  imu_parameters.T_BS.setIdentity();
  imu_parameters.a_max = FLAGS_vio_ceres_accel_saturation;
  imu_parameters.g_max = FLAGS_vio_ceres_gyro_saturation;
  imu_parameters.sigma_g_c = FLAGS_gyro_noise_density;
  imu_parameters.sigma_bg = FLAGS_gyro_bias_random_walk;
  imu_parameters.sigma_a_c = FLAGS_acc_noise_density;
  imu_parameters.sigma_ba = FLAGS_acc_bias_random_walk;
  imu_parameters.sigma_gw_c = FLAGS_vio_ceres_sigma_gyro_drift;
  imu_parameters.sigma_aw_c = FLAGS_vio_ceres_sigma_accel_drift;
  // Tau is not actually used...
  imu_parameters.tau = 3600.0;
  imu_parameters.g = FLAGS_vio_ceres_gravity;
  imu_parameters.a0.setZero();
  imu_parameters.rate = FLAGS_swe_imu_rate;
  backend_.addImu(imu_parameters);

  if (FLAGS_vio_ceres_max_optimization_time > 0.0)
  {
    backend_.setOptimizationTimeLimit(FLAGS_vio_ceres_max_optimization_time, 1);
  }
}

CeresBackendInterface::~CeresBackendInterface()
{
  if (thread_ != nullptr)
  {
    stopThread();
  }
}

void CeresBackendInterface::startThread()
{
  CHECK(thread_ == nullptr) << "Tried to start thread that is already running!";
  stop_thread_ = false;
  thread_.reset(new std::thread(&CeresBackendInterface::optimizationLoop,this));
}

void CeresBackendInterface::stopThread()
{
  VLOG(1) << "Interrupting and stopping optimization thread.";
  stop_thread_ = true;
  if (thread_ != nullptr)
  {
    wait_condition_.notify_all();
    thread_->join();
    thread_.reset();
  }
  VLOG(1) << "Thread stopped and joined.";
  timers_.saveToFile(FLAGS_log_dir, "timings_ceres.yaml");
  VLOG(1) << timers_;

  const size_t num_cams = backend_.getNumCameras();
  for (size_t i = 0; i < num_cams; ++i)
  {
    Transformation T_SCi;
    BackendId id = backend_.currentFrameId();
    backend_.getCameraSensorStatesFromNFrameId(id, i, T_SCi);
    VLOG(1) << "\nEstimated extrinsics for camera " << i << ":\n"
            << std::setprecision(15)
            << T_SCi;
  }
}

void CeresBackendInterface::addTrackedNFrame(
    const std::shared_ptr<NFrame>& nframe_k,
    const ImuStamps& imu_stamps,
    const ImuAccGyrContainer& imu_accgyr,
    const VioMotionType motion_type,
    const std::vector<LandmarkHandle>& lm_opportunistic,
    const std::vector<LandmarkHandle>& lm_persistent_new,
    const std::vector<LandmarkHandle>& lm_persistent_continued)
{
  if (stop_thread_)
  {
    return;
  }
  auto t_total = timers_[Timer::add_tracked_nframe].timeScope();
  {
    std::lock_guard<std::mutex> lock(mutex_backend_);
    BackendId nframe_id = createNFrameId(nframe_k->seq(), nframe_k->handle());

    // Keyframe decision -------------------------------------------------------
    bool as_keyframe = nframe_k->isKeyframe();
    // Starting up => need keyframes.
    if (!backend_initialized_)
    {
      if (backend_.numFrames() > 1)
      {
        backend_initialized_ = true;
      }
      as_keyframe = true;
    }

    // Adding new state to backend ---------------------------------------------
    {
      auto t = timers_[Timer::add_state].timeScope();
      if (!backend_.addStates(nframe_k, imu_stamps, imu_accgyr, as_keyframe))
      {
        LOG(ERROR) << "Failed to add state. Will drop frames.";
        return;
      }
    }

    if (motion_type == VioMotionType::NoMotion)
    {
      constexpr double zero_vel_prior_sigma = 1.0e-3;
      VLOG(5) << "Adding zero velocity prior.";
      if (!backend_.addVelocityPrior(nframe_id,
                                     Vector3::Zero(),
                                     zero_vel_prior_sigma))
      {
          LOG(ERROR) << "Failed to add a zero velocity prior!";
      }
    }

    // Adding zero velocity prior if we are in an appriate motion state.
    if (FLAGS_vio_ceres_add_velocity_prior && (
        motion_type == VioMotionType::RotationOnly ||
        motion_type == VioMotionType::Invalid))
    {
      constexpr double constant_vel_update_sigma = 1.0e-2;
      constexpr double zero_vel_prior_sigma = 1.0e-3;
      constexpr double max_velocity_for_zero_vel_prior = 0.1;
      constexpr double zero_vel_prior_dampening_factor = 0.4;
      SpeedAndBias propagated_speed_and_biases;
      backend_.getSpeedAndBiasFromNFrameId(nframe_id, propagated_speed_and_biases);
      if (propagated_speed_and_biases.head<3>().norm() <
          max_velocity_for_zero_vel_prior)
      {
        VLOG(5) << "Adding zero velocity prior.";
        if (!backend_.addVelocityPrior(nframe_id,
                                       Vector3::Zero(),
                                       zero_vel_prior_sigma))
        {
            LOG(ERROR) << "Failed to add a zero velocity prior!";
        }
      }
      else
      {
        VLOG(5) << "Adding constant velocity prior.";
        Vector3 damped_velocity =
            propagated_speed_and_biases.head<3>() *
            zero_vel_prior_dampening_factor;
        if (!backend_.addVelocityPrior(nframe_id,
                                       damped_velocity,
                                       constant_vel_update_sigma))
        {
          LOG(ERROR) << "Failed to add a constant velocity prior!";
        }
      }
    }

    // Adding new landmarks to backend -----------------------------------------
    if (nframe_k->isKeyframe())
    {
      auto t = timers_[Timer::add_landmarks].timeScope();
      VLOG(20) << "Adding new landmark tracks to backend.";
      addLandmarksAndObservationsToBackend(lm_opportunistic);
      addLandmarksAndObservationsToBackend(lm_persistent_new);
      VLOG(20) << "Adding continued observations to backend.";
      addNewestObservationsToBackend(lm_persistent_continued, nframe_k);
    }
    else
    {
      size_t num_new_observations = 0;
      for (size_t frame_idx = 0; frame_idx < nframe_k->size(); ++frame_idx)
      {
        for (size_t kp_idx = 0;
             kp_idx < nframe_k->at(frame_idx).landmark_handles_.size(); ++kp_idx)
        {
          LandmarkHandle& lmh = nframe_k->at(frame_idx).landmark_handles_[kp_idx];
          if (landmarks_.type(lmh) == LandmarkType::Persistent
              && landmarks_.isStored(lmh))
          {
            const Frame& frame = nframe_k->at(frame_idx);
            //! @todo use actual camera as template parameter.
            if (backend_.addObservation<ze::Camera>(
                  lmh, createNFrameId(nframe_k->seq(),nframe_k->handle()),
                  frame.px_vec_.col(kp_idx), frame.level_vec_(kp_idx),
                  frame_idx, kp_idx))
            {
              ++num_new_observations;
            }
          }
        }
      }
      VLOG(20) << "Added " << num_new_observations <<
                  " continued observation in non-KF to backend.";
    }

    //! @todo Set time limit for optimization.

    last_added_nframe_ = createNFrameId(nframe_k->seq(), nframe_k->handle());
  } // release backend.
  wait_condition_.notify_one();
}

void CeresBackendInterface::optimizationLoop()
{
  VLOG(1) << "Optimization thread started.";
  //! @todo make return of removed_landmarks optional in
  //! applyMarginalizationStrategy()
  LandmarkHandles removed_landmarks;
  while (!stop_thread_)
  {
    {
      std::unique_lock<std::mutex> lock(mutex_backend_);
      wait_condition_.wait(lock,
                           [&]
      {
        return last_added_nframe_ != last_optimized_nframe_ || stop_thread_;
      });
      if (stop_thread_)
      {
        return;
      }
      // Optimization ----------------------------------------------------------
      {
        auto t = timers_[Timer::optimize].timeScope();
        backend_.optimize(settings_.num_iterations,
                          settings_.num_threads,
                          settings_.verbose);
      }

      // Marginalization -------------------------------------------------------
      if (settings_.marginalize)
      {
        auto t = timers_[Timer::marginalization].timeScope();
        if (!backend_.applyMarginalizationStrategy(settings_.num_keyframes,
                                                   settings_.num_imu_frames,
                                                   removed_landmarks))
        {
          LOG(ERROR) << "Marginalization failed!";
        }
      }
      last_optimized_nframe_ = last_added_nframe_;
    } // release backend mutex.
  }
  VLOG(1) << "Optimization thread ended.";
}

bool CeresBackendInterface::updateStateWithResultFromLastOptimization(
    const bool wait_for_backend)
{
  if (last_updated_nframe_ == last_optimized_nframe_)
  {
    VLOG(100) << "No state update available.";
    return false;
  }

  if (stop_thread_)
  {
    return false;
  }

  if (wait_for_backend)
  {
    while (last_added_nframe_ != last_optimized_nframe_)
      usleep(100);
  }
  else if (last_added_nframe_ != last_optimized_nframe_)
  {
    VLOG(100) << "New frame has not been processed yet!";
    return false;
  }

  auto t = timers_[Timer::update_state].timeScope();

  std::lock_guard<std::mutex> lock(mutex_backend_);
  VLOG(100) << "Updating states with latest results from ceres optimizer.";

  // The frontend might have run ahead and already started work on new frames.
  // We need to update their poses as well.
  const Transformation T_Bk_Blkf = states_.T_Bk_W() * states_.T_Blkf_W().inverse();
  const Transformation T_Bkm1_Blkf = states_.T_Bkm1_W() * states_.T_Blkf_W().inverse();

  std::vector<BackendId> frame_ids = backend_.getAllFrameIds();
  states_.T_B_W_backend().clear();
  Transformation T_WS;
  SpeedAndBias speed_and_bias;
  for (BackendId id : frame_ids)
  {
    NFrameHandle handle = id.nframeHandle();
    if (states_.isStored(handle))
    {
      if(!backend_.get_T_WS(id, T_WS))
      {
        LOG(ERROR) << "Could not get pose estimate from ceres optimizer!";
        continue;
      }
      Transformation T_B_W = T_WS.inverse();
      states_.T_B_W(handle) = T_B_W;
      states_.T_B_W_backend().push_back(T_B_W);

      if (id == last_added_nframe_)
      {
        if (!backend_.getSpeedAndBiasFromNFrameId(id, speed_and_bias))
        {
          LOG(ERROR) << "Could not get speed and bias estimate from ceres optimizer";
          continue;
        }
        states_.v_W(handle) = speed_and_bias.head<3>();
        states_.accBias() = speed_and_bias.tail<3>();
        states_.gyrBias() = speed_and_bias.segment<3>(3);
        imu_integrator_->setBiases(states_.accBias(), states_.gyrBias());
      }
    }
  }

  if (FLAGS_vio_add_every_nth_frame_to_backend <= 0)
  {
    // Reset pose for non-keyframes
    if (!states_.nframeK()->isKeyframe())
    {
      states_.T_Bk_W() = T_Bk_Blkf * states_.T_Blkf_W();
      states_.T_Bk_W().getRotation().normalize();
    }
    if (!states_.nframeKm1()->isKeyframe())
    {
      states_.T_Bkm1_W() = T_Bkm1_Blkf * states_.T_Blkf_W();
      states_.T_Bkm1_W().getRotation().normalize();
    }
  }

  last_updated_nframe_ = last_optimized_nframe_;
  return true;
}

bool CeresBackendInterface::addLandmarksAndObservationsToBackend(
    const std::vector<LandmarkHandle>& new_landmarks)
{
  HomPosition landmark;
  bool success = true;
  for (auto& lm_h : new_landmarks)
  {
    if (!landmarks_.isActive(lm_h))
    {
      continue;
    }
    landmark << landmarks_.p_W(lm_h), 1.0;
    if (!backend_.addLandmark(lm_h, landmark))
    {
      success = false;
      LOG(ERROR) << "Could not add landmark to backend!";
      continue;
    }
    const LandmarkObsVec& obs_vec = landmarks_.obs(lm_h);
    for (const LandmarkObs& obs : obs_vec)
    {
      //! @todo in the backend consistency is checked by checking whether
      //! the observation is also saved in the nframe. However non keyframes
      //! do not save these observations. Therefore the check fails. Either
      //! we disable the check / change it or just accept that if a new landmark
      //! track is found it will only add observations to keyframes.
      if (states_.nframe(obs.nframe_handle_)->isKeyframe())
      {
        const Frame& frame = states_.nframe(obs.nframe_handle_)->at(obs.frame_idx_);
        if (!backend_.addObservation<ze::Camera>(
              lm_h,
              createNFrameId(states_.nframe(obs.nframe_handle_)->seq(),
                             obs.nframe_handle_),
              frame.px_vec_.col(obs.keypoint_idx_),
              frame.level_vec_(obs.keypoint_idx_),
              obs.frame_idx_, obs.keypoint_idx_))
        {
          success = false;
          LOG(WARNING) << "Failed to add an observation!";
        }
      }
    }
  }
  return success;
}

bool CeresBackendInterface::addNewestObservationsToBackend(
    const std::vector<LandmarkHandle>& continued_landmarks,
    const NFrame::ConstPtr& nframe)
{
  bool success = true;
  const NFrameHandle handle = nframe->handle();
  const BackendId nframe_id = createNFrameId(nframe->seq(), handle);
  for (const LandmarkHandle& lm_h : continued_landmarks)
  {
    //! @todo find out actual camera model such that there is no vtable lookup
    //! in the reprojection error term.
    const LandmarkObsVec& obs_vec = landmarks_.obs(lm_h);
    size_t count = 0;
    for (auto obs_it = obs_vec.rbegin(); obs_it != obs_vec.rend(); ++obs_it)
    {
      if (obs_it->nframe_handle_ != handle)
      {
        continue;
      }
      const Frame& frame = nframe->at(obs_it->frame_idx_);
      if (!backend_.addObservation<ze::Camera>(
            lm_h, nframe_id,
            frame.px_vec_.col(obs_it->keypoint_idx_),
            frame.level_vec_(obs_it->keypoint_idx_),
            obs_it->frame_idx_,
            obs_it->keypoint_idx_))
      {
        success = false;
        LOG(WARNING) << "Failed to add an observation!";
      }
      ++count;
      if (count == num_cameras_)
      {
        break;
      }
    }
  }
  return success;
}


} // namespace ze
