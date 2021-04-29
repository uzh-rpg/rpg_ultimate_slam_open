/*********************************************************************************
This code is provided for internal research and development purposes by Huawei solely,
in accordance with the terms and conditions of the research collaboration agreement of May 7, 2020.
Any further use for commercial purposes is subject to a written agreement.
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2016, ETH Zurich, Wyss Zurich, Zurich Eye
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Dec 30, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *    Modified: Zurich Eye
 *********************************************************************************/

/**
 * @file ze/Estimator.hpp
 * @brief Header file for the Estimator class. This does all the backend work.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#pragma once

#include <array>
#include <memory>
#include <mutex>

#pragma diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Eigen 3.2.7 uses std::binder1st and std::binder2nd which are deprecated since c++11
// Fix is in 3.3 devel (http://eigen.tuxfamily.org/bz/show_bug.cgi?id=872).
#include <ceres/ceres.h>
#pragma diagnostic pop

#include <ze/common/logging.hpp>
#include <ze/nlls/map.hpp>
#include <ze/nlls/estimator_types.hpp>
#include <ze/vio_common/nframe.hpp>

DECLARE_double(vio_acc_bias_init_x);
DECLARE_double(vio_acc_bias_init_y);
DECLARE_double(vio_acc_bias_init_z);
DECLARE_double(vio_gyr_bias_init_x);
DECLARE_double(vio_gyr_bias_init_y);
DECLARE_double(vio_gyr_bias_init_z);

namespace ze {

// fwd:
namespace nlls {
class MarginalizationError;
class CeresIterationCallback;
}

struct States
{
  // ordered from oldest to newest.
  std::vector<BackendId> ids;
  std::vector<bool> is_keyframe;
  std::vector<int64_t> timestamps;

  States() = default;

  void addState(BackendId id, bool keyframe, int64_t timestamp)
  {
    DEBUG_CHECK(id.type() == IdType::NFrame);
    ids.push_back(id);
    is_keyframe.push_back(keyframe);
    timestamps.push_back(timestamp);
  }

  bool removeState(BackendId id)
  {
    auto slot = findSlot(id);
    if (slot.second)
    {
      ids.erase(ids.begin() + slot.first);
      is_keyframe.erase(is_keyframe.begin() + slot.first);
      timestamps.erase(timestamps.begin() + slot.first);
      return true;
    }
    return false;
  }

  std::pair<size_t, bool> findSlot(BackendId id) const
  {
    for (size_t i = 0; i < ids.size(); ++i)
    {
      if (ids[i] == id)
      {
        return std::make_pair(i, true);
      }
    }
    return std::make_pair(0, false);
  }
};


//! The estimator class
/*!
 The estimator class. This does all the backend work.
 Frames:
 W: World
 B: Body
 C: Camera
 S: Sensor (IMU)
 */
class Estimator
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Estimator();

  /**
   * @brief Constructor if a ceres map is already available.
   * @param map_ptr Shared pointer to ceres map.
   */
  explicit Estimator(std::shared_ptr<nlls::Map> map_ptr);
  ~Estimator();

  /// @name Sensor configuration related
  ///@{
  /**
   * @brief Add a camera rig to the configuration. Sensors can only be added and
   *        never removed.
   * @param extrinsics_estimation_parameters The parameters that tell how to
   *        estimate extrinsics.
   * @param camera_rig Shared pointer to the camera rig.
   */
  void addCameraRig(
      const ExtrinsicsEstimationParametersVec& extrinsics_estimation_parameters,
      const ze::CameraRig::ConstPtr& camera_rig);

  /**
   * @brief Add an IMU to the configuration.
   * @warning Currently there is only one IMU supported.
   * @param imu_parameters The IMU parameters.
   * @return index of IMU.
   */
  int addImu(const ze::ImuParameters& imu_parameters);

  /**
   * @brief Remove all cameras from the configuration
   */
  void clearCameras();

  /**
   * @brief Remove all IMUs from the configuration.
   */
  void clearImus();

  /// @}

  /**
   * @brief Add a pose to the state.
   * @param nframe Matched nFrame.
   * @param imu_stamps Timestamps of IMU measurements from last state to new one.
   * @param imu_acc_gyr IMU measurements from last state to new one.
   * @param as_keyframe Is this new frame a keyframe?
   * @return True if successful.
   */
  bool addStates(const NFrame::ConstPtr& nframe,
                 const ImuStamps& imu_stamps,
                 const ImuAccGyrContainer& imu_acc_gyr,
                 bool as_keyframe);

  /**
   * @brief Prints state information to buffer.
   * @param pose_id The pose Id for which to print.
   * @param buffer The puffer to print into.
   */
  void printStates(BackendId pose_id, std::ostream& buffer) const;

  /**
   * @brief Add a landmark.
   * @param landmark_handle Handle of the new landmark.
   * @param landmark Homogeneous coordinates of landmark in W-frame.
   * @return True if successful.
   */
  bool addLandmark(LandmarkHandle landmark_handle,
                   const Eigen::Vector4d& landmark);

  bool addVelocityPrior(BackendId frame_id,
                        const Eigen::Vector3d& velocity,
                        double sigma);

  /**
   * @brief Add an observation to a landmark.
   * \tparam GEOMETRY_TYPE The camera geometry type for this observation.
   * @param landmark_handle Handle of landmark.
   * @param nframe_id ID of the nframe.
   * @param measurement Keypoint measurement.
   * @param level Pyramid level of the observation.
   * @param cam_idx ID of camera frame where the landmark was observed.
   * @param keypoint_idx ID of keypoint corresponding to the landmark.
   * @return Residual block ID for that observation.
   */
  template<class GEOMETRY_TYPE>
  ceres::ResidualBlockId addObservation(LandmarkHandle landmark_handle,
                                        BackendId nframe_id,
                                        const Eigen::Vector2d& measurement,
                                        int8_t level,
                                        size_t cam_idx,
                                        size_t keypoint_idx);
  /**
   * @brief Remove an observation from a landmark, if available.
   * @param landmark_id ID of landmark.
   * @param pose_id ID of pose where the landmark was observed.
   * @param cam_idx ID of camera frame where the landmark was observed.
   * @param keypoint_idx ID of keypoint corresponding to the landmark.
   * @return True if observation was present and successfully removed.
   */
  bool removeObservation(BackendId landmark_id, BackendId pose_id,
                         size_t cam_idx, size_t keypoint_idx);

  /**
   * @brief Applies the dropping/marginalization strategy according to the RSS'13/IJRR'14 paper.
   *        The new number of frames in the window will be numKeyframes+numImuFrames.
   * @param num_keyframes Number of keyframes.
   * @param num_imu_frames Number of frames in IMU window.
   * @param[out] removed_landmarks Get the landmarks that were removed by this
   *                               operation.
   * @return True if successful.
   */
  bool applyMarginalizationStrategy(size_t num_keyframes, size_t num_imu_frames,
                                    LandmarkHandles& removed_landmarks);

  /**
   * @brief Initialise pose from IMU measurements. For convenience as static.
   * @param[in] imu_stamps Timestamps of the IMU measurements.
   * @param[in] imu_acc_gyr IMU measurements.
   * @param[out] T_WS initialised pose.
   * @return True if successful.
   */
  static bool initPoseAndBiasFromImu(const ImuStamps& imu_stamps,
                                     const ImuAccGyrContainer& imu_acc_gyr,
                                     const double g,
                                     ze::Transformation& T_WS,
                                     SpeedAndBias &speed_and_bias);

  /**
   * @brief Start ceres optimization.
   * @param[in] num_iter Maximum number of iterations.
   * @param[in] num_threads Number of threads.
   * @param[in] verbose Print out optimization progress and result, if true.
   */
  void optimize(size_t num_iter, size_t num_threads = 1, bool verbose = false);

  /**
   * @brief Set a time limit for the optimization process.
   * @param[in] time_limit Time limit in seconds. If timeLimit < 0 the time limit is removed.
   * @param[in] min_iterations minimum iterations the optimization process should do
   *            disregarding the time limit.
   * @return True if successful.
   */
  bool setOptimizationTimeLimit(double time_limit, int min_iterations);

  /**
   * @brief Checks whether the landmark is added to the estimator.
   * @param landmark_id The ID.
   * @return True if added.
   */
  bool isLandmarkAdded(BackendId landmark_id) const
  {
    bool isAdded = landmarks_map_.find(landmark_id) != landmarks_map_.end();
    DEBUG_CHECK(isAdded == map_ptr_->parameterBlockExists(landmark_id.asInteger()))
        << "id="<<landmark_id<<" inconsistent. isAdded = " << isAdded;
    return isAdded;
  }

  /**
   * @brief Checks whether the landmark is initialized.
   * @param landmark_id The ID.
   * @return True if initialised.
   */
  bool isLandmarkInitialized(BackendId landmark_id) const;

  /// @name Getters
  ///\{

  size_t getNumCameras() const { return extrinsics_estimation_parameters_.size(); }

  /**
   * @brief Get a specific landmark.
   * @param[in]  landmark_id ID of desired landmark.
   * @param[out] map_point Landmark information, such as quality, coordinates etc.
   * @return True if successful.
   */
  bool getLandmark(BackendId landmark_id, MapPoint& map_point) const;

  /**
   * @brief Get a copy of all the landmarks as a PointMap.
   * @param[out] landmarks The landmarks.
   * @return number of landmarks.
   */
  size_t getLandmarks(PointMap& landmarks) const;

  /**
   * @brief Get a copy of all the landmark in a MapPointVector. This is for legacy support.
   *        Use getLandmarks(ze::PointMap&) if possible.
   * @param[out] landmarks A vector of all landmarks.
   * @see getLandmarks().
   * @return number of landmarks.
   */
  size_t getLandmarks(ze::MapPointVector& landmarks) const;

  bool get_T_WS(BackendId id, Transformation& T_WS) const;

  /**
   * @brief Get pose for a given pose ID.
   * @param[in] handle Handle of the NFrame.
   * @param[in] seq Sequence of the NFrame.
   * @param[out] T_WS Homogeneous transformation of this pose.
   * @return True if successful.
   */
  bool get_T_WS(NFrameHandle handle,
                int32_t seq,
                Transformation& T_WS) const
  {
    return get_T_WS(createNFrameId(seq, handle), T_WS);
  }


  bool getSpeedAndBiasFromNFrameId(BackendId id,
                                   SpeedAndBias& speed_and_bias) const;

  /**
   * @brief Get speeds and IMU biases for a given pose ID.
   * @param[in]  handle NFrame handle of pose to get speeds and biases for.
   * @param[in]  nframe_seq Sequence of the NFrame.
   * @param[out] speed_and_bias Speed And bias requested.
   * @return True if successful.
   */
  bool getSpeedAndBiasFromNFrameHandle(NFrameHandle handle,
                                       int32_t nframe_seq,
                                       SpeedAndBias& speed_and_bias) const
  {
    return getSpeedAndBiasFromNFrameId(createNFrameId(nframe_seq, handle),
                                       speed_and_bias);
  }

  /**
   * @brief Get camera states for a given pose ID.
   * @param[in]  pose_id ID of pose to get camera state for.
   * @param[in]  camera_idx index of camera to get state for.
   * @param[out] T_SCi Homogeneous transformation from sensor (IMU) frame to camera frame.
   * @return True if successful.
   */
  bool getCameraSensorStatesFromNFrameId(BackendId pose_id, size_t camera_idx,
                                         Transformation& T_SCi) const;

  /// @brief Get the number of states/frames in the estimator.
  /// \return The number of frames.
  size_t numFrames() const
  {
    return states_.ids.size();
  }

  /// @brief Get the number of landmarks in the estimator
  /// \return The number of landmarks.
  size_t numLandmarks() const
  {
    return landmarks_map_.size();
  }

  /// @brief Get the ID of the current keyframe.
  /// @todo return handle
  /// \return The ID of the current keyframe.
  BackendId currentKeyframeId() const;

  /**
   * @brief Get the ID of an older frame.
   * @param[in] age age of desired frame. 0 would be the newest frame added to the state.
   * @return ID of the desired frame or 0 if parameter age was out of range.
   */
  BackendId frameIdByAge(size_t age) const;

  /// @brief Get the ID of the newest frame added to the state.
  /// \return The ID of the current frame.
  BackendId currentFrameId() const;

  NFrameHandle currentFrameHandle() const;

  std::vector<BackendId> getAllFrameIds() const;

  ///@}

  /**
   * @brief Checks if a particular frame is a keyframe.
   * @param[in] nframe_handle Handle of frame to check.
   * @param[in] seq Sequence of frame to check.
   * @return True if the frame is a keyframe.
   */
  bool isKeyframe(NFrameHandle nframe_handle, int32_t seq) const
  {
    BackendId id = createNFrameId(seq, nframe_handle);
    auto slot = states_.findSlot(id);
    DEBUG_CHECK(slot.second) << "Frame with ID " << id << " does not exist.";
    return states_.is_keyframe[slot.first];
  }

  /**
   * @brief Checks if a particular frame is still in the IMU window.
   * @param[in] frame_id ID of frame to check.
   * @return True if the frame is in IMU window.
   */
  bool isInImuWindow(BackendId frame_id) const;

  /// @name Getters
  /// @{
  /**
   * @brief Get the timestamp for a particular frame.
   * @param[in] frame_id ID of frame.
   * @return Timestamp of frame.
   */
  int64_t timestamp(BackendId frame_id) const
  {
    auto slot = states_.findSlot(frame_id);
    DEBUG_CHECK(slot.second) << "Frame with ID " << frame_id
                             << " does not exist.";
    if (slot.second)
    {
      return states_.timestamps[slot.second];
    }
    return 0;
  }

  ///@}
  /// @name Setters
  ///@{
  /**
   * @brief Set pose for a given pose ID.
   * @param[in] pose_id ID of the pose that should be changed.
   * @param[in] T_WS new homogeneous transformation.
   * @return True if successful.
   */
  bool set_T_WS(BackendId pose_id, const ze::Transformation& T_WS);

  /**
   * @brief Set the speeds and IMU biases for a given pose ID.
   * @param[in] pose_id ID of the pose to change corresponding speeds and biases for.
   * @param[in] speed_and_bias new speeds and biases.
   * @return True if successful.
   */
  bool setSpeedAndBiasFromNFrameId(BackendId pose_id,
                                   const SpeedAndBias& speed_and_bias);

  /**
   * @brief Set the transformation from sensor to camera frame for a given pose ID.
   * @todo use handle
   * @warning This accesses the optimization graph, so not very fast.
   * @param[in] pose_id ID of the pose to change corresponding camera states for.
   * @param[in] camera_idx Index of camera to set state for.
   * @param[in] T_SCi new homogeneous transformation from sensor (IMU) to camera frame.
   * @return True if successful.
   */
  bool setCameraSensorStates(BackendId pose_id, uint8_t camera_idx,
                              const Transformation& T_SCi);

  /// @brief Set the homogeneous coordinates for a landmark.
  /// @todo use handle
  /// @param[in] landmark_id The landmark ID.
  /// @param[in] landmark Homogeneous coordinates of landmark in W-frame.
  /// @return True if successful.
  bool setLandmark(BackendId landmark_id, const Eigen::Vector4d& landmark);

  /// @brief Set the landmark initialization state.
  /// @todo use handle
  /// @param[in] landmarkId The landmark ID.
  /// @param[in] initialized Whether or not initialised.
  void setLandmarkInitialized(BackendId landmark_id, bool initialized);

  /// @brief Set whether a frame is a keyframe or not.
  /// @todo use handle
  /// @param[in] frame_id The frame ID.
  /// @param[in] isKeyframe Whether or not keyrame.
  void setKeyframe(BackendId frame_id, bool isKeyframe)
  {
    auto slot = states_.findSlot(frame_id);
    if (slot.second)
    {
      states_.is_keyframe[slot.first] = isKeyframe;
    }
  }

  /// @brief set ceres map
  /// @param[in] mapPtr The pointer to the nlls::Map.
  void setMap(std::shared_ptr<nlls::Map> map_ptr)
  {
    map_ptr_ = map_ptr;
  }
  ///@}

 private:

  /**
   * @brief Remove an observation from a landmark.
   * @param residual_block_id Residual ID for this landmark.
   * @return True if successful.
   */
  bool removeObservation(ceres::ResidualBlockId residual_block_id);

  // getters
  std::pair<Transformation, bool> getPoseEstimate(BackendId id) const;

  std::pair<SpeedAndBias, bool> getSpeedAndBiasEstimate(BackendId id) const;

  // setters
  bool setPoseEstimate(BackendId id, const Transformation& pose);

  bool setSpeedAndBiasEstimate(BackendId id, const SpeedAndBias& sab);

  ze::CameraRig::ConstPtr camera_rig_;
  //! If we do not estimate extrinsics, then these are the parameter block IDs
  //! for all extrinsics.
  std::vector<BackendId> constant_extrinsics_ids_;
  bool estimate_temporal_extrinsics_ {false};

  // the following keeps track of all the states at different time instances (key=poseId)
  States states_; ///< Buffer for currently considered states.
  std::shared_ptr<nlls::Map> map_ptr_; ///< The underlying ze::Map.

  // this is the reference pose
  BackendId reference_pose_id_; ///< The pose ID of the reference (currently not changing)

  // the following are updated after the optimization
  ze::PointMap landmarks_map_; ///< Contains all the current landmarks (synched after optimisation).
  mutable std::mutex landmarks_mutex_;  ///< Regulate access of landmarksMap_.

  // parameters
  ExtrinsicsEstimationParametersVec extrinsics_estimation_parameters_; ///< Extrinsics parameters.
  std::vector<ze::ImuParameters, Eigen::aligned_allocator<ze::ImuParameters> >
  imu_parameters_; ///< IMU parameters.

  // loss function for reprojection errors
  std::shared_ptr< ceres::LossFunction> cauchy_loss_function_ptr_; ///< Cauchy loss.
  std::shared_ptr< ceres::LossFunction> huber_loss_function_ptr_; ///< Huber loss.

  // the marginalized error term
  std::shared_ptr<nlls::MarginalizationError> marginalization_error_ptr_; ///< The marginalisation class
  ceres::ResidualBlockId marginalization_residual_id_; ///< Remembers the marginalisation object's Id

  // ceres iteration callback object
  std::unique_ptr<nlls::CeresIterationCallback> ceres_callback_; ///< Maybe there was a callback registered, store it here.
};

}  // namespace ze

#include <ze/nlls/estimator_impl.hpp>
