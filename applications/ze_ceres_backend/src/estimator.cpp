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
 * @file Estimator.cpp
 * @brief Source file for the Estimator class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <ze/nlls/estimator.hpp>

#include <ze/common/time_conversions.hpp>
#include <ze/nlls/ceres_iteration_callback.hpp>
#include <ze/nlls/imu_error.hpp>
#include <ze/nlls/marginalization_error.hpp>
#include <ze/nlls/pose_error.hpp>
#include <ze/nlls/pose_parameter_block.hpp>
#include <ze/nlls/relative_pose_error.hpp>
#include <ze/nlls/reprojection_error.hpp>
#include <ze/nlls/speed_and_bias_error.hpp>

/// \brief ze Main namespace of this package.
namespace ze {

// Constructor if a ceres map is already available.
Estimator::Estimator(
    std::shared_ptr<nlls::Map> map_ptr)
    : map_ptr_(map_ptr),
      cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
      huber_loss_function_ptr_(new ceres::HuberLoss(1)),
      marginalization_residual_id_(0)
{}

// The default constructor.
Estimator::Estimator()
    : Estimator(std::make_shared<nlls::Map>())
{}

Estimator::~Estimator()
{}

// Add a camera to the configuration. Sensors can only be added and never removed.
void Estimator::addCameraRig(
    const ExtrinsicsEstimationParametersVec& extrinsics_estimation_parameters,
    const ze::CameraRig::ConstPtr& camera_rig)
{
  CHECK(camera_rig != nullptr);
  CHECK_EQ(camera_rig->size(), extrinsics_estimation_parameters.size());
  extrinsics_estimation_parameters_ = extrinsics_estimation_parameters;
  camera_rig_ = camera_rig;
  constant_extrinsics_ids_.resize(camera_rig_->size());

  for (size_t i = 0; i < extrinsics_estimation_parameters_.size(); ++i)
  {
    if (extrinsics_estimation_parameters_[i].sigma_c_relative_translation > 1e-12
        ||
        extrinsics_estimation_parameters_[i].sigma_c_relative_orientation > 1e-12)
    {
      //! @todo This has to be changed to a vector of bools if for certain
      //! cameras we estimate the temporal changes of the extrinsics and for
      //! others we do not.
      estimate_temporal_extrinsics_ = true;
      LOG(WARNING) << "Estimating temporal changes of extrinsics seems to cause"
                   << " jumps in the estimations!";
    }
  }
}

// Add an IMU to the configuration.
int Estimator::addImu(const ImuParameters& imu_parameters)
{
  if(imu_parameters_.size() > 1)
  {
    LOG(ERROR) << "only one IMU currently supported";
    return -1;
  }
  imu_parameters_.push_back(imu_parameters);
  return imu_parameters_.size() - 1;
}

// Remove all cameras from the configuration
void Estimator::clearCameras()
{
  extrinsics_estimation_parameters_.clear();
}

// Remove all IMUs from the configuration.
void Estimator::clearImus()
{
  imu_parameters_.clear();
}

// Add a pose to the state.
bool Estimator::addStates(
    const NFrame::ConstPtr& nframe,
    const ImuStamps& imu_stamps,
    const ImuAccGyrContainer& imu_acc_gyr,
    bool as_keyframe)
{
  BackendId nframe_id = createNFrameId(nframe->seq(),
                                       nframe->handle());
  const int64_t timestamp = nframe->timestamp();
  int64_t last_timestamp = 0;
  VLOG(20) << "Adding state to estimator. nFrameHandle: " << nframe->handle()
           << " seq " << nframe->seq()
           << " with ze-id: " << std::hex << nframe_id << std::dec
           << " num IMU measurements: " << imu_stamps.rows()
           << " as keyframe? " << as_keyframe;
  // note: this is before matching...
  // TODO !!
  Transformation T_WS;
  SpeedAndBias speed_and_bias;
  if (states_.ids.empty())
  {
    // in case this is the first frame ever, let's initialize the pose:
    bool success0 = initPoseAndBiasFromImu(
          imu_stamps,
          imu_acc_gyr,
          imu_parameters_.at(0).g,
          T_WS,
          speed_and_bias);
    DEBUG_CHECK(success0)
        << "pose could not be initialized from imu measurements.";
    if (!success0)
      return false;
  }
  else
  {
    last_timestamp = states_.timestamps.back();
    // get the previous states
    BackendId T_WS_id = states_.ids.back();
    BackendId speed_and_bias_id = changeIdType(T_WS_id, IdType::ImuStates);
    T_WS =
        std::static_pointer_cast<nlls::PoseParameterBlock>(
          map_ptr_->parameterBlockPtr(T_WS_id.asInteger()))->estimate();
    speed_and_bias =
        std::static_pointer_cast<nlls::SpeedAndBiasParameterBlock>(
            map_ptr_->parameterBlockPtr(speed_and_bias_id.asInteger()))->estimate();

    // propagate pose and speedAndBias
    int num_used_imu_measurements =
        nlls::ImuError::propagation(
          imu_stamps, imu_acc_gyr, imu_parameters_.at(0), T_WS, speed_and_bias,
          last_timestamp, timestamp);
    DEBUG_CHECK(num_used_imu_measurements > 1) << "propagation failed";
    if (num_used_imu_measurements < 1)
    {
      LOG(INFO) << "numUsedImuMeasurements=" << num_used_imu_measurements;
      return false;
    }
  }

  // check if id was used before
  DEBUG_CHECK(!states_.findSlot(nframe_id).second)
      << "pose ID" << nframe_id << " was used before!";

  // create global states
  std::shared_ptr<nlls::PoseParameterBlock> pose_parameter_block =
      std::make_shared<nlls::PoseParameterBlock>(T_WS, nframe_id.asInteger(),
                                                 timestamp);

  if(states_.ids.empty())
  {
    reference_pose_id_ = nframe_id; // set this as reference pose
    if (!map_ptr_->addParameterBlock(pose_parameter_block, nlls::Map::Pose6d))
    {
      return false;
    }
  }
  else
  {
    if (!map_ptr_->addParameterBlock(pose_parameter_block, nlls::Map::Pose6d))
    {
      return false;
    }
  }

  // add to buffer
  states_.addState(nframe_id, as_keyframe, timestamp);

  // the following will point to the last states:
  const size_t last_slot_number = states_.ids.size() - 2;

  // initialize new sensor states
  // cameras:
  std::vector<BackendId> extrinsics_ids = constant_extrinsics_ids_;
  for (size_t i = 0; i < extrinsics_estimation_parameters_.size(); ++i)
  {
    if (estimate_temporal_extrinsics_ || states_.ids.size() == 1)
    {
      const Transformation T_S_C = camera_rig_->T_C_B(i).inverse();
      extrinsics_ids[i] = changeIdType(nframe_id, IdType::Extrinsics, i);
      std::shared_ptr<nlls::PoseParameterBlock> extrinsics_parameter_block =
          std::make_shared<nlls::PoseParameterBlock>(
            T_S_C, extrinsics_ids[i].asInteger(), timestamp);
      if (!map_ptr_->addParameterBlock(extrinsics_parameter_block,
                                      nlls::Map::Pose6d))
      {
        return false;
      }
      if (states_.ids.size() == 1)
      {
        constant_extrinsics_ids_ = extrinsics_ids;
      }
    }
    // update the states info
  }

  // IMU states are automatically propagated.
  for (size_t i=0; i<imu_parameters_.size(); ++i)
  {
    BackendId id = changeIdType(nframe_id, IdType::ImuStates);
    std::shared_ptr<nlls::SpeedAndBiasParameterBlock> speed_and_bias_parameter_block =
        std::make_shared<nlls::SpeedAndBiasParameterBlock>(speed_and_bias,
                                                           id.asInteger(),
                                                           timestamp);

    if(!map_ptr_->addParameterBlock(speed_and_bias_parameter_block))
    {
      return false;
    }
  }

  // depending on whether or not this is the very beginning, we will add priors
  // or relative terms to the last state:
  if (states_.ids.size() == 1)
  {
    // let's add a prior
    Eigen::Matrix<double,6,6> information = Eigen::Matrix<double,6,6>::Zero();
    information(5,5) = 1.0e8;
    information(0,0) = 1.0e8;
    information(1,1) = 1.0e8;
    information(2,2) = 1.0e8;
    std::shared_ptr<nlls::PoseError > pose_error =
        std::make_shared<nlls::PoseError>(T_WS, information);
    map_ptr_->addResidualBlock(pose_error, nullptr, pose_parameter_block);
    //mapPtr_->isJacobianCorrect(id2,1.0e-6);

    // sensor states
    for (size_t i = 0; i < extrinsics_estimation_parameters_.size(); ++i)
    {
      double translation_stdev = extrinsics_estimation_parameters_[i].sigma_absolute_translation;
      double translation_variance = translation_stdev * translation_stdev;
      double rotation_stdev = extrinsics_estimation_parameters_[i].sigma_absolute_orientation;
      double rotation_variance = rotation_stdev * rotation_stdev;
      if(translation_variance > 1.0e-16 || rotation_variance > 1.0e-16)
      {
        const Transformation T_SC = camera_rig_->T_C_B(i).inverse();
        std::shared_ptr<nlls::PoseError > camera_pose_error =
            std::make_shared<nlls::PoseError>(T_SC, translation_variance,
                                              rotation_variance);
        // add to map
        map_ptr_->addResidualBlock(
            camera_pose_error,
            nullptr,
            map_ptr_->parameterBlockPtr(extrinsics_ids[i].asInteger()));
        //mapPtr_->isJacobianCorrect(id,1.0e-6);
      }
      else
      {
        map_ptr_->setParameterBlockConstant(extrinsics_ids[i].asInteger());
      }
    }
    for (size_t i = 0; i < imu_parameters_.size(); ++i)
    {
      // get these from parameter file
      const double sigma_bg = imu_parameters_.at(0).sigma_bg;
      const double sigma_ba = imu_parameters_.at(0).sigma_ba;
      std::shared_ptr<nlls::SpeedAndBiasError > speed_and_bias_error =
          std::make_shared<nlls::SpeedAndBiasError>(
            speed_and_bias, 1.0, sigma_bg*sigma_bg, sigma_ba*sigma_ba);
      // add to map
      map_ptr_->addResidualBlock(
          speed_and_bias_error,
          nullptr,
          map_ptr_->parameterBlockPtr(
              changeIdType(nframe_id, IdType::ImuStates).asInteger()));
      //mapPtr_->isJacobianCorrect(id,1.0e-6);
    }
  }
  else
  {
    // add IMU error terms
    const BackendId last_nframe_id = states_.ids[last_slot_number];
    for (size_t i = 0; i < imu_parameters_.size(); ++i)
    {
      std::shared_ptr<nlls::ImuError> imuError =
          std::make_shared<nlls::ImuError>(imu_stamps, imu_acc_gyr,
                                           imu_parameters_.at(i),
                                           last_timestamp,
                                           timestamp);
      /*ceres::ResidualBlockId id = */map_ptr_->addResidualBlock(
          imuError,
          nullptr,
          map_ptr_->parameterBlockPtr(last_nframe_id.asInteger()),
          map_ptr_->parameterBlockPtr(
              changeIdType(last_nframe_id, IdType::ImuStates).asInteger()),
          map_ptr_->parameterBlockPtr(nframe_id.asInteger()),
          map_ptr_->parameterBlockPtr(
              changeIdType(nframe_id, IdType::ImuStates).asInteger()));
      //imuError->setRecomputeInformation(false);
      //mapPtr_->isJacobianCorrect(id,1.0e-9);
      //imuError->setRecomputeInformation(true);
    }

    // add relative sensor state errors
    for (size_t i = 0; i < extrinsics_estimation_parameters_.size(); ++i)
    {
      if(extrinsics_ids[i] != constant_extrinsics_ids_[i])
      {
        // i.e. they are different estimated variables, so link them with a temporal error term
        double dt = nanosecToSecTrunc(timestamp - last_timestamp);
        double translation_sigma_c =
            extrinsics_estimation_parameters_[i].sigma_c_relative_translation;
        double translation_variance = translation_sigma_c * translation_sigma_c * dt;
        double rotation_sigma_c =
            extrinsics_estimation_parameters_[i].sigma_c_relative_orientation;
        double rotation_variance = rotation_sigma_c * rotation_sigma_c * dt;
        std::shared_ptr<nlls::RelativePoseError> relative_extrinsics_error =
            std::make_shared<nlls::RelativePoseError>(translation_variance,
                                                      rotation_variance);
        map_ptr_->addResidualBlock(
            relative_extrinsics_error,
            nullptr,
            map_ptr_->parameterBlockPtr(
                changeIdType(last_nframe_id, IdType::Extrinsics, i).asInteger()),
            map_ptr_->parameterBlockPtr(
                changeIdType(nframe_id, IdType::Extrinsics, i).asInteger()));
        //mapPtr_->isJacobianCorrect(id,1.0e-6);
      }
    }
  }

  return true;
}

// Add a landmark.
bool Estimator::addLandmark(LandmarkHandle landmark_handle,
                            const Eigen::Vector4d& landmark)
{
  BackendId landmark_ze_id = createLandmarkId(landmark_handle.handle);

  std::shared_ptr<nlls::HomogeneousPointParameterBlock>
      point_parameter_block =
      std::make_shared<nlls::HomogeneousPointParameterBlock>(
        landmark, landmark_ze_id.asInteger());
  if (!map_ptr_->addParameterBlock(point_parameter_block,
                                   nlls::Map::HomogeneousPoint))
  {
    return false;
  }

  // remember
  double dist = std::numeric_limits<double>::max();
  if(fabs(landmark[3]) > 1.0e-8)
  {
    dist = (landmark / landmark[3]).head<3>().norm(); // euclidean distance
  }

  landmarks_map_.emplace_hint(
        landmarks_map_.end(),
        landmark_ze_id, MapPoint(landmark_ze_id, landmark, dist));
  DEBUG_CHECK(isLandmarkAdded(landmark_ze_id))
      << "bug: inconsistend landmarkdMap_ with mapPtr_.";
  return true;
}

bool Estimator::addVelocityPrior(BackendId frame_id,
                                 const Eigen::Vector3d& velocity,
                                 double sigma)
{
  DEBUG_CHECK(states_.findSlot(frame_id).second);
  SpeedAndBias speed_and_bias;
  speed_and_bias.head<3>() = velocity;
  speed_and_bias.tail<6>().setZero();
  const double speed_information = 1.0 / (sigma * sigma);
  const double bias_information = 0.0;
  Eigen::Matrix<double, 9, 9> information;
  information.setIdentity();
  information.topLeftCorner<3, 3>() *= speed_information;
  information.bottomLeftCorner<6, 6>() *= bias_information;
  std::shared_ptr<nlls::SpeedAndBiasError> prior =
      std::make_shared<nlls::SpeedAndBiasError>(speed_and_bias, information);
  ceres::ResidualBlockId id =
      map_ptr_->addResidualBlock(
        prior,
        nullptr,
        map_ptr_->parameterBlockPtr(
          changeIdType(frame_id, IdType::ImuStates).asInteger()));
  return id != nullptr;
}

// Remove an observation from a landmark.
bool Estimator::removeObservation(ceres::ResidualBlockId residual_block_id)
{
  const nlls::Map::ParameterBlockCollection parameters =
      map_ptr_->parameters(residual_block_id);
  const BackendId landmarkId(parameters.at(1).first);
  DEBUG_CHECK(landmarkId.type() == IdType::Landmark);
  // remove in landmarksMap
  MapPoint& map_point = landmarks_map_.at(landmarkId);
  for(auto it = map_point.observations.begin();
      it!= map_point.observations.end();)
  {
    if(it->second == uint64_t(residual_block_id))
    {
      it = map_point.observations.erase(it);
    }
    else
    {
      ++it;
    }
  }
  // remove residual block
  map_ptr_->removeResidualBlock(residual_block_id);
  return true;
}

// Remove an observation from a landmark, if available.
bool Estimator::removeObservation(BackendId landmark_id, BackendId pose_id,
                                  size_t cam_idx, size_t keypoint_idx)
{
  if(landmarks_map_.find(landmark_id) == landmarks_map_.end())
  {
    for (PointMap::iterator it = landmarks_map_.begin(); it!= landmarks_map_.end(); ++it)
    {
      LOG(INFO) << it->first<<", no. obs = "<<it->second.observations.size();
    }
    LOG(INFO) << landmarks_map_.at(landmark_id).id;
  }
  DEBUG_CHECK(isLandmarkAdded(landmark_id)) << "landmark not added";

  ze::KeypointIdentifier kid(pose_id, cam_idx, keypoint_idx);
  MapPoint& map_point = landmarks_map_.at(landmark_id);
  std::map<ze::KeypointIdentifier, uint64_t>::iterator it =
      map_point.observations.find(kid);
  if(it == landmarks_map_.at(landmark_id).observations.end()){
    return false; // observation not present
  }

  // remove residual block
  map_ptr_->removeResidualBlock(reinterpret_cast< ceres::ResidualBlockId>(it->second));

  // remove also in local map
  map_point.observations.erase(it);

  return true;
}

/**
 * @brief Does a vector contain a certain element.
 * @tparam Class of a vector element.
 * @param vector Vector to search element in.
 * @param query Element to search for.
 * @return True if query is an element of vector.
 */
template<class T>
bool vectorContains(const std::vector<T>& vector, const T & query)
{
  for (size_t i = 0; i < vector.size(); ++i)
  {
    if (vector[i] == query)
    {
      return true;
    }
  }
  return false;
}

// Applies the dropping/marginalization strategy according to the RSS'13/IJRR'14 paper.
// The new number of frames in the window will be numKeyframes+numImuFrames.
bool Estimator::applyMarginalizationStrategy(
    size_t num_keyframes, size_t num_imu_frames,
    LandmarkHandles& removed_landmarks)
{
  // keep the newest numImuFrames
  std::vector<BackendId>::reverse_iterator rit_id = states_.ids.rbegin();
  std::vector<bool>::reverse_iterator rit_keyframe = states_.is_keyframe.rbegin();

  for (size_t k = 0; k < num_imu_frames; ++k)
  {
    ++rit_id;
    ++rit_keyframe;
    if (rit_id==states_.ids.rend())
    {
      // nothing to do.
      return true;
    }
  }

  // remove linear marginalizationError, if existing
  if (marginalization_error_ptr_ && marginalization_residual_id_)
  {
    bool success = map_ptr_->removeResidualBlock(marginalization_residual_id_);
    DEBUG_CHECK(success) << "could not remove marginalization error";
    marginalization_residual_id_ = 0;
    if (!success)
      return false;
  }

  // these will keep track of what we want to marginalize out.
  //! @todo keepParameterBlocks could be removed. Only 'false' is pushed back..
  std::vector<uint64_t> parameter_blocks_to_be_marginalized;
  std::vector<bool> keep_parameter_blocks;

  if (!marginalization_error_ptr_)
  {
    //! @todo should be moved to constructor.
    marginalization_error_ptr_.reset(
        new nlls::MarginalizationError(*map_ptr_.get()));
  }

  // distinguish if we marginalize everything or everything but pose
  std::vector<BackendId> remove_frames;
  std::vector<BackendId> remove_all_but_pose;
  std::vector<BackendId> all_linearized_frames;
  size_t counted_keyframes = 0;
  // Note: rit is now pointing to the first frame not in the sliding window
  // => Either the first keyframe or the frame falling out of the sliding window.
  while (rit_id != states_.ids.rend())
  {
    if (!(*rit_keyframe) || counted_keyframes >= num_keyframes)
    {
      // Frame is neither a keyframe or we already have enough keyframes.
      // => pose will be marginalized.
      remove_frames.push_back(*rit_id);
    }
    else
    {
      counted_keyframes++;
    }
    //! @todo what is the difference between those two vectors??
    remove_all_but_pose.push_back(*rit_id);
    all_linearized_frames.push_back(*rit_id);
    ++rit_id;// check the next frame
    ++rit_keyframe;
  }

  // marginalize everything but pose:
  for (size_t k = 0; k < remove_all_but_pose.size(); ++k)
  {
    // Add all IMU error terms.
    uint64_t speed_and_bias_id =
        changeIdType(remove_all_but_pose[k], IdType::ImuStates).asInteger();
    //! @todo if applyMarginalization is always called with the same arguments
    //! and in every iteration then there would always only be one
    //! speed and bias block to be marginalized.
    if (!map_ptr_->parameterBlockExists(speed_and_bias_id))
    {
      continue; // already marginalized.
    }
    if (map_ptr_->parameterBlockPtr(speed_and_bias_id)->fixed())
    {
      continue; // Do not remove fixed blocks.
    }
    parameter_blocks_to_be_marginalized.push_back(speed_and_bias_id);
    keep_parameter_blocks.push_back(false);

    // Get all residuals connected to this state.
    nlls::Map::ResidualBlockCollection residuals =
        map_ptr_->residuals(speed_and_bias_id);
    for (size_t r = 0; r < residuals.size(); ++r)
    {
      std::shared_ptr<nlls::ReprojectionErrorBase> reprojection_error =
          std::dynamic_pointer_cast<nlls::ReprojectionErrorBase>(
          residuals[r].error_interface_ptr);
      if (reprojection_error == nullptr)
      {
        // we make sure no reprojection errors are yet included.
        marginalization_error_ptr_->addResidualBlock(residuals[r].residual_block_id);
      }
    }
  }

  // marginalize ONLY pose now:
  bool redo_fixation = false;
  for (size_t k = 0; k < remove_frames.size(); ++k)
  {
    // schedule removal
    parameter_blocks_to_be_marginalized.push_back(remove_frames[k].asInteger());
    keep_parameter_blocks.push_back(false);

    // add remaing error terms
    nlls::Map::ResidualBlockCollection residuals =
        map_ptr_->residuals(remove_frames[k].asInteger());

    for (size_t r = 0; r < residuals.size(); ++r)
    {
      if(std::dynamic_pointer_cast<nlls::PoseError>(residuals[r].error_interface_ptr))
      {
        // avoids linearising initial pose error
        map_ptr_->removeResidualBlock(residuals[r].residual_block_id);
        redo_fixation = true;
        continue;
      }
      std::shared_ptr<nlls::ReprojectionErrorBase> reprojection_error =
          std::dynamic_pointer_cast<nlls::ReprojectionErrorBase>(
          residuals[r].error_interface_ptr);
      if (reprojection_error == nullptr)
      {
        // we make sure no reprojection errors are yet included.
        marginalization_error_ptr_->addResidualBlock(residuals[r].residual_block_id);
      }
    }

    // add remaining error terms of the sensor states.
    if (estimate_temporal_extrinsics_)
    {
      for (size_t cam_idx = 0; cam_idx < camera_rig_->size(); ++cam_idx)
      {
        uint64_t extr_id =
            changeIdType(remove_frames[k], IdType::Extrinsics, cam_idx).asInteger();
        if (map_ptr_->parameterBlockPtr(extr_id)->fixed())
        {
          continue;  // we never eliminate fixed blocks.
        }
        parameter_blocks_to_be_marginalized.push_back(extr_id);
        keep_parameter_blocks.push_back(false);

        nlls::Map::ResidualBlockCollection residuals =
            map_ptr_->residuals(extr_id);
        for (size_t r = 0; r < residuals.size(); ++r)
        {
          std::shared_ptr<nlls::ReprojectionErrorBase> reprojection_error =
              std::dynamic_pointer_cast<nlls::ReprojectionErrorBase>(
                residuals[r].error_interface_ptr);
          if(reprojection_error == nullptr)
          {
            // we make sure no reprojection errors are yet included.
            marginalization_error_ptr_->addResidualBlock(residuals[r].residual_block_id);
          }
        }
      }
    }

    // now finally we treat all the observations.
    DEBUG_CHECK(all_linearized_frames.size()>0) << "bug";
    // In the case that the frame falling out of the sliding window not being a
    // keyframe, this is the id of this frame.
    const BackendId current_kf_id = all_linearized_frames.at(0);

    // If the frame dropping out of the sliding window is not a keyframe, then
    // the observations are deleted. If it is, then the landmarks visible in
    // the oldest keyframe but not the newest one are marginalized.
    {
      for(PointMap::iterator pit = landmarks_map_.begin();
          pit != landmarks_map_.end();)
      {
        nlls::Map::ResidualBlockCollection residuals =
            map_ptr_->residuals(pit->first.asInteger());

        // first check if we can skip
        bool skip_landmark = true;
        bool has_new_observations = false;
        bool just_delete = false;
        bool marginalize = true;
        bool error_term_added = false;
        //! @todo second entry always true?
        std::map<uint64_t, bool> visible_in_frame;
        size_t obs_count = 0;
        for (size_t r = 0; r < residuals.size(); ++r)
        {
          std::shared_ptr<nlls::ReprojectionErrorBase> reprojection_error =
              std::dynamic_pointer_cast<nlls::ReprojectionErrorBase>(
                  residuals[r].error_interface_ptr);
          if (reprojection_error)
          {
            BackendId pose_id =
                BackendId(
                  map_ptr_->parameters(residuals[r].residual_block_id).at(0).first);
            // since we have implemented the linearisation to account for robustification,
            // we don't kick out bad measurements here any more like
            // if(vectorContains(allLinearizedFrames,poseId)){ ...
            //   if (error.transpose() * error > 6.0) { ... removeObservation ... }
            // }
            if(vectorContains(remove_frames, pose_id))
            {
              skip_landmark = false;
            }
            if(pose_id >= current_kf_id)
            {
              marginalize = false;
              has_new_observations = true;
            }
            if(vectorContains(all_linearized_frames, pose_id))
            {
              visible_in_frame.insert(std::make_pair(pose_id.asInteger(), true));
              obs_count++;
            }
          }
        }

        if(residuals.size()==0)
        {
          // No observations of landmark anymore. I.e. only observed by a now
          // marginalised frame.
          map_ptr_->removeParameterBlock(pit->first.asInteger());
          removed_landmarks.push_back(pit->first.landmarkHandle());
          pit = landmarks_map_.erase(pit);
          continue;
        }

        if(skip_landmark)
        {
          pit++;
          continue;
        }

        // so, we need to consider it.
        for (size_t r = 0; r < residuals.size(); ++r)
        {
          std::shared_ptr<nlls::ReprojectionErrorBase> reprojection_error =
              std::dynamic_pointer_cast<nlls::ReprojectionErrorBase>(
                  residuals[r].error_interface_ptr);
          if (reprojection_error)
          {
            BackendId pose_id(
                  map_ptr_->parameters(residuals[r].residual_block_id).at(0).first);
            if((vectorContains(remove_frames, pose_id) && has_new_observations) ||
               (!vectorContains(all_linearized_frames, pose_id) && marginalize))
            {
              // ok, let's ignore the observation.
              removeObservation(residuals[r].residual_block_id);
              residuals.erase(residuals.begin() + r);
              r--;
            }
            else if(marginalize && vectorContains(all_linearized_frames, pose_id))
            {
              // TODO: consider only the sensible ones for marginalization
              if(obs_count < 2)
              {
                removeObservation(residuals[r].residual_block_id);
                residuals.erase(residuals.begin() + r);
                r--;
              }
              else
              {
                // add information to be considered in marginalization later.
                error_term_added = true;
                marginalization_error_ptr_->addResidualBlock(
                    residuals[r].residual_block_id, false);
              }
            }
            // check anything left
            if (residuals.size() == 0)
            {
              just_delete = true;
              marginalize = false;
            }
          }
        }

        if(just_delete)
        {
          map_ptr_->removeParameterBlock(pit->first.asInteger());
          removed_landmarks.push_back(pit->first.landmarkHandle());
          pit = landmarks_map_.erase(pit);
          continue;
        }
        if(marginalize && error_term_added)
        {
          parameter_blocks_to_be_marginalized.push_back(pit->first.asInteger());
          keep_parameter_blocks.push_back(false);
          removed_landmarks.push_back(pit->first.landmarkHandle());
          pit = landmarks_map_.erase(pit);
          continue;
        }

        pit++;
      }
    }

    // update book-keeping and go to the next frame
    //if(it != statesMap_.begin()){ // let's remember that we kept the very first pose
    VLOG(20) << "Marginalizing out state with id " << remove_frames[k];
    states_.removeState(remove_frames[k]);
  }

  // now apply the actual marginalization
  if(parameter_blocks_to_be_marginalized.size() > 0)
  {
    if (FLAGS_v >= 20)
    {
      std::stringstream s;
      s << "Marginalizing following parameter blocks:\n";
      for (uint64_t id : parameter_blocks_to_be_marginalized)
      {
        s << BackendId(id) << "\n";
      }
      VLOG(20) << s.str();
    }
    marginalization_error_ptr_->marginalizeOut(parameter_blocks_to_be_marginalized,
                                               keep_parameter_blocks);
  }

  // update error computation
  if(parameter_blocks_to_be_marginalized.size() > 0)
  {
    marginalization_error_ptr_->updateErrorComputation();
  }

  // add the marginalization term again
  if(marginalization_error_ptr_->num_residuals()==0)
  {
    marginalization_error_ptr_.reset();
  }
  if (marginalization_error_ptr_)
  {
    std::vector<std::shared_ptr<nlls::ParameterBlock> > parameter_block_ptrs;
    marginalization_error_ptr_->getParameterBlockPtrs(parameter_block_ptrs);
    marginalization_residual_id_ = map_ptr_->addResidualBlock(
        marginalization_error_ptr_, nullptr, parameter_block_ptrs);
    DEBUG_CHECK(marginalization_residual_id_) << "could not add marginalization error";
    if (!marginalization_residual_id_)
    {
      return false;
    }
  }

  if(redo_fixation)
  {
    // finally fix the first pose properly
    //mapPtr_->resetParameterization(statesMap_.begin()->first, nlls::Map::Pose3d);
    Transformation T_WS_0;
    BackendId oldest_id = states_.ids[0];
    get_T_WS(oldest_id, T_WS_0);
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double,6,6>::Zero();
    information(0, 0) = 1.0e14;
    information(1, 1) = 1.0e14;
    information(2, 2) = 1.0e14;
    information(5, 5) = 1.0e14;
    std::shared_ptr<nlls::PoseError> pose_error =
        std::make_shared<nlls::PoseError>(T_WS_0, information);
    map_ptr_->addResidualBlock(
          pose_error, nullptr,
          map_ptr_->parameterBlockPtr(oldest_id.asInteger()));
  }

  return true;
}

// Prints state information to buffer.
void Estimator::printStates(BackendId pose_id, std::ostream& buffer) const
{
  auto slot = states_.findSlot(pose_id);
  if (!slot.second)
  {
    buffer << "Tried to print info on pose with ID " << pose_id
           << " which is not part of the estimator." << std::endl;
    return;
  }
  buffer << "Pose. ID: " << pose_id
         << " - Keyframe: " << (states_.is_keyframe[slot.first] ? "yes" : "no")
         << " - Timestamp: " << states_.timestamps[slot.first]
         << ":\n";
  buffer << getPoseEstimate(pose_id).first << "\n";

  BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
  auto sab = getSpeedAndBiasEstimate(speed_and_bias_id);
  if (sab.second)
  {
    buffer << "Speed and Bias. ID: " << speed_and_bias_id << ":\n";
    buffer << sab.first.transpose() << "\n";
  }
  std::vector<BackendId> extrinsics_id = constant_extrinsics_ids_;
  if (estimate_temporal_extrinsics_)
  {
    for (size_t i = 0; i < extrinsics_id.size(); ++i)
    {
      extrinsics_id[i] = changeIdType(pose_id, IdType::Extrinsics, i);
    }
  }
  for (size_t i = 0; i < extrinsics_id.size(); ++i)
  {
    auto extrinsics = getPoseEstimate(extrinsics_id[i]);
    if (extrinsics.second)
    {
      buffer << "Extrinsics. ID: " << extrinsics_id[i] << ":\n";
      buffer << extrinsics.first << "\n";
    }
  }
  buffer << "-------------------------------------------" << std::endl;
}

// Initialise pose from IMU measurements. For convenience as static.
bool Estimator::initPoseAndBiasFromImu(
    const ImuStamps& imu_stamps,
    const ImuAccGyrContainer& imu_acc_gyr,
    const double g,
    Transformation& T_WS,
    SpeedAndBias& speed_and_bias)
{
  DEBUG_CHECK_EQ(imu_stamps.rows(), imu_acc_gyr.cols());
  // set translation to zero, unit rotation
  T_WS.setIdentity();

  const int n_measurements = imu_stamps.rows();

  if (n_measurements == 0)
  {
    return false;
  }

  // acceleration vector
  Eigen::Vector3d acc_B = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr_B = Eigen::Vector3d::Zero();
  for (int i = 0; i < n_measurements; ++i)
  {
    acc_B += imu_acc_gyr.col(i).head<3>();
    gyr_B += imu_acc_gyr.col(i).tail<3>();
  }
  acc_B /= static_cast<double>(n_measurements);
  gyr_B /= static_cast<double>(n_measurements);
  Eigen::Vector3d e_acc = acc_B.normalized();

  // align with ez_W:
  Eigen::Vector3d ez_W(0.0, 0.0, 1.0);
  Eigen::Matrix<double, 6, 1> pose_increment;
  pose_increment.head<3>() = Eigen::Vector3d::Zero();
  pose_increment.tail<3>() = ez_W.cross(e_acc).normalized();
  double angle = std::acos(ez_W.transpose() * e_acc);
  pose_increment.tail<3>() *= angle;
  T_WS = ze::Transformation::exp(-pose_increment) * T_WS;
  speed_and_bias.head<3>().setZero();

  // Set biases
  speed_and_bias.segment<3>(3) = gyr_B;
  speed_and_bias.segment<3>(6) =
      acc_B - T_WS.inverse() * (ez_W * g);

  if (FLAGS_vio_acc_bias_init_x != 0.0 || FLAGS_vio_acc_bias_init_y != 0.0 ||
      FLAGS_vio_acc_bias_init_z != 0.0) {
    LOG(ERROR) << "HARDCODED IMU ACC BIASES FOR INITIALIZATION!";
    speed_and_bias.segment<3>(3) = Eigen::Vector3d(
          FLAGS_vio_gyr_bias_init_x,
          FLAGS_vio_gyr_bias_init_y,
          FLAGS_vio_gyr_bias_init_z
          );
  }
  if (FLAGS_vio_gyr_bias_init_x != 0.0 || FLAGS_vio_gyr_bias_init_y != 0.0 ||
      FLAGS_vio_gyr_bias_init_z != 0.0) {
    LOG(ERROR) << "HARDCODED IMU GYR BIASES FOR INITIALIZATION!";
    speed_and_bias.segment<3>(6) = Eigen::Vector3d(
          FLAGS_vio_acc_bias_init_x,
          FLAGS_vio_acc_bias_init_y,
          FLAGS_vio_acc_bias_init_z
          );
  }

  LOG(INFO) << "Initializing biases:";
  LOG(INFO) << "   Gyro: " << speed_and_bias.segment<3>(3).transpose();
  LOG(INFO) << "    Acc: " << speed_and_bias.segment<3>(6).transpose();


  return true;
}

// Start ceres optimization.
#ifdef USE_OPENMP
void Estimator::optimize(size_t num_iter, size_t num_threads, bool verbose)
#else
void Estimator::optimize(size_t num_iter, size_t /*num_threads*/,
                                 bool verbose) // avoid warning since numThreads unused
#warning openmp not detected, your system may be slower than expected
#endif

{
  // assemble options
  map_ptr_->options.linear_solver_type = ceres::SPARSE_SCHUR;
  //mapPtr_->options.initial_trust_region_radius = 1.0e4;
  //mapPtr_->options.initial_trust_region_radius = 2.0e6;
  //mapPtr_->options.preconditioner_type = ceres::IDENTITY;
  map_ptr_->options.trust_region_strategy_type = ceres::DOGLEG;
  //mapPtr_->options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  //mapPtr_->options.use_nonmonotonic_steps = true;
  //mapPtr_->options.max_consecutive_nonmonotonic_steps = 10;
  //mapPtr_->options.function_tolerance = 1e-12;
  //mapPtr_->options.gradient_tolerance = 1e-12;
  //mapPtr_->options.jacobi_scaling = false;
#ifdef USE_OPENMP
    map_ptr_->options.num_threads = num_threads;
#endif
  map_ptr_->options.max_num_iterations = num_iter;

  if (verbose)
  {
    map_ptr_->options.minimizer_progress_to_stdout = true;
  }
  else
  {
    map_ptr_->options.logging_type = ceres::LoggingType::SILENT;
    map_ptr_->options.minimizer_progress_to_stdout = false;
  }

  // call solver
  map_ptr_->solve();

  // update landmarks
  {
    for(auto it = landmarks_map_.begin(); it!=landmarks_map_.end(); ++it)
    {
      // update coordinates
      it->second.point =
          std::static_pointer_cast<nlls::HomogeneousPointParameterBlock>(
          map_ptr_->parameterBlockPtr(it->first.asInteger()))->estimate();
    }
  }

  // summary output
  if (verbose)
  {
    LOG(INFO) << map_ptr_->summary.FullReport();
    std::stringstream s;
    for (const auto& id : states_.ids)
    {
      printStates(id, s);
    }
    LOG(INFO) << s.str();
  }
}

// Set a time limit for the optimization process.
bool Estimator::setOptimizationTimeLimit(double time_limit, int min_iterations)
{
  if(ceres_callback_ != nullptr)
  {
    if(time_limit < 0.0)
    {
      // no time limit => set minimum iterations to maximum iterations
      ceres_callback_->setMinimumIterations(map_ptr_->options.max_num_iterations);
      return true;
    }
    ceres_callback_->setTimeLimit(time_limit);
    ceres_callback_->setMinimumIterations(min_iterations);
    return true;
  }
  else if(time_limit >= 0.0)
  {
    ceres_callback_.reset(
          new nlls::CeresIterationCallback(time_limit, min_iterations));
    map_ptr_->options.callbacks.push_back(ceres_callback_.get());
    return true;
  }
  // no callback yet registered with ceres.
  // but given time limit is lower than 0, so no callback needed
  return true;
}

// getters
// Get a specific landmark.
bool Estimator::getLandmark(BackendId landmark_id,
                            MapPoint& map_point) const
{
  std::lock_guard<std::mutex> l(landmarks_mutex_);
  if (landmarks_map_.find(landmark_id) == landmarks_map_.end())
  {
    DEBUG_CHECK(false)
        << "landmark with id = " << landmark_id << " does not exist.";
    return false;
  }
  map_point = landmarks_map_.at(landmark_id);
  return true;
}

// Checks whether the landmark is initialized.
bool Estimator::isLandmarkInitialized(BackendId landmark_id) const
{
  DEBUG_CHECK(isLandmarkAdded(landmark_id)) << "landmark not added";
  return std::static_pointer_cast<nlls::HomogeneousPointParameterBlock>(
      map_ptr_->parameterBlockPtr(landmark_id.asInteger()))->initialized();
}

// Get a copy of all the landmarks as a PointMap.
size_t Estimator::getLandmarks(PointMap& landmarks) const
{
  std::lock_guard<std::mutex> l(landmarks_mutex_);
  landmarks = landmarks_map_;
  return landmarks_map_.size();
}

// Get a copy of all the landmark in a MapPointVector. This is for legacy support.
// Use getLandmarks(ze::PointMap&) if possible.
size_t Estimator::getLandmarks(MapPointVector& landmarks) const
{
  std::lock_guard<std::mutex> l(landmarks_mutex_);
  landmarks.clear();
  landmarks.reserve(landmarks_map_.size());
  for(PointMap::const_iterator it=landmarks_map_.begin(); it!=landmarks_map_.end(); ++it)
  {
    landmarks.push_back(it->second);
  }
  return landmarks_map_.size();
}

// Get pose for a given nframe handle.
bool Estimator::get_T_WS(BackendId id,
                         Transformation& T_WS) const
{
  DEBUG_CHECK(id.type() == IdType::NFrame) << "wrong id type: id = " << id;
  bool success;
  std::tie(T_WS, success) = getPoseEstimate(id);
  return success;
}

// Get speeds and IMU biases for a given pose ID.
bool Estimator::getSpeedAndBiasFromNFrameId(BackendId id,
                                            SpeedAndBias& speed_and_bias) const
{
  bool success;
  BackendId sab_id = changeIdType(id, IdType::ImuStates);
  std::tie(speed_and_bias, success) = getSpeedAndBiasEstimate(sab_id);
  return success;
}

// Get camera states for a given pose ID.
bool Estimator::getCameraSensorStatesFromNFrameId(
    BackendId pose_id, size_t camera_idx, Transformation& T_SCi) const
{
  BackendId extrinsics_id;
  if (estimate_temporal_extrinsics_)
  {
    extrinsics_id = changeIdType(pose_id, IdType::Extrinsics, camera_idx);
  }
  else
  {
    extrinsics_id = constant_extrinsics_ids_.at(camera_idx);
  }
  bool success;
  std::tie(T_SCi, success) = getPoseEstimate(extrinsics_id);
  return success;
}

// Get the ID of the current keyframe.
BackendId Estimator::currentKeyframeId() const
{
  for (int i = states_.ids.size() - 1; i >= 0; --i)
  {
    if (states_.is_keyframe[i])
    {
      return states_.ids[i];
    }
  }
  DEBUG_CHECK(false) << "no existing keyframes ...";
  return BackendId();
}

// Get the ID of an older frame.
BackendId Estimator::frameIdByAge(size_t age) const
{
  std::vector<BackendId>::const_reverse_iterator rit = states_.ids.rbegin();
  for(size_t i=0; i < age; ++i)
  {
    ++rit;
    DEBUG_CHECK(rit != states_.ids.rend())
        << "requested age " << age << " out of range.";
  }
  return *rit;
}

// Get the ID of the newest frame added to the state.
BackendId Estimator::currentFrameId() const
{
  DEBUG_CHECK(states_.ids.size() > 0) << "no frames added yet.";
  return states_.ids.back();
}

NFrameHandle Estimator::currentFrameHandle() const
{
  return currentFrameId().nframeHandle();
}

std::vector<BackendId> Estimator::getAllFrameIds() const
{
  return states_.ids;
}

// Checks if a particular frame is still in the IMU window
bool Estimator::isInImuWindow(BackendId frame_id) const
{
  return getSpeedAndBiasEstimate(frame_id).second;
}

// Set pose for a given pose ID.
bool Estimator::set_T_WS(BackendId pose_id,
                         const Transformation& T_WS)
{
  DEBUG_CHECK(pose_id.type() == IdType::NFrame);
  return setPoseEstimate(pose_id, T_WS);
}

// Set the speeds and IMU biases for a given pose ID.
bool Estimator::setSpeedAndBiasFromNFrameId(BackendId pose_id,
                                            const SpeedAndBias& speed_and_bias)
{
  DEBUG_CHECK(pose_id.type() == IdType::NFrame);
  BackendId sab_id = changeIdType(pose_id, IdType::ImuStates);
  return setSpeedAndBiasEstimate(sab_id, speed_and_bias);
}

// Set the transformation from sensor to camera frame for a given pose ID.
bool Estimator::setCameraSensorStates(
    BackendId pose_id, uint8_t camera_idx,
    const Transformation & T_SCi)
{
  DEBUG_CHECK(pose_id.type() == IdType::NFrame);
  BackendId extrinsics_id = changeIdType(pose_id, IdType::Extrinsics, camera_idx);
  return setPoseEstimate(extrinsics_id, T_SCi);
}

// Set the homogeneous coordinates for a landmark.
bool Estimator::setLandmark(
    BackendId landmark_id, const Eigen::Vector4d& landmark)
{
  std::shared_ptr<nlls::ParameterBlock> parameterBlockPtr =
      map_ptr_->parameterBlockPtr(landmark_id.asInteger());
#ifndef NDEBUG
  std::shared_ptr<nlls::HomogeneousPointParameterBlock> derivedParameterBlockPtr =
  std::dynamic_pointer_cast<nlls::HomogeneousPointParameterBlock>(parameterBlockPtr);
  if(!derivedParameterBlockPtr)
  {
    DEBUG_CHECK(false) << "wrong pointer type requested.";
    return false;
  }
  derivedParameterBlockPtr->setEstimate(landmark);;
#else
  std::static_pointer_cast<nlls::HomogeneousPointParameterBlock>(
      parameterBlockPtr)->setEstimate(landmark);
#endif

  // also update in map
  landmarks_map_.at(landmark_id).point = landmark;
  return true;
}

// Set the landmark initialization state.
void Estimator::setLandmarkInitialized(BackendId landmark_id,
                                       bool initialized)
{
  DEBUG_CHECK(isLandmarkAdded(landmark_id)) << "landmark not added";
  std::static_pointer_cast<nlls::HomogeneousPointParameterBlock>(
      map_ptr_->parameterBlockPtr(landmark_id.asInteger()))->setInitialized(initialized);
}

// private stuff
// getters

std::pair<Transformation, bool> Estimator::getPoseEstimate(BackendId id) const
{
  DEBUG_CHECK(id.type() == IdType::NFrame ||
              id.type() == IdType::Extrinsics);
  if (!map_ptr_->parameterBlockExists(id.asInteger()))
  {
    return std::make_pair(Transformation(), false);
  }
#ifndef NDEBUG
  std::shared_ptr<nlls::ParameterBlock> base_ptr =
      map_ptr_->parameterBlockPtr(id.asInteger());
  if (base_ptr != nullptr)
  {
    std::shared_ptr<nlls::PoseParameterBlock> block_ptr =
        std::dynamic_pointer_cast<nlls::PoseParameterBlock>(base_ptr);
    CHECK(block_ptr != nullptr) << "Incorrect pointer cast detected!";
    return std::make_pair(block_ptr->estimate(), true);
  }
#else
  std::shared_ptr<nlls::PoseParameterBlock> block_ptr =
      std::static_pointer_cast<nlls::PoseParameterBlock>(
        map_ptr_->parameterBlockPtr(id.asInteger()));
  if (block_ptr != nullptr)
  {
    return std::make_pair(block_ptr->estimate(), true);
  }
#endif
  return std::make_pair(Transformation(), false);
}

std::pair<SpeedAndBias, bool> Estimator::getSpeedAndBiasEstimate(BackendId id) const
{
  DEBUG_CHECK(id.type() == IdType::ImuStates);
  if (!map_ptr_->parameterBlockExists(id.asInteger()))
  {
    return std::make_pair(SpeedAndBias(), false);
  }
#ifndef NDEBUG
  std::shared_ptr<nlls::ParameterBlock> base_ptr =
      map_ptr_->parameterBlockPtr(id.asInteger());
  if (base_ptr != nullptr)
  {
    std::shared_ptr<nlls::SpeedAndBiasParameterBlock> block_ptr =
        std::dynamic_pointer_cast<nlls::SpeedAndBiasParameterBlock>(base_ptr);
    CHECK(block_ptr != nullptr) << "Incorrect pointer cast detected!";
    return std::make_pair(block_ptr->estimate(), true);
  }
#else
  std::shared_ptr<nlls::SpeedAndBiasParameterBlock> block_ptr =
      std::static_pointer_cast<nlls::SpeedAndBiasParameterBlock>(
        map_ptr_->parameterBlockPtr(id.asInteger()));
  if (block_ptr != nullptr)
  {
    return std::make_pair(block_ptr->estimate(), true);
  }
#endif
  return std::make_pair(SpeedAndBias(), false);
}

bool Estimator::setPoseEstimate(BackendId id, const Transformation& pose)
{
  if (!map_ptr_->parameterBlockExists(id.asInteger()))
  {
    return false;
  }

  DEBUG_CHECK(id.type() == IdType::NFrame ||
              id.type() == IdType::Extrinsics);
#ifndef NDEBUG
  std::shared_ptr<nlls::ParameterBlock> base_ptr =
      map_ptr_->parameterBlockPtr(id.asInteger());
  if (base_ptr == nullptr)
  {
    return false;
  }
  std::shared_ptr<nlls::PoseParameterBlock> block_ptr =
      std::dynamic_pointer_cast<nlls::PoseParameterBlock>(base_ptr);
  CHECK(base_ptr != nullptr) << "Incorrect pointer cast detected!";
  block_ptr->setEstimate(pose);
#else
  std::shared_ptr<nlls::PoseParameterBlock> block_ptr =
      std::static_pointer_cast<nlls::PoseParameterBlock>(
        map_ptr_->parameterBlockPtr(id.asInteger()));
  block_ptr->setEstimate(pose);
#endif
  return true;
}

bool Estimator::setSpeedAndBiasEstimate(BackendId id, const SpeedAndBias& sab)
{
  DEBUG_CHECK(id.type() == IdType::ImuStates);
  if (!map_ptr_->parameterBlockExists(id.asInteger()))
  {
    return false;
  }
#ifndef NDEBUG
  std::shared_ptr<nlls::ParameterBlock> base_ptr =
      map_ptr_->parameterBlockPtr(id.asInteger());
  if (base_ptr == nullptr)
  {
    return false;
  }
  std::shared_ptr<nlls::SpeedAndBiasParameterBlock> block_ptr =
      std::dynamic_pointer_cast<nlls::SpeedAndBiasParameterBlock>(base_ptr);
  CHECK(base_ptr != nullptr) << "Incorrect pointer cast detected!";
  block_ptr->setEstimate(sab);
#else
  std::shared_ptr<nlls::SpeedAndBiasParameterBlock> block_ptr =
      std::static_pointer_cast<nlls::SpeedAndBiasParameterBlock>(
        map_ptr_->parameterBlockPtr(id.asInteger()));
  block_ptr->setEstimate(sab);
#endif
  return true;
}

}  // namespace ze


