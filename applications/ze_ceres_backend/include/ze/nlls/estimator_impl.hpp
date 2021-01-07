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
 *  Created on: Jan 10, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Zurich Eye
 *********************************************************************************/

/**
 * @file implementation/Estimator.hpp
 * @brief Header implementation file for the Estimator class.
 * @author Stefan Leutenegger
 */

#pragma once

#include <ze/nlls/estimator.hpp>
#include <ze/nlls/reprojection_error.hpp>

/// \brief ze Main namespace of this package.
namespace ze {

// Add an observation to a landmark.
template<class GEOMETRY_TYPE>
ceres::ResidualBlockId Estimator::addObservation(
    LandmarkHandle landmark_handle,
    BackendId nframe_id,
    const Eigen::Vector2d& measurement,
    int8_t level,
    size_t cam_idx,
    size_t keypoint_idx)
{
  DEBUG_CHECK_GE(level, 0);
  // get Landmark ID.
  const BackendId landmark_id = createLandmarkId(landmark_handle);
  DEBUG_CHECK(isLandmarkAdded(landmark_id)) << "landmark not added";

  // avoid double observations
  KeypointIdentifier kid(nframe_id, cam_idx, keypoint_idx);
  if (landmarks_map_.at(landmark_id).observations.find(kid)
      != landmarks_map_.at(landmark_id).observations.end())
  {
    return nullptr;
  }

  // get the keypoint measurement
  size_t slot;
  bool success;
  std::tie(slot, success) = states_.findSlot(nframe_id);
  if (!success)
  {
    LOG(ERROR) << "Tried to add observation for frame that is either already "
               << "marginalized out or not yet added to the state. ID = "
               << nframe_id;
    return nullptr;
  }

  Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
  information *= 1.0 / static_cast<double>(1 << level);

  // create error term
  DEBUG_CHECK(
        std::dynamic_pointer_cast<const GEOMETRY_TYPE>(camera_rig_->atShared(cam_idx)))
      << "Incorrect pointer cast requested. ";
  std::shared_ptr<nlls::ReprojectionError<GEOMETRY_TYPE> > reprojection_error =
      std::make_shared<nlls::ReprojectionError<GEOMETRY_TYPE> >(
        std::static_pointer_cast<const GEOMETRY_TYPE>(camera_rig_->atShared(cam_idx)),
        measurement, information);

  BackendId extrinsics_id = constant_extrinsics_ids_[cam_idx];
  if (estimate_temporal_extrinsics_)
  {
    extrinsics_id = changeIdType(nframe_id, IdType::Extrinsics, cam_idx);
  }
  ceres::ResidualBlockId ret_val = map_ptr_->addResidualBlock(
      reprojection_error,
      cauchy_loss_function_ptr_ ? cauchy_loss_function_ptr_.get() : nullptr,
      map_ptr_->parameterBlockPtr(nframe_id.asInteger()),
      map_ptr_->parameterBlockPtr(landmark_id.asInteger()),
      map_ptr_->parameterBlockPtr(extrinsics_id.asInteger()));

  // remember
  landmarks_map_.at(landmark_id).observations.insert(
      std::pair<ze::KeypointIdentifier, uint64_t>(
          kid, reinterpret_cast<uint64_t>(ret_val)));

  return ret_val;
}

}  // namespace ze
