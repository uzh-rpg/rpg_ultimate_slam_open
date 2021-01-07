// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.

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

#include <string>
#include <vector>
#include <gflags/gflags.h>

#include <ze/cameras/camera.hpp>
#include <ze/common/types.hpp>
#include <ze/common/macros.hpp>
#include <ze/common/transformation.hpp>

DECLARE_string(calib_filename);
DECLARE_string(mask_cam0);
DECLARE_string(mask_cam1);
DECLARE_string(mask_cam2);
DECLARE_string(mask_cam3);

namespace ze {

// convenience typedefs.
using CameraVector     = std::vector<Camera::Ptr>;
using StereoIndexPair  = std::pair<uint8_t, uint8_t>;
using StereoIndexPairs = std::vector<StereoIndexPair>;

//! A camera rig is a set of rigidly attached cameras. The cameras are
//! assumed intrinsically and extrinsically calibrated.
class CameraRig
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ZE_POINTER_TYPEDEFS(CameraRig);

  CameraRig() = delete;

  CameraRig(
      const TransformationVector& T_C_B,
      const CameraVector& cameras,
      const std::string& label,
      const real_t stereo_min_fov_overlap = 0.7,
      const real_t stereo_min_baseline = 0.04);

  //! @name Camera extrinsic calibration: Pose of (B)ody/Imu in (C)amera frame.
  //! @{
  inline const Transformation& T_C_B(size_t camera_index) const
  {
    DEBUG_CHECK_LT(camera_index, T_C_B_.size());
    return T_C_B_[camera_index];
  }

  inline Transformation T_B_C(size_t camera_index) const
  {
    DEBUG_CHECK_LT(camera_index, T_C_B_.size());
    return T_C_B_[camera_index].inverse();
  }

  inline const TransformationVector& T_C_B_vec() const
  {
    return T_C_B_;
  }
  //! @}

  //! @name Camera accessors.
  //! @{
  inline const Camera& at(size_t camera_index) const
  {
    DEBUG_CHECK_LT(camera_index, cameras_.size());
    return *cameras_[camera_index];
  }

  inline std::shared_ptr<Camera>& atShared(size_t camera_index)
  {
    DEBUG_CHECK_LT(camera_index, cameras_.size());
    return cameras_[camera_index];
  }

  inline std::shared_ptr<const Camera> atShared(size_t camera_index) const
  {
    DEBUG_CHECK_LT(camera_index, cameras_.size());
    return cameras_[camera_index];
  }

  inline bool getDvsCamera(Camera::Ptr* dvs_camera) const {
    CHECK_NOTNULL(dvs_camera);
    if (dvs_camera_ == nullptr) {
      return false;
    } else {
      *dvs_camera = dvs_camera_;
      return true;
    }
  }

  inline bool getDvsCamera(Camera::ConstPtr* dvs_camera) const {
    CHECK_NOTNULL(dvs_camera);
    if (dvs_camera_ == nullptr) {
      return false;
    } else {
      *dvs_camera = dvs_camera_;
      return true;
    }
  }

  inline bool getDvsCameraIndex(size_t* index) const {
    CHECK_NOTNULL(index);
    if (dvs_camera_ == nullptr) {
      return false;
    } else {
      *index = dvs_camera_index_;
      return true;
    }
  }

  inline bool hasDvsCamera() const {
    return dvs_camera_ != nullptr;
  }

  inline void setDvsCamera(const Camera& dvs_camera) const {
    *dvs_camera_ = dvs_camera;
  }

  /// Remove distortion and create lookup tables.
  void setupDvs();

  inline const CameraVector& cameras() const { return cameras_; }
  //! @}

  inline size_t size() const { return cameras_.size(); }

  inline const std::string& label() const { return label_; }

  inline const StereoIndexPairs& stereoPairs() const { return stereo_pairs_; }

  inline void setStereoPairs(const StereoIndexPairs& pairs) { stereo_pairs_ = pairs; }

  //! @name Camera iteration.
  //! @{
  typedef CameraVector::value_type value_type;
  typedef CameraVector::iterator iterator;
  typedef CameraVector::const_iterator const_iterator;
  CameraVector::iterator begin() { return cameras_.begin(); }
  CameraVector::iterator end() { return cameras_.end(); }
  CameraVector::const_iterator begin() const { return cameras_.begin(); }
  CameraVector::const_iterator end() const { return cameras_.end(); }
  CameraVector::const_iterator cbegin() const { return cameras_.cbegin(); }
  CameraVector::const_iterator cend() const { return cameras_.cend(); }
  //! @}

  //! Create lookup tables, only for dvs.
  void initLookupTables(std::shared_ptr<const Camera>& camera);

  //! Get a rig that contains only a subset of the cameras.
  CameraRig::Ptr getSubRig(
      const std::vector<uint32_t>& camera_indices,
      const std::string& label);

  //! Dvs keypoint and bearing lookup tables
  Eigen::Matrix<float, 4, Eigen::Dynamic> dvs_bearing_lut_;
  Eigen::Matrix<float, 2, Eigen::Dynamic> dvs_keypoint_lut_;

  //! Calculators for Dvs keypoint and bearing lut.
  void calculateBearingLUT(const std::shared_ptr<const Camera>& c,
                       Eigen::Matrix<float, 4, Eigen::Dynamic>* dvs_bearing_lut);
  void calculateKeypointLUT(const std::shared_ptr<const Camera>& c,
                 const Eigen::Matrix<float, 4, Eigen::Dynamic>& dvs_bearing_lut,
                 Eigen::Matrix<float, 2, Eigen::Dynamic>* dvs_keypoint_lut);

private:
  //! The mounting transformations.
  TransformationVector T_C_B_;

  //! The camera geometries.
  CameraVector cameras_;

  //! A dvs camera if any.
  Camera::Ptr dvs_camera_;
  size_t dvs_camera_index_;

  //! Unique pairs of camera indices with overlapping field of view.
  StereoIndexPairs stereo_pairs_;

  //! A label for this camera rig, a name.
  std::string label_;

};

//! Load a camera rig form a yaml file. Returns a nullptr if the loading fails.
CameraRig::Ptr cameraRigFromYaml(const std::string& yaml_file);

//! Load a camera rig form a gflag parameters. Returns a nullptr if the loading fails.
CameraRig::Ptr cameraRigFromGflags();

//! Formatted printing.
std::ostream& operator<<(std::ostream& out, const CameraRig& rig);
std::ostream& operator<<(std::ostream& out, const StereoIndexPairs& stereo_pairs);

//! Compute overlapping field of view and baseline for all stereo pairs
StereoIndexPairs identifyStereoPairsInRig(
    const CameraRig& rig,
    const real_t& min_fov_overlap,
    const real_t& min_baseline);

} // namespace ze

