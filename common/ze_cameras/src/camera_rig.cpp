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

#include <ze/cameras/camera_rig.hpp>

#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/core/image_raw.hpp>
#include <ze/cameras/camera_utils.hpp>
#include <ze/cameras/camera_yaml_serialization.hpp>
#include <ze/common/path_utils.hpp>
#include <ze/cameras/camera_impl.hpp>
#include <opencv2/calib3d/calib3d.hpp>

DEFINE_bool(vio_use_events, false,
            "Use events instead of images.");
DEFINE_bool(vio_use_events_and_images, false,
            "Use events and images. Ignores vio_use_events flag.");

DEFINE_bool(vio_rescale_intrinsics_for_distortion, true,
            "Whether to rescale the camera intrinsics to mitigate the loss of FoV introduced by image undistortion.");

DEFINE_string(calib_filename, "", "Camera calibration file.");
DEFINE_string(mask_cam0, "", "Mask for camera 0");
DEFINE_string(mask_cam1, "", "Mask for camera 1");
DEFINE_string(mask_cam2, "", "Mask for camera 2");
DEFINE_string(mask_cam3, "", "Mask for camera 3");

namespace ze {

  // -----------------------------------------------------------------------------
  CameraRig::CameraRig(
      const TransformationVector& T_C_B,
      const CameraVector& cameras,
      const std::string& label,
      const real_t stereo_min_fov_overlap,
      const real_t stereo_min_baseline)
    : dvs_bearing_lut_()
    , dvs_keypoint_lut_()
    , T_C_B_(T_C_B)
    , cameras_(cameras)
    , dvs_camera_(nullptr)
    , dvs_camera_index_(0)
    , label_(label)
  {
    CHECK_EQ(T_C_B_.size(), cameras_.size());

    // Search if the rig contains a dvs camera.
    // If dvs not requested, delete it.
    // If dvs requested, store it.
    for(size_t i = 0; i < size(); ++i)
    {
      CHECK_NOTNULL(cameras_.at(i).get());
      if (cameras_[i]->label() == "dvs") {
        LOG(INFO) << "Camera rig contains a DVS camera";
        if (FLAGS_vio_use_events_and_images) {
          // Store the dvs camera.
          dvs_camera_ = cameras_.at(i);
          dvs_camera_index_ = i;
          setupDvs();
        } else {
          if (FLAGS_vio_use_events) {
            // Only keep the dvs camera.
            dvs_camera_ = cameras_.at(i);
            dvs_camera_index_ = i;
            setupDvs();
            cameras_.at(0) = cameras_.at(i);
            T_C_B_.at(0) = T_C_B_.at(i);
            cameras_.resize(1);
            T_C_B_.resize(1);
            dvs_camera_index_ = 0;
            CHECK_EQ(T_C_B_.size(), cameras_.size());
          } else {
            // Delete the dvs camera.
            cameras_.erase(cameras_.begin() + i);
            T_C_B_.erase(T_C_B_.begin() + i);
            CHECK_EQ(T_C_B_.size(), cameras_.size());
          }
        }
        break;
      }
    }

    // Only check stereo if we have no dvs camera.
    if (size() > 1u && !hasDvsCamera())
    {
      setStereoPairs(identifyStereoPairsInRig(
                       *this, stereo_min_fov_overlap, stereo_min_baseline));
    }
  }

  void CameraRig::setupDvs() {
    // Go from (K, D) params to (K`, 0) params for dvs camera,
    // while computing a keypoint and bearing LookUp Table
    // for later use in the draw events function.
    CHECK_NOTNULL(dvs_camera_.get());
    // Calculate bearing vectors using intrinsics and distortion parameters
    // of the camera
    calculateBearingLUT(dvs_camera_, &(dvs_bearing_lut_));

    // Now,
    // Ignore distortion for dvs camera from now on.
    //-------------------------------------------------------------------------
    LOG(INFO) << "Ignoring cam distortion for camera with label: "
              << dvs_camera_->label() << " and type: "
              << dvs_camera_->typeAsString();

    // Convert the parsed camera to a PinholeCamera without distortion
    const VectorX& intrinsics = dvs_camera_->projectionParameters();
    const VectorX& distortion = dvs_camera_->distortionParameters();

    cv::Matx33d K;
    K << intrinsics(0), 0, intrinsics(2),
        0, intrinsics(1), intrinsics(3),
        0, 0, 1;

    cv::Matx33d newK;

    std::vector<double> D;
    for(auto i=0; i<distortion.rows(); ++i)
    {
      D.push_back(distortion(i));
    }

    if(FLAGS_vio_rescale_intrinsics_for_distortion)
    {
      if(dvs_camera_->type() == CameraType::PinholeEquidistant)
      {
        cv::Mat K_ref;
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, D,
                                                                cv::Size(dvs_camera_->width(), dvs_camera_->height()),
                                                                cv::Matx33f::eye(),
                                                                K_ref);
        newK = cv::Matx33d(K_ref);
      }
      else if(dvs_camera_->type() == CameraType::PinholeRadialTangential)
      {
        newK = cv::getOptimalNewCameraMatrix(K, D,
                                             cv::Size(dvs_camera_->width(), dvs_camera_->height()),
                                             0);
      }
      else
      {
        LOG(WARNING) << "Unknown camera distortion model. Will not rescale the intrinsics matrix";
        newK = cv::Matx33d(K);
      }
    }
    else
    {
      LOG(INFO) << "Will not rescale the intrinsics matrix";
      newK = cv::Matx33d(K);
    }

    LOG(INFO) << "Original camera intrinsics: " << K;
    LOG(INFO) << "Rescaled camera intrinsics: " << newK;

    // Reset camera with new K and without distortion.
    dvs_camera_ = std::make_shared<ze::PinholeCamera>(ze::createPinholeCamera(
                                  dvs_camera_->width(), dvs_camera_->height(),
                                  newK(0,0), newK(1,1), newK(0,2), newK(1,2)));
    dvs_camera_->setLabel(cameras_.at(dvs_camera_index_)->label());
    cameras_.at(dvs_camera_index_) = dvs_camera_;

    // Calculate keypoint lookup table.
    calculateKeypointLUT(dvs_camera_,
                         dvs_bearing_lut_,
                         &dvs_keypoint_lut_);
  }

  // -----------------------------------------------------------------------------
  CameraRig::Ptr CameraRig::getSubRig(
      const std::vector<uint32_t>& camera_indices,
      const std::string& label)
  {
    CameraVector cameras;
    TransformationVector T;
    for (uint32_t i : camera_indices)
    {
      cameras.push_back(atShared(i));
      T.push_back(T_C_B(i));
    }
    return std::make_shared<CameraRig>(T, cameras, label);
  }

  // -----------------------------------------------------------------------------
  CameraRig::Ptr cameraRigFromYaml(const std::string& yaml_file)
  {
    CHECK(fileExists(yaml_file)) << "File does not exist: " << yaml_file;

    CameraRig::Ptr rig;
    try
    {
      YAML::Node doc = YAML::LoadFile(yaml_file.c_str());
      rig = doc.as<CameraRig::Ptr>();
    }
    catch (const std::exception& ex)
    {
      LOG(ERROR) << "Cannot load CameraRig from file:" << yaml_file << "\n"
                 << ex.what();
      return nullptr;
    }

    return rig;
  }

  // -----------------------------------------------------------------------------
  CameraRig::Ptr cameraRigFromGflags()
  {
    CHECK(fileExists(FLAGS_calib_filename)) << "Camera file does not exist.";
    CameraRig::Ptr rig = cameraRigFromYaml(FLAGS_calib_filename);
    CHECK(rig);

    if(!FLAGS_mask_cam0.empty())
    {
      CHECK_GT(rig->size(), 0u);
      CHECK(fileExists(FLAGS_mask_cam0));
      ImageCv8uC1::Ptr mask;
      cvBridgeLoad<Pixel8uC1>(mask, FLAGS_mask_cam0, PixelOrder::gray);
      rig->atShared(0)->setMask(mask);
    }

    if(!FLAGS_mask_cam1.empty())
    {
      CHECK_GT(rig->size(), 1u);
      CHECK(fileExists(FLAGS_mask_cam1));
      ImageCv8uC1::Ptr mask;
      cvBridgeLoad<Pixel8uC1>(mask, FLAGS_mask_cam1, PixelOrder::gray);
      rig->atShared(1)->setMask(mask);
    }

    if(!FLAGS_mask_cam2.empty())
    {
      CHECK_GT(rig->size(), 2u);
      CHECK(fileExists(FLAGS_mask_cam2));
      ImageCv8uC1::Ptr mask;
      cvBridgeLoad<Pixel8uC1>(mask, FLAGS_mask_cam2, PixelOrder::gray);
      rig->atShared(2)->setMask(mask);
    }

    if(!FLAGS_mask_cam3.empty())
    {
      CHECK_GT(rig->size(), 3u);
      CHECK(fileExists(FLAGS_mask_cam3));
      ImageCv8uC1::Ptr mask;
      cvBridgeLoad<Pixel8uC1>(mask, FLAGS_mask_cam3, PixelOrder::gray);
      rig->atShared(3)->setMask(mask);
    }

    return rig;
  }

  // -----------------------------------------------------------------------------
  void CameraRig::calculateBearingLUT(const std::shared_ptr<const Camera>& c,
                                      Eigen::Matrix<float, 4, Eigen::Dynamic>* dvs_bearing_lut)
  {
    CHECK_NOTNULL(dvs_bearing_lut);
    size_t n = c->height() * c->width();
    dvs_bearing_lut->resize(4, n);

    for (size_t y=0; y != c->height(); ++y)
    {
      for (size_t x=0; x != c->width(); ++x)
      {
        // This back projects keypoints and undistorts the
        // image into bearing vectors.
        ze::Bearing f = c->backProject(ze::Keypoint(x,y));
        dvs_bearing_lut->col(x + y * c->width()) =
            Eigen::Vector4f(f[0], f[1], f[2], 1.);
      }
    }
  }

  void CameraRig::calculateKeypointLUT(const std::shared_ptr<const Camera>& c,
                                       const Eigen::Matrix<float, 4, Eigen::Dynamic>& dvs_bearing_lut,
                                       Eigen::Matrix<float, 2, Eigen::Dynamic>* dvs_keypoint_lut)
  {
    CHECK_NOTNULL(dvs_keypoint_lut);
    size_t n = c->height() * c->width();
    CHECK(n == static_cast<size_t>(dvs_bearing_lut.cols())) << "Size of bearing"
                                                               " lut is not consistent with camera.";

    dvs_keypoint_lut->resize(2, n);
    for (size_t i=0; i != n; ++i)
    {
      ze::Keypoint p = c->project(
            dvs_bearing_lut.col(i).head<3>().cast<double>());
      dvs_keypoint_lut->col(i) = p.cast<float>();
    }
  }

  // -----------------------------------------------------------------------------
  std::ostream& operator<<(std::ostream& out, const CameraRig& rig)
  {
    out << "Camera Rig: \n"
        << "  Label = " << rig.label() << "\n"
        << "  Stereo pairs =" << rig.stereoPairs() << "\n";
    for (size_t i = 0; i < rig.size(); ++i)
    {
      out << "- Camera " << i << "\n"
          << rig.at(i) << "\n"
          << "    T_B_C = \n" << rig.T_C_B(i).inverse() << "\n";
    }
    return out;
  }

  // -----------------------------------------------------------------------------
  std::ostream& operator<<(std::ostream& out, const StereoIndexPairs& stereo_pairs)
  {
    for (auto it : stereo_pairs)
    {
      out << " (" << static_cast<int>(it.first) << ", "
          << static_cast<int>(it.second) << ")";
    }
    return out;
  }

  // -----------------------------------------------------------------------------
  StereoIndexPairs identifyStereoPairsInRig(
      const CameraRig& rig,
      const real_t& min_fov_overlap,
      const real_t& min_baseline)
  {
    StereoIndexPairs pairs;
    for (uint32_t cam_A = 0u; cam_A < rig.size(); ++cam_A)
    {
      for (uint32_t cam_B = cam_A + 1u; cam_B < rig.size(); ++cam_B)
      {
        real_t overlap = overlappingFieldOfView(rig, cam_A, cam_B);
        real_t baseline = (rig.T_C_B(cam_B) * rig.T_C_B(cam_A).inverse()).getPosition().norm();


        if (overlap > min_fov_overlap && baseline > min_baseline)
        {
          VLOG(1) << "Camera " << cam_A << " and " << cam_B
                  << ": Overlap = " << overlap << ", Baseline = " << baseline
                  << " -> Stereo Rig.";
          pairs.push_back(std::make_pair(cam_A, cam_B));
        }
        else
        {
          VLOG(1) << "Camera " << cam_A << " and " << cam_B
                  << ": Overlap = " << overlap << ", Baseline = " << baseline
                  << " -> No stereo rig (baseline or overlap too small)";
        }
      }
    }
    return pairs;
  }

} // namespace ze
