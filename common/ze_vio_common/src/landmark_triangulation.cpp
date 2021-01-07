// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_common/landmark_triangulation.hpp>

#include <algorithm>
#include <unordered_map>
#include <ze/common/logging.hpp>

#include <ze/cameras/camera.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <ze/geometry/triangulation.hpp>
#include <ze/vio_common/frame.hpp>
#include <ze/vio_common/landmark_table.hpp>
#include <ze/vio_common/nframe.hpp>
#include <ze/vio_common/nframe_table.hpp>

DEFINE_double(vio_landmark_triangulate_purge_floor, -1,
             "Remove all landmarks that are [m] below the body. Inactive if"
             " value is negative.");

namespace ze {

// -----------------------------------------------------------------------------
std::pair<Vector3, TriangulationResult> triangulateLandmark(
    const NFrameTable& states,
    const TransformationVector& T_C_B,
    const LandmarkObsVec& obs_vec,
    const real_t clip_close,
    const real_t clip_far,
    const NFrameHandle* ignore_nframe)
{
  // Get anchor observation.
  if (obs_vec.empty())
  {
    return std::make_pair(Vector3::Zero(), TriangulationResult::FailNoAnchorObservation);
  }

  // Landmark cannot be triangulated if it has only one observation.
  if (obs_vec.size() == 1u)
  {
    return std::make_pair(Vector3::Zero(), TriangulationResult::FailTooFewObservations);
  }

  // Prepare camera poses and bearing vectors for retriangulation.
  TransformationVector T_C_W;
  Bearings f_C(3, obs_vec.size());
  T_C_W.reserve(obs_vec.size());
  int n = 0;
  for (const LandmarkObs& obs : obs_vec)
  {
    if (ignore_nframe)
    {
      if (*ignore_nframe == obs.nframe_handle_)
      {
        continue; // We may want to ignore an NFrame (e.g. the latest one)
      }
    }

    if (!states.isStored(obs.nframe_handle_))
    {
      //LOG(WARNING) << "Camera pose not in states for triangulation.";
      continue;
    }
    T_C_W.push_back((T_C_B[obs.frame_idx_] * states.T_B_W(obs.nframe_handle_)));
    f_C.col(n++) = obs.f_;
  }

  if (T_C_W.size() < 2u)
  {
    return std::make_pair(Vector3::Zero(), TriangulationResult::FailTooFewObservations);
  }
  f_C.conservativeResize(3, T_C_W.size());

  // Triangulate.
  Vector4 p_W_homogeneous;
  bool success;
  std::tie(p_W_homogeneous, success) = triangulateHomogeneousDLT(T_C_W, f_C);
  if (!success)
  {
    return std::make_pair(Vector3::Zero(), TriangulationResult::FailTriangulation);
  }

  // Update landmark position.
  Vector3 p_W = p_W_homogeneous.head<3>() / p_W_homogeneous(3);

  // Check if landmark is in front of camera.
  Vector3 p_C = T_C_W.front() * p_W;
  if (p_C.z() < 0.0)
  {
    return std::make_pair(p_W, TriangulationResult::FailNegativeDepth);
  }

  // Check if landmark is too far or too close
  if (p_C.z() < clip_close)
  {
    return std::make_pair(p_W, TriangulationResult::FailTooClose);
  }
  else if (p_C.z() > clip_far)
  {
    return std::make_pair(p_W, TriangulationResult::FailTooFar);
  }

  return std::make_pair(p_W, TriangulationResult::Success);
}

// -----------------------------------------------------------------------------
uint32_t retriangulateAllLandmarks(
    const NFrameTable& states,
    const TransformationVector& T_C_B,
    LandmarkTable& landmarks,
    bool set_all_failures_to_outlier)
{
  uint32_t num_failed_too_few_obs = 0u;
  uint32_t num_failed_no_anchor_obs = 0u;
  uint32_t num_failed_states_not_available = 0u;
  uint32_t num_failed_triangulation = 0u;
  uint32_t num_failed_negative_depth = 0u;
  uint32_t num_failed_too_far = 0u;
  uint32_t num_failed_too_close = 0u;
  uint32_t num_aborted_persistent = 0u;
  uint32_t num_updated_3d_points = 0u;
  for (uint32_t i = 0u; i < landmarks.numLandmarks(); ++i)
  {
    LandmarkType& type = landmarks.typeAtSlot(i);
    if (isLandmarkInactive(type))
    {
      continue;
    }

    // Triangulate
    const LandmarkObsVec& obs_vec = landmarks.obsAtSlot(i);
    Vector3 p_W;
    TriangulationResult res;
    std::tie(p_W, res) = triangulateLandmark(states, T_C_B, obs_vec);
    switch (res)
    {
      case TriangulationResult::Success:
        landmarks.p_W_atSlot(i) = p_W;
        ++num_updated_3d_points;
        break;
      case TriangulationResult::FailTooFewObservations:
        ++num_failed_too_few_obs;
        break;
      case TriangulationResult::FailNoAnchorObservation:
        ++num_failed_no_anchor_obs;
        break;
      case TriangulationResult::FailStatesNotAvailable:
        ++num_failed_states_not_available;
        break;
      case TriangulationResult::FailTriangulation:
        ++num_failed_triangulation;
        break;
      case TriangulationResult::FailNegativeDepth:
        ++num_failed_negative_depth;
        break;
      case TriangulationResult::FailTooFar:
        ++num_failed_too_far;
        break;
      case TriangulationResult::FailTooClose:
        ++num_failed_too_close;
        break;
      default:
        LOG(FATAL) << "Triangulation result not known.";
        break;
    }

    if (isLandmarkSeed(type))
    {
      if (obs_vec.size() == 1u)
      {
        // In this case, we were not able to re-triangulate the point, hence,
        // we need to update the cached seed position.
        const LandmarkObs& obs = obs_vec[0u];
        if (states.isStored(obs.nframe_handle_))
        {
          const Transformation T_W_Cref =
              (T_C_B[obs.frame_idx_] * states.T_B_W(obs.nframe_handle_)).inverse();
          landmarks.p_W_atSlot(i) = T_W_Cref * (obs.f_ / landmarks.seedAtSlot(i)(0));
        }
        else
        {
          LOG(WARNING) << "Reference frame not stored anymore.";
        }
      }
      else if (res == TriangulationResult::Success)
      {
        // In this case, we were able to triangulate the landmark and need to
        // update the seed depth.
        DEBUG_CHECK(!obs_vec.empty());
        const LandmarkObs& obs = obs_vec[0u];
        if (states.isStored(obs.nframe_handle_))
        {
          const Transformation T_Cref_W =
              T_C_B[obs.frame_idx_] * states.T_B_W(obs.nframe_handle_);
          landmarks.seedAtSlot(i)(0) = real_t{1.0} / (T_Cref_W * p_W).norm();
        }
        else
        {
          LOG(WARNING) << "Reference frame not stored anymore.";
        }
      }
    }


    if (FLAGS_vio_landmark_triangulate_purge_floor > 0)
    {
      real_t dist = states.T_Bk_W().inverse().getPosition().z() - p_W.z();
      if (dist > FLAGS_vio_landmark_triangulate_purge_floor)
      {
        type = LandmarkType::Outlier;
      }
    }

    if ((set_all_failures_to_outlier || type == LandmarkType::Persistent)
        && res != TriangulationResult::Success)
    {
      ++num_aborted_persistent;
      type = LandmarkType::Outlier;
    }
  }

  VLOG(3) << "Successfully retriangulated points = " << num_updated_3d_points;
  VLOG_IF(1, num_aborted_persistent)
      << "Aborted persistent tracks after triangulation = " << num_aborted_persistent;
  VLOG_IF(1, num_failed_too_few_obs)
      << "Triangulation fail too few observations = " << num_failed_too_few_obs;
  VLOG_IF(1, num_failed_no_anchor_obs)
      << "Triangulation fail no anchor observations = " << num_failed_no_anchor_obs;
  VLOG_IF(1, num_failed_states_not_available)
      << "Triangulation fail state not available = " << num_failed_states_not_available;
  VLOG_IF(1, num_failed_triangulation)
      << "Triangulation fail triangulation = " << num_failed_triangulation;
  VLOG_IF(1, num_failed_negative_depth)
      << "Triangulation fail point behind camera = " << num_failed_negative_depth;
  VLOG_IF(1, num_failed_too_far)
      << "Triangulation fail point too far = " << num_failed_too_far;
  VLOG_IF(1, num_failed_too_close)
      << "Triangulation fail point too close = " << num_failed_too_close;
  return num_updated_3d_points;
}

// -----------------------------------------------------------------------------
real_t getLandmarkParallaxDegrees(
    const NFrameTable& states,
    const TransformationVector& T_C_B,
    const LandmarkObsVec& obs,
    const Position& p_W,
    const NFrameHandle* ignore_nframe,
    const uint8_t* frame_idx)
{
  // Get all the observing bearing vectors (in world coordinates).
  Bearings f(3, obs.size());
  int n = 0;
  for (int i = 0; i < f.cols(); ++i)
  {
    // It may be specified that we want to measure the parallax only across
    // observations from the same camera.
    if (frame_idx && *frame_idx != obs[i].frame_idx_)
    {
      continue;
    }

    // Check if we should ignore this nframe.
    if (ignore_nframe && *ignore_nframe == obs[i].nframe_handle_)
    {
      continue;
    }

    // Check if the pose of this nframe is available.
    if (!states.isStored(obs[i].nframe_handle_))
    {
      continue;
    }

    Position W_p_C =
        (T_C_B[obs[i].frame_idx_]
        * states.T_B_W(obs[i].nframe_handle_)).inverse().getPosition();
    f.col(n++) = W_p_C - p_W;
  }
  f.conservativeResize(3,n);
  normalizeBearings(f);

  if (n <= 1)
  {
    return 0.0;
  }

  // Compare parallax with first observation.
  real_t min_cos_angle = 1.0;
  int min_cos_angle_idx = 0;
  for (int i = 1; i < n; ++i)
  {
    real_t cos_angle = f.col(0).dot(f.col(i));
    if (cos_angle < min_cos_angle)
    {
      min_cos_angle = cos_angle;
      min_cos_angle_idx = i;
    }
  }

  if (n <= 2)
  {
    return std::acos(min_cos_angle) * 180.0 / M_PI;
  }

  // To make sure, we find the max parallax, we do the same again but this time
  // with the max distant from the previous loop.
  min_cos_angle = 1.0;
  for (int i = 0; i < n; ++i)
  {
    if (i == min_cos_angle_idx)
    {
      continue;
    }

    real_t cos_angle = f.col(min_cos_angle_idx).dot(f.col(i));
    min_cos_angle = std::min(min_cos_angle, cos_angle);
  }

  return std::acos(min_cos_angle) * 180.0 / M_PI;
}

// -----------------------------------------------------------------------------
void optimizeLandmark(
    const NFrameTable& states,
    const TransformationVector& T_C_B,
    const LandmarkHandle& lm_h,
    LandmarkTable& landmarks)
{
  // Get data.
  const LandmarkObsVec& obs_vec = landmarks.obs(lm_h);
  if (obs_vec.size() < 2u)
  {
    LOG(ERROR) << "Optimizing point with less than two observations";
    return;
  }
  TransformationVector T_C_W_vec;
  T_C_W_vec.reserve(obs_vec.size());
  Bearings p_C(3, obs_vec.size());
  size_t n = 0u;
  for (size_t i = 0u; i < obs_vec.size(); ++i)
  {
    const LandmarkObs& obs = obs_vec[i];
    if (states.isStored(obs.nframe_handle_))
    {
      T_C_W_vec.push_back(T_C_B[obs.frame_idx_] * states.T_B_W(obs.nframe_handle_));
      p_C.col(n++) = obs.f_;
    }
  }
  p_C.conservativeResize(3, n);

  // Nonlinear refinement of triangulation.
  triangulateGaussNewton(T_C_W_vec, p_C, landmarks.p_W(lm_h));
}

} // namespace ze
