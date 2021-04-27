// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <array>
#include <ze/common/logging.hpp>

#include <ze/common/types.hpp>
#include <ze/vio_common/nframe_handle.hpp>

namespace ze {

// -----------------------------------------------------------------------------
struct LandmarkObs
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LandmarkObs() = delete;

  LandmarkObs(const NFrameHandle nframe_id,
              const uint8_t cam_index,
              const uint16_t keypoint_index,
              const Eigen::Ref<const Bearing>& f)
    : f_(f)
    , nframe_handle_(nframe_id)
    , keypoint_idx_(keypoint_index)
    , frame_idx_(cam_index)
  {}

  Bearing f_;
  NFrameHandle nframe_handle_;
  uint16_t keypoint_idx_;
  uint8_t frame_idx_;
};
using LandmarkObsVec =
  std::vector<LandmarkObs, Eigen::aligned_allocator<LandmarkObs>>;
using KeypointsVec =
  std::vector<Keypoint, Eigen::aligned_allocator<Keypoint>>;

std::ostream& operator<<(std::ostream& out, const LandmarkObs& obs);
std::ostream& operator<<(std::ostream& out, const LandmarkObsVec& obs_vec);

// -----------------------------------------------------------------------------
enum class LandmarkType : int8_t
{
  Unknown = 0,
  Outlier = 1,
  Seed = 2,
  SeedConverged = 3,
  Opportunistic = 4,
  OpportunisticAbsorbed = 5,
  OpportunisticTerminated = 6,
  Persistent = 7,
  PersistentTerminated = 8
};
using LandmarkTypes = std::vector<LandmarkType>;

inline bool isLandmarkInactive(const LandmarkType t)
{
  return (t == LandmarkType::Unknown
          || t == LandmarkType::Outlier
          || t == LandmarkType::OpportunisticAbsorbed
          || t == LandmarkType::OpportunisticTerminated
          || t == LandmarkType::PersistentTerminated);
}

inline bool isLandmarkActive(const LandmarkType t)
{
  return !isLandmarkInactive(t);
}

inline bool isLandmarkTriangulated(const LandmarkType t)
{
  return (t == LandmarkType::SeedConverged
          || t == LandmarkType::Opportunistic
          || t == LandmarkType::Persistent);
}

inline bool isLandmarkSeed(const LandmarkType t)
{
  return (t == LandmarkType::Seed
          || t == LandmarkType::SeedConverged);
}

std::string landmarkTypeAsString(const LandmarkType t);

// -----------------------------------------------------------------------------
struct LandmarkInfo
{
  NFrameHandle last_nframe_obs_;
  uint8_t n_nframe_obs_ = 0u;
  uint8_t n_succ_proj_ = 0u;
  uint8_t n_fail_proj_ = 0u;
  bool localization_landmark = false;
};

} // namespace ze
