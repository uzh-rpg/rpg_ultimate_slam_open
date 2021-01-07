// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <array>
#include <string>
#include <ze/common/logging.hpp>

#include <ze/common/types.hpp>
#include <ze/vio_common/landmark_handle.hpp>
#include <ze/vio_common/landmark_types.hpp>

namespace ze {

class LandmarkTable
{
public:
  static constexpr uint32_t c_capacity_ = 3000u;
  using LandmarkPositions = Eigen::Matrix<real_t, 3, c_capacity_>;
  using LandmarkSeeds = Eigen::Matrix<real_t, 4, c_capacity_>;
  using version_t = uint8_t;

  LandmarkTable();

  LandmarkHandles getNewLandmarkHandles(uint32_t num, uint32_t iter_count);

  inline bool isStored(const LandmarkHandle handle, bool silent = false) const
  {
    if (!isValidLandmarkHandle(handle))
    {
      return false;
    }
    CHECK_LT(handle.slot, c_capacity_);
    if (handle.version == versions_[handle.slot])
    {
      return true;
    }
    LOG_IF(WARNING, !silent) << "Landmark is not stored.";
    return false;
  }

  inline LandmarkHandle getHandleAtSlot(uint32_t slot) const
  {
    return LandmarkHandle(slot, versions_[slot]);
  }

  // ---------------------------------------------------------------------------
  // Access Landmark Type:
  inline const LandmarkType& type(const LandmarkHandle handle) const
  {
    return types_[handle.slot];
  }

  inline LandmarkType& type(const LandmarkHandle handle)
  {
    return types_[handle.slot];
  }

  inline const LandmarkType& typeAtSlot(const uint32_t slot) const
  {
    return types_[slot];
  }

  inline LandmarkType& typeAtSlot(const uint32_t slot)
  {
    return types_[slot];
  }

  inline std::array<LandmarkType, c_capacity_>& types()
  {
    return types_;
  }

  inline const std::array<LandmarkType, c_capacity_>& types() const
  {
    return types_;
  }

  std::string typesFormattedString() const;

  // ---------------------------------------------------------------------------
  // Access Landmark Position:
  inline const Eigen::Ref<const Position> p_W(const LandmarkHandle handle) const
  {
    return p_W_.col(handle.slot);
  }

  inline Eigen::Ref<Position> p_W(const LandmarkHandle handle)
  {
    return p_W_.col(handle.slot);
  }

  inline const Eigen::Ref<const Position> p_W_atSlot(const uint32_t slot) const
  {
    return p_W_.col(slot);
  }

  inline Eigen::Ref<Position> p_W_atSlot(const uint32_t slot)
  {
    return p_W_.col(slot);
  }

  inline const LandmarkPositions& p_W() const
  {
    return p_W_;
  }

  inline LandmarkPositions& p_W()
  {
    return p_W_;
  }

  // ---------------------------------------------------------------------------
  // Access Landmark Seeds:
  inline const Eigen::Ref<const Seed> seed(const LandmarkHandle handle) const
  {
    return seeds_.col(handle.slot);
  }

  inline Eigen::Ref<Seed> seed(const LandmarkHandle handle)
  {
    return seeds_.col(handle.slot);
  }

  inline const Eigen::Ref<const Seed> seedAtSlot(const uint32_t slot) const
  {
    return seeds_.col(slot);
  }

  inline Eigen::Ref<Seed> seedAtSlot(const uint32_t slot)
  {
    return seeds_.col(slot);
  }

  inline const LandmarkSeeds& seeds() const
  {
    return seeds_;
  }

  inline LandmarkSeeds& seeds()
  {
    return seeds_;
  }

  // ---------------------------------------------------------------------------
  // Access Landmark Info:
  inline const LandmarkInfo& info(const LandmarkHandle handle) const
  {
    return infos_[handle.slot];
  }

  inline LandmarkInfo& info(const LandmarkHandle handle)
  {
    return infos_[handle.slot];
  }

  inline const LandmarkInfo& infoAtSlot(const uint32_t slot) const
  {
    return infos_[slot];
  }

  inline LandmarkInfo& infoAtSlot(const uint32_t slot)
  {
    return infos_[slot];
  }

  inline const std::array<LandmarkInfo, c_capacity_>& infos() const
  {
    return infos_;
  }

  inline std::array<LandmarkInfo, c_capacity_>& infos()
  {
    return infos_;
  }

  // ---------------------------------------------------------------------------
  // Landmark Observations:

  void addObservation(const LandmarkHandle h, const LandmarkObs& new_obs);

  inline const LandmarkObsVec& obs(const LandmarkHandle handle) const
  {
    return obs_[handle.slot];
  }

  inline LandmarkObsVec& obs(const LandmarkHandle handle)
  {
    return obs_[handle.slot];
  }

  inline const LandmarkObsVec& obsAtSlot(const uint32_t slot) const
  {
    return obs_[slot];
  }

  inline LandmarkObsVec& obsAtSlot(const uint32_t slot)
  {
    return obs_[slot];
  }

  inline const std::array<LandmarkObsVec, c_capacity_>& obs() const
  {
    return obs_;
  }

  inline std::array<LandmarkObsVec, c_capacity_>& obs()
  {
    return obs_;
  }

  inline const LandmarkObs& anchorObs(const LandmarkHandle h) const
  {
    const LandmarkObsVec& obs_vec = obs(h);
    DEBUG_CHECK(!obs_vec.empty());
    return obs_vec[0u];
  }

  inline const LandmarkObs& anchorObsAtSlot(const uint32_t slot) const
  {
    const LandmarkObsVec& obs_vec = obs_[slot];
    DEBUG_CHECK(!obs_vec.empty());
    return obs_vec[0u];
  }

  inline const LandmarkObs& lastObs(const LandmarkHandle h) const
  {
    return obs(h).back();
  }

  inline const LandmarkObs& lastObsAtSlot(const uint32_t slot) const
  {
    return obs_[slot].back();
  }

  inline size_t numObs(const LandmarkHandle h) const
  {
    return obs(h).size();
  }

  inline size_t numObsAtSlot(const uint32_t slot) const
  {
    return obsAtSlot(slot).size();
  }

  inline NFrameHandle lastObsNFrameHandle(const LandmarkHandle h) const
  {
    return info(h).last_nframe_obs_;
  }

  inline NFrameHandle lastObsNFrameHandleAtSlot(const uint32_t slot) const
  {
    return infos_[slot].last_nframe_obs_;
  }

  inline uint32_t numNFrameObs(const LandmarkHandle h) const
  {
    return info(h).n_nframe_obs_;
  }

  inline uint32_t numNFrameObsAtSlot(const uint32_t slot) const
  {
    return infos_[slot].n_nframe_obs_;
  }

  // ---------------------------------------------------------------------------
  // Access Versions:
  inline std::array<version_t, c_capacity_>& versions()
  {
    return versions_;
  }

  inline const std::array<version_t, c_capacity_>& versions() const
  {
    return versions_;
  }

  // ---------------------------------------------------------------------------
  // At which iteration was the landmark last seen.
  inline uint32_t lastIterVisible(const LandmarkHandle handle) const
  {
    return last_iter_visible_[handle.slot];
  }

  inline uint32_t& lastIterVisible(const LandmarkHandle handle)
  {
    return last_iter_visible_[handle.slot];
  }

  inline uint32_t lastIterVisibleAtSlot(const uint32_t slot) const
  {
    return last_iter_visible_[slot];
  }

  inline uint32_t& lastIterVisibleAtSlot(const uint32_t slot)
  {
    return last_iter_visible_[slot];
  }

  void setLastIterVisible(const LandmarkHandles handles, uint32_t iter);

  // ---------------------------------------------------------------------------
  // Utilities:
  inline uint32_t numLandmarks() const
  {
    return num_landmarks_;
  }

  inline void setNumLandmark(uint32_t n_landmarks)
  {
    num_landmarks_ = n_landmarks;
  }

  inline uint8_t numSuccessfulProjections(const LandmarkHandle h) const
  {
    return info(h).n_succ_proj_;
  }

  inline int projectionQualityAtSlot(const uint32_t slot) const
  {
    const LandmarkInfo& i = infoAtSlot(slot);
    return static_cast<int>(i.n_succ_proj_) - i.n_fail_proj_;
  }

  inline int projectionQuality(const LandmarkHandle h) const
  {
    return projectionQualityAtSlot(h.slot);
  }

  inline void incrementSuccessfulProjectionsAtSlot(const uint32_t slot)
  {
    LandmarkInfo& i = infoAtSlot(slot);
    if (i.n_succ_proj_ < std::numeric_limits<uint8_t>::max())
    {
      ++i.n_succ_proj_;
    }
  }

  inline void incrementSuccessfulProjections(const LandmarkHandle h)
  {
    incrementSuccessfulProjectionsAtSlot(h.slot);
  }

  inline void incrementFailedProjectionsAtSlot(const uint32_t slot, const uint8_t num = 1u)
  {
    LandmarkInfo& i = infoAtSlot(slot);
    if (i.n_fail_proj_ < (std::numeric_limits<uint8_t>::max() - 10u))
    {
      i.n_fail_proj_ += num;
    }
  }

  inline void incrementFailedProjections(const LandmarkHandle h, const uint8_t num = 1u)
  {
    incrementFailedProjectionsAtSlot(h.slot, num);
  }

  inline bool isTriangulated(const LandmarkHandle h) const
  {
    return info(h).n_succ_proj_ > 0;
  }

  inline bool isSlotActive(const uint32_t slot) const
  {
    return isLandmarkActive(types_[slot]);
  }

  inline bool isActive(const LandmarkHandle h) const
  {
    return isLandmarkActive(types_[h.slot]);
  }

  inline void setAsLocalizationLandmark(const LandmarkHandle h)
  {
    info(h).localization_landmark = true;
  }

  inline bool isLocalizationLandmark(const LandmarkHandle h) const
  {
    return info(h).localization_landmark;
  }

  // ---------------------------------------------------------------------------
  // Reset:
  inline void resetLandmark(const LandmarkHandle h)
  {
    resetLandmarkAtSlot(h.slot);
  }

  inline void resetLandmarkAtSlot(const uint32_t slot)
  {
    types_[slot] = LandmarkType::Unknown;
    version_t& v = versions_[slot];
    v = (v == LandmarkHandle::maxVersion()) ? 2u : v + 1u;
  }

  void cleanupInactiveLandmarks();

  void cleanupSeedLandmarks();


  inline KeypointsVec& track(const LandmarkHandle handle)
  {
    return tracks_[handle.slot];
  }

  inline const std::array<KeypointsVec, c_capacity_>& tracks() const
  {
    return tracks_;
  }

  inline std::array<KeypointsVec, c_capacity_>& tracks()
  {
    return tracks_;
  }

private:
  uint32_t num_landmarks_;
  std::array<version_t, c_capacity_> versions_;
  std::array<LandmarkType, c_capacity_> types_;
  LandmarkPositions p_W_;
  std::array<LandmarkInfo, c_capacity_> infos_;
  std::array<LandmarkObsVec, c_capacity_> obs_;
  std::array<KeypointsVec, c_capacity_> tracks_;
  std::array<uint32_t, c_capacity_> last_iter_visible_;
  LandmarkSeeds seeds_;
};

} // namespace ze
