// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <vector>
#include <ze/common/versioned_slot_handle.hpp>

namespace ze {

using LandmarkHandle  = VersionedSlotHandle<uint32_t, 16, 8>;
using LandmarkHandles = std::vector<LandmarkHandle>;

constexpr uint8_t c_landmark_version_deleted   = 0u;
constexpr uint8_t c_landmark_version_invalid   = 1u;
constexpr uint8_t c_landmark_version_min_valid = 2u;

inline bool isValidLandmarkHandle(LandmarkHandle h)
{
  return h.version >= c_landmark_version_min_valid;
}

inline bool isValidLandmarkHandleType(LandmarkHandle::value_t v)
{
  return LandmarkHandle(v).version >= c_landmark_version_min_valid;
}

inline bool isDeletedLandmarkHandle(LandmarkHandle h)
{
  return h.version == c_landmark_version_deleted;
}

inline bool isOutlierLandmarkHandle(LandmarkHandle h)
{
  return h.version == c_landmark_version_invalid;
}

inline void markLandmarkHandleAsInvalid(LandmarkHandle& h)
{
  h.version = c_landmark_version_deleted;
}

inline void markLandmarkHandleAsOutlier(LandmarkHandle& h)
{
  h.version = c_landmark_version_invalid;
}

} // namespace ze
