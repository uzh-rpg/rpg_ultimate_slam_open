// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <vector>
#include <ze/common/versioned_slot_handle.hpp>

namespace ze {

using NFrameHandle    = VersionedSlotHandle<uint16_t, 8, 8>;
using NFrameHandles   = std::vector<NFrameHandle>;

constexpr uint8_t c_nframe_version_invalid   = 0u;
constexpr uint8_t c_nframe_version_min_valid = 1u;

inline bool isValidNFrameHandle(NFrameHandle h)
{
  return h.version >= c_nframe_version_min_valid;
}

} // namespace ze
