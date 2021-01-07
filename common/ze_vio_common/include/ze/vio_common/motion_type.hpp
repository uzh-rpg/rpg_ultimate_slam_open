// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <cstdint>

namespace ze {

//! Motion type of frame.
enum class VioMotionType : int8_t
{
  NotComputed,
  Invalid,
  RotationOnly,
  NoMotion,
  GeneralMotion
};

} // namespace ze
