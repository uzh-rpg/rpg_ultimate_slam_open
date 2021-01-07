// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include<ze/common/transformation.hpp>
#include <ze/common/types.hpp>

namespace ze {

//------------------------------------------------------------------------------
// Localization States.
enum class LocalizationState : uint8_t {
  // No reference map has been set, localization is not performed.
  kUninitialized,
  // Baseframe transformation has not yet been initialized.
  kNotLocalized,
  // Baseframe was initialized and global map matching is performed.
  kLocalized,
  // Map matching is performed using map tracking.
  kMapTracking
};

// A Structure to summarize the localization states for visualization.
struct LocalizationInformation {
  //! The current localization state.
  LocalizationState localization_state = LocalizationState::kUninitialized;

  //! The number of landmarks used for localization
  uint32_t n_localization_landmarks = 0;

  //! Transformation from Mission to Global.
  Transformation T_G_M;
};

} // namespace ze
