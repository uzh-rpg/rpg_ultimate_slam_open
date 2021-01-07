// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <gflags/gflags.h>

DECLARE_double(vio_min_depth);
DECLARE_double(vio_max_depth);
DECLARE_double(vio_median_depth);

DECLARE_uint64(vio_min_tracked_features_total);
DECLARE_uint64(vio_max_tracked_features_per_frame);

DECLARE_bool(vio_delayed_nframe_processing);