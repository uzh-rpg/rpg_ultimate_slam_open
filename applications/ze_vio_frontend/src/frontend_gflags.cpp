// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_frontend/frontend_gflags.hpp>

// Scene characteristics:
DEFINE_double(vio_min_depth,    0.3,
              "Depth range minimum [m]");
DEFINE_double(vio_max_depth,   50.0,
              "Depth range maximum [m]");
DEFINE_double(vio_median_depth, 6.0,
              "Depth range average [m]");

// Number of tracks:
DEFINE_uint64(vio_min_tracked_features_total, 300,
              "Numer of tracked features in total.");
DEFINE_uint64(vio_max_tracked_features_per_frame, 120,
              "Maximum number of features per frame that we want to match.");

// Backend:
DEFINE_bool(vio_delayed_nframe_processing, true,
            "Optimize frame_km1 and not frame_k (SWE default).");
