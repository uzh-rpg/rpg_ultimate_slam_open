// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <tuple>

#include <ze/common/transformation.hpp>
#include <ze/common/types.hpp>

DECLARE_int32(vio_kfselect_criterion);
DECLARE_int32(vio_kfselect_min_num_frames_between_kfs);
DECLARE_uint64(vio_kfselect_numfts_upper_thresh);
DECLARE_uint64(vio_kfselect_numfts_lower_thresh);
DECLARE_double(vio_kfselect_min_dist);
DECLARE_double(vio_kfselect_min_disparity);

namespace ze {

// fwd
class LandmarkTable;
class NFrame;
class NFrameTable;

//! @return Returns true when new keyframe is necessary.
bool needNewKeyframe(
    const NFrame& nframe_k,
    const NFrame& nframe_lkf,
    const NFrameTable& states,
    const Transformation& T_Bk_W,
    const TransformationVector& T_C_B,
    const uint32_t num_tracked);

} // namespace ze
