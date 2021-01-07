// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_frontend/refinement.hpp>
#include <ze/vio_frontend/frontend_gflags.hpp>

#include <ze/geometry/pose_optimizer.hpp>
#include <ze/vio_common/landmark_table.hpp>
#include <ze/vio_common/landmark_triangulation.hpp>
#include <ze/vio_common/nframe.hpp>
#include <ze/vio_common/nframe_table.hpp>

namespace ze {

// -----------------------------------------------------------------------------
void optimizeLandmarks(
    const CameraRig& rig,
    const NFrameTable& states,
    NFrame& nframe,
    LandmarkTable& landmarks)
{
  //! @todo: like this, we are triangulating the same landmark multiple times.

  for(size_t frame_idx = 0u; frame_idx < rig.size(); ++frame_idx)
  {
    const Frame& frame = nframe.at(frame_idx);
    for (uint32_t i = 0u; i < frame.num_features_; ++i)
    {
      const LandmarkHandle lm_h = frame.landmark_handles_[i];
      if (isValidLandmarkHandle(lm_h) && landmarks.obs(lm_h).size() >= 2u)
      {
        // Refine landmark.
        optimizeLandmark(states, rig.T_C_B_vec(), lm_h, landmarks);
      }
    }
  }
}

} // namespace ze
