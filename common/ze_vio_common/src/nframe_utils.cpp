// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_common/nframe_utils.hpp>

#include <ze/common/combinatorics.hpp>
#include <ze/common/statistics.hpp>
#include <ze/vio_common/landmark_utils.hpp>
#include <ze/vio_common/landmark_table.hpp>
#include <ze/vio_common/nframe.hpp>
#include <ze/vio_common/nframe_handle.hpp>

namespace ze {

//------------------------------------------------------------------------------
std::vector<real_t> getDisparitiesSq(
    const NFrame& nframe_cur,
    const NFrame& nframe_ref)
{
  std::vector<real_t> disparities_sq;
  for (size_t i = 0u; i < nframe_cur.size(); ++i)
  {
    getDisparitiesSq(nframe_cur.at(i), nframe_ref.at(i), disparities_sq);
  }
  return disparities_sq;
}

//------------------------------------------------------------------------------
void getDisparitiesSq(
    const Frame& frame_cur,
    const Frame& frame_ref,
    std::vector<real_t>& disparities_sq)
{
  // First, find those features that have a match in the other frame.
  std::vector<std::pair<uint32_t, uint32_t>> cur_ref_matches =
      getMatchIndices<LandmarkHandle::value_t>(
        frame_cur.getLandmarkHandlesAsVector(),
        frame_ref.getLandmarkHandlesAsVector(),
        std::bind(&isValidLandmarkHandleType, std::placeholders::_1));

  // Compute squared disparities.
  disparities_sq.reserve(disparities_sq.size() + cur_ref_matches.size());
  for (const std::pair<uint32_t, uint32_t>& cur_ref : cur_ref_matches)
  {
    disparities_sq.push_back(
          (frame_cur.px_vec_.col(cur_ref.first)
           - frame_ref.px_vec_.col(cur_ref.second)).squaredNorm());
  }
}

//------------------------------------------------------------------------------
void setSceneDepthInNFrame(
    NFrame& nframe,
    const LandmarkTable& landmarks,
    const Transformation& T_B_W,
    const TransformationVector& T_C_B,
    const real_t default_min_depth,
    const real_t default_max_depth,
    const real_t default_median_depth)
{
  DEBUG_CHECK_EQ(T_C_B.size(), nframe.size());

  for (uint32_t frame_idx = 0u; frame_idx < nframe.size(); ++frame_idx)
  {
    const Transformation T_C_W = T_C_B[frame_idx] * T_B_W;
    Frame& frame = nframe.at(frame_idx);
    std::vector<real_t> depths;
    depths.reserve(frame.num_features_);
    for(uint32_t i = 0u; i < frame.num_features_; ++i)
    {
      const LandmarkHandle lm_h = frame.landmark_handles_[i];
      if (!landmarks.isStored(lm_h) || !landmarks.isActive(lm_h))
      {
        continue;
      }

      const Position p_C = T_C_W * landmarks.p_W(lm_h);
      if (p_C.z() < real_t{0.0})
      {
        VLOG(1) << "Negative depth.";
        continue;
      }

      depths.push_back(p_C.norm());
    }

    if(depths.empty())
    {
      frame.num_tracked_cached_ = 0;
      frame.median_depth_ = default_median_depth;
      frame.min_depth_ = default_min_depth;
      frame.max_depth_ = default_max_depth;
      VLOG(1) << "Cam-" << frame_idx << ": "
                   << "Too few features to compute scene depths. "
                   << "Set default:"
                   << " min = " << frame.min_depth_
                   << ", max = " << frame.max_depth_
                   << ", median = " << frame.median_depth_;
    }
    else
    {
      frame.num_tracked_cached_ = depths.size();
      frame.median_depth_ = median<real_t>(depths).first;
      auto res = std::minmax_element(depths.begin(), depths.end());
      frame.min_depth_ = *res.first;
      frame.max_depth_ = *res.second;
      VLOG(10) << "Cam-" << frame_idx << ": "
               << " depth min = " << frame.min_depth_
               << ", max = " << frame.max_depth_
               << ", median = " << frame.median_depth_
               << ", from " << depths.size()
               << " measurements.";
    }
  }
}

// -----------------------------------------------------------------------------
std::pair<uint32_t, uint32_t> getCameraWithLeastFeatures(
    const NFrame& nframe,
    const LandmarkTable& landmarks)
{
  uint32_t min_features = std::numeric_limits<uint32_t>::max();
  uint32_t min_features_cam_idx = 0u;
  for (uint32_t frame_idx = 0u; frame_idx < nframe.size(); ++frame_idx)
  {
    uint32_t num_tracked = 0u;
    const Frame& frame = nframe.at(frame_idx);
    for (uint32_t i = 0u; i < frame.num_features_; ++i)
    {
      LandmarkHandle lm_h = frame.landmark_handles_[i];
      if (landmarks.isStored(lm_h, true))
      {
        if (isLandmarkActive(landmarks.type(lm_h)))
        {
          ++num_tracked;
        }
      }
    }
    if (num_tracked < min_features)
    {
      min_features = num_tracked;
      min_features_cam_idx = frame_idx;
    }
  }

  return std::make_pair(min_features_cam_idx, min_features);
}

// -----------------------------------------------------------------------------
std::vector<uint32_t> getCamerasWithoutFeatures(const NFrame& nframe)
{
  std::vector<uint32_t> empty_frame_indices;
  for (uint32_t i = 0u; i < nframe.size(); ++i)
  {
    if (nframe.at(i).num_features_ == 0)
    {
      empty_frame_indices.push_back(i);
    }
  }
  return empty_frame_indices;
}

} // namespace ze
