// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_frontend/keyframe_selection.hpp>

#include <ze/common/combinatorics.hpp>
#include <ze/common/statistics.hpp>
#include <ze/vio_common/landmark_table.hpp>
#include <ze/vio_common/nframe_table.hpp>
#include <ze/vio_common/nframe_utils.hpp>

DEFINE_int32(vio_kfselect_criterion, 1,
             "0: Downlooking, 1: Forward");
DEFINE_int32(vio_kfselect_min_num_frames_between_kfs, 0,
             "Minimum number of frames between keyframes.");
DEFINE_uint64(vio_kfselect_numfts_upper_thresh, 180,
              "If we track more than this amount of features, we will never select "
              " a new keyframe.");
DEFINE_uint64(vio_kfselect_numfts_lower_thresh, 90,
              "Force key-frame selection if we fall below this number of features.");
DEFINE_double(vio_kfselect_min_dist, 0.12,
              "Minimum Euclidean distance between frames for a new Keyframe [m].");
DEFINE_double(vio_kfselect_min_disparity, 30.0,
              "Minimum disparity to select a new keyframe [pixels].");
DEFINE_int32(vio_kfselect_every_nth_frame, -1,
             "Select a new keyframe every n'th time.");
DEFINE_int32(vio_kfselect_min_every_nth_frame, -1,
             "At least select a new keyframe every n'th time.");
DEFINE_int32(vio_first_n_frames_as_keyframes, 0, "Add the first n frames as KF.");

namespace ze {

//------------------------------------------------------------------------------
bool needNewKeyframe(
    const NFrame& nframe_k,
    const NFrame& nframe_lkf,
    const NFrameTable& states,
    const Transformation& T_Bk_W,
    const TransformationVector& T_C_B,
    const uint32_t num_tracked)
{
  if (FLAGS_vio_first_n_frames_as_keyframes * 5 > nframe_k.seq()
      && nframe_k.seq() % 5 == 0)
  {
    VLOG(1) << "NEW KEYFRAME: first_n_frames_as_keyframes";
    return true;
  }

  if (FLAGS_vio_kfselect_min_every_nth_frame > 0)
  {
    if (nframe_k.seq() - nframe_lkf.seq() >= FLAGS_vio_kfselect_min_every_nth_frame)
    {
      VLOG(1) << "NEW KEYFRAME: kfselect_min_every_nth_frame";
      return true;
    }
  }

  if (FLAGS_vio_kfselect_every_nth_frame > 0)
  {
    if (nframe_k.seq() % FLAGS_vio_kfselect_every_nth_frame == 0)
    {
      VLOG(1) << "NEW KEYFRAME: kfselect_every_nth_frame";
      return true;
    }
    return false;
  }

  if (FLAGS_vio_kfselect_criterion == 0u)
  {
    // Iterate over all keyframes and check relative pose.
    real_t median_depth = nframe_k.at(0u).median_depth_;
    CHECK_GT(median_depth, 0.0);
    Transformation T_Ck_W = T_C_B[0] * T_Bk_W;
    NFrameHandles handles = states.getNLastKeyframes(5);
    for (NFrameHandle handle : handles)
    {
      Vector3 relpos = T_Ck_W * states.T_B_W(handle).inverse().getPosition();
      if(   std::abs(relpos.x()) / median_depth < FLAGS_vio_kfselect_min_dist
         && std::abs(relpos.y()) / median_depth < FLAGS_vio_kfselect_min_dist * 0.8
         && std::abs(relpos.z()) / median_depth < FLAGS_vio_kfselect_min_dist * 1.3)
      {
        return false;
      }
    }
    VLOG(1) << "NEW KEYFRAME: kfselect_min_dist";
    return true;
  }

  // else, FORWARD:
  if (num_tracked > FLAGS_vio_kfselect_numfts_upper_thresh)
  {
    VLOG(3) << "KF Select: NO - Have sufficient tracked features: "
            << num_tracked;
    return false;
  }

  // TODO: this only works for mono!
  if (nframe_k.seq() - nframe_lkf.seq()
      < FLAGS_vio_kfselect_min_num_frames_between_kfs)
  {
    VLOG(3) << "KF Select: NO - Recent frame was already keyframe: ";
    return false;
  }

  if (num_tracked < FLAGS_vio_kfselect_numfts_lower_thresh)
  {
    VLOG(1) << "NEW KEYFRAME: kfselect_numfts_lower_thresh. " << num_tracked;
    return true;
  }

  // Compute median disparity.
  std::vector<real_t> disparities_sq = getDisparitiesSq(nframe_k, nframe_lkf);
  if (!disparities_sq.empty())
  {
    real_t median_disparity = std::sqrt(ze::median(disparities_sq).first);
    VLOG(3) << "KF Select: Disparity since last KF = " << median_disparity;
    if (median_disparity < FLAGS_vio_kfselect_min_disparity)
    {
      return false;
    }
  }
  VLOG(1) << "NEW KEYFRAME: kfselect_min_disparity";
  return true;
}


} // namespac ze
