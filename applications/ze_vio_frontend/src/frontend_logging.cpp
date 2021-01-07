// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_frontend/frontend_base.hpp>

#include <ze/common/file_utils.hpp>
#include <ze/vio_common/landmark_table.hpp>
#include <ze/vio_common/landmark_types.hpp>
#include <ze/vio_common/nframe.hpp>

namespace ze {

void FrontendBase::logNumTrackedFeatures(
    const NFrame& nframe,
    const LandmarkTable& landmarks)
{
  static std::ofstream log_num_tracked_features;
  if (!log_num_tracked_features.is_open())
  {
    openOutputFileStream(
          joinPath(FLAGS_log_dir, "frontend_log_num_tracked_features.csv"),
          &log_num_tracked_features);
    log_num_tracked_features << "# seq, cam-idx, seeds, converged_seeds, opportunistic, persistent, outliers\n";
  }

  for (size_t cam_idx = 0u; cam_idx < nframe.size(); ++cam_idx)
  {
    const Frame& frame = nframe.at(cam_idx);

    // Count landmark observations.
    uint32_t num_seeds = 0u;
    uint32_t num_seeds_converged = 0u;
    uint32_t num_opportunistic = 0u;
    uint32_t num_persistent = 0u;
    uint32_t num_outliers = 0u;
    for (uint32_t i = 0u; i < frame.num_features_; ++i)
    {
      LandmarkHandle lm_h = frame.landmark_handles_[i];
      if (isOutlierLandmarkHandle(lm_h))
      {
        ++num_outliers;
        continue;
      }
      if (!landmarks.isStored(lm_h))
      {
        continue;
      }
      LandmarkType type = landmarks.type(lm_h);
      switch (type)
      {
        case LandmarkType::Seed:
          ++num_seeds;
          break;
        case LandmarkType::SeedConverged:
          ++num_seeds_converged;
          break;
        case LandmarkType::Opportunistic:
        case LandmarkType::OpportunisticAbsorbed:
        case LandmarkType::OpportunisticTerminated:
          ++num_opportunistic;
          break;
        case LandmarkType::Persistent:
        case LandmarkType::PersistentTerminated:
          ++num_persistent;
          break;
        default:
          LOG(FATAL) << "Landmark type not known.";
      }
    }

    log_num_tracked_features
        << nframe.seq() << ","
        << cam_idx << ","
        << num_seeds << ","
        << num_seeds_converged << ","
        << num_opportunistic << ","
        << num_persistent << ","
        << num_outliers << "\n";
  }
}

} // namespace ze
