// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_common/landmark_types.hpp>

namespace ze {

std::ostream& operator<<(std::ostream& out, const LandmarkObs& obs)
{
  out << "NFrame = " << obs.nframe_handle_ << "\t "
      << "Frame-Idx = " << static_cast<int>(obs.frame_idx_) << "\t "
      << "Keypoint-Idx = " << static_cast<int>(obs.keypoint_idx_);
  return out;
}

std::ostream& operator<<(std::ostream& out, const LandmarkObsVec& obs_vec)
{
  for (const LandmarkObs& obs : obs_vec)
  {
    out << obs << "\n";
  }
  return out;
}

std::string landmarkTypeAsString(const LandmarkType t)
{
  switch (t)
  {
    case LandmarkType::Unknown:
      return "Unknown"; break;
    case LandmarkType::Outlier:
      return "Outlier"; break;
    case LandmarkType::Seed:
      return "Seed"; break;
    case LandmarkType::SeedConverged:
      return "SeedConverged"; break;
    case LandmarkType::Opportunistic:
      return "Opportunistic"; break;
    case LandmarkType::OpportunisticAbsorbed:
      return "OpportunisticAbsorbed"; break;
    case LandmarkType::OpportunisticTerminated:
      return "OpportunisticTerminated"; break;
    case LandmarkType::Persistent:
      return "Persistent"; break;
    case LandmarkType::PersistentTerminated:
      return "PersistentTerminated"; break;
    default:
      LOG(FATAL) << "Landmark type unknown."; break;
  }
  return "";
}

} // namespace ze
