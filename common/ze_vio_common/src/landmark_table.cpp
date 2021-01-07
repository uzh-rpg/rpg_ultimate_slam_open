// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_common/landmark_table.hpp>

namespace ze {

LandmarkTable::LandmarkTable()
{
  num_landmarks_ = 0u;
  versions_.fill(c_landmark_version_invalid);
  types_.fill(LandmarkType::Unknown);
  infos_.fill(LandmarkInfo());
  obs_.fill(LandmarkObsVec());
  last_iter_visible_.fill(0u);
}

// -----------------------------------------------------------------------------
LandmarkHandles LandmarkTable::getNewLandmarkHandles(uint32_t num, uint32_t iter_count)
{
  LandmarkHandles handles;
  handles.reserve(num);
  for (uint32_t i = 0u; i < c_capacity_; ++i)
  {
    if (isLandmarkInactive(types_[i]))
    {
      // Increase the version number of the slot:
      version_t& v = versions_[i];
      v = (v == LandmarkHandle::maxVersion()) ? c_landmark_version_min_valid : v + 1u;

      // Return a handle that is composed of the slot and the version:
      LandmarkHandle handle(i, v);
      DEBUG_CHECK(isValidLandmarkHandle(handle));
      handles.push_back(handle);

      if (handles.size() >= num)
      {
        num_landmarks_ = std::max(i+1u, num_landmarks_);
        break;
      }
    }
  }

  if (handles.size() < num)
  {
    LOG(ERROR) << "Landmark storage: " << typesFormattedString();
    LOG(FATAL) << "Landmark storage too small. Have only space for "
               << handles.size() << " but need " << num;
  }

  // Set types:
  for (const LandmarkHandle h : handles)
  {
    types_[h.slot] = LandmarkType::Seed;
  }

  // Reset info:
  for (const LandmarkHandle h : handles)
  {
    infos_[h.slot] = LandmarkInfo();
  }

  // Reset observations:
  for (const LandmarkHandle h : handles)
  {
    obs_[h.slot].clear();
  }

  // Reset iter count:
  for (const LandmarkHandle h : handles)
  {
    last_iter_visible_[h.slot] = iter_count;
  }

  // Reset track:
  for (const LandmarkHandle h : handles)
  {
    tracks_[h.slot].clear();
  }


  return handles;
}

// -----------------------------------------------------------------------------
void LandmarkTable::addObservation(const LandmarkHandle h, const LandmarkObs& new_obs)
{
  obs(h).push_back(new_obs);

  LandmarkInfo& lm_info = info(h);
  if (lm_info.last_nframe_obs_ != new_obs.nframe_handle_)
  {
    if (lm_info.n_nframe_obs_ < std::numeric_limits<uint8_t>::max())
    {
      ++lm_info.n_nframe_obs_;
    }
    lm_info.last_nframe_obs_ = new_obs.nframe_handle_;
  }

  if (lm_info.n_nframe_obs_ > 11u)
  {
    if (isLandmarkSeed(type(h)))
    {
      VLOG(3) << "Removing Reference observation in seed.";
    }
    obs(h).erase(obs(h).begin(), obs(h).begin() + 1); // TODO use rolling buffer.
  }
}

// -----------------------------------------------------------------------------
std::string LandmarkTable::typesFormattedString() const
{
  std::stringstream s;
  uint32_t n = 0;
  for (const LandmarkType t : types_)
  {
    switch (t)
    {
      case LandmarkType::Unknown: s << 'O'; break;
      case LandmarkType::Outlier: s << 'X'; break;
      case LandmarkType::OpportunisticAbsorbed:
      case LandmarkType::OpportunisticTerminated:
      case LandmarkType::PersistentTerminated: s << '.'; break;
      case LandmarkType::Persistent: s << '$'; break;
      case LandmarkType::Opportunistic: s << '*'; break;
      case LandmarkType::Seed: s << 's'; break;
      case LandmarkType::SeedConverged: s << 'c'; break;
      default:
        LOG(FATAL) << "Type not implemented.";
    }
    ++n;
    if (n == num_landmarks_)
    {
      s << '|';
    }
  }
  return s.str();
}

//------------------------------------------------------------------------------
void LandmarkTable::cleanupInactiveLandmarks()
{
  for (int i = num_landmarks_-1; i >= 0; --i)
  {
    if (isLandmarkActive(types_[i]))
    {
      num_landmarks_ = i + 1;
      break;
    }
    else
    {
      types_[i] = LandmarkType::Unknown;
    }
  }
}

//------------------------------------------------------------------------------
void LandmarkTable::cleanupSeedLandmarks()
{
  for (int i = num_landmarks_-1; i >= 0; --i)
  {
    if (isLandmarkSeed(types_[i]))
    {
      types_[i] = LandmarkType::Unknown;
    }
  }
}

// -----------------------------------------------------------------------------
void LandmarkTable::setLastIterVisible(const LandmarkHandles handles, uint32_t iter)
{
  //! @todo: pass handle iterators because handles may be pre-allocated larger.
  for (LandmarkHandle lm_h : handles)
  {
    if (isValidLandmarkHandle(lm_h))
    {
      last_iter_visible_[lm_h.slot] = iter;
    }
  }
}

} // namespace ze
