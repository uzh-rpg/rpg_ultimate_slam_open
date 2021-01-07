// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_common/nframe_table.hpp>

namespace ze {

// -----------------------------------------------------------------------------
NFrameTable::NFrameTable()
{
  reset();
}
// -----------------------------------------------------------------------------
NFrame::Ptr NFrameTable::makeAndStoreNewNFrame(
    const StampedImages& stamped_images,
    const uint32_t max_num_features)
{
  NFrameHandle handle = getNewHandle();
  NFrame::Ptr nf = std::make_shared<NFrame>(handle, stamped_images, max_num_features);
  nframe(handle) = nf;

  return nf;
}

// -----------------------------------------------------------------------------
NFrame::Ptr NFrameTable::makeAndStoreNewEmptyTestNFrame()
{
  NFrameHandle handle = getNewHandle();
  NFrame::Ptr nf = makeEmptyTestNFrame(handle);
  nframe(handle) = nf;
  return nf;
}

// -----------------------------------------------------------------------------
NFrameHandles NFrameTable::getNLastKeyframes(uint32_t num_keyframes) const
{
  CHECK_LE(num_keyframes, c_num_keyframes_);
  NFrameHandles handles;
  handles.reserve(num_keyframes);
  uint32_t count = 0u;
  for (auto it = last_keyframes_.rbegin(); it < last_keyframes_.rend(); ++it)
  {
    DEBUG_CHECK(isValidNFrameHandle(*it));
    DEBUG_CHECK(isStored(*it));
    handles.push_back(*it);
    ++count;
    if (count == num_keyframes)
    {
      break;
    }
  }
  return handles;
}

// -----------------------------------------------------------------------------
NFrameHandle NFrameTable::getNewHandle()
{
  NFrameHandle handle_k;
  if (isValidNFrameHandle(nframe_h_km1_)
      && !nframe(nframe_h_km1_)->isKeyframe())
  {
    // NFrame at time k-2 is not a keyframe, we can use its slot. In this case,
    // we don't need to increase the version counter.
    handle_k = nframe_h_km1_;
  }
  else
  {
    // We overwrite the oldest frame by incrementing the handle.
    end_ = (end_ + 1) % c_capacity_;

    auto goodSlotLambda = [&]() -> bool
    {
      if (   end_ == nframe_h_k_.slot
          || end_ == nframe_h_km1_.slot)
      {
        return false;
      }
      for (const NFrameHandle h : last_keyframes_)
      {
        if (h.slot == end_)
        {
          return false;
        }
      }
      return true;
    };

    while (!goodSlotLambda())
    {
      end_ = (end_ + 1) % c_capacity_;
    }

    // NFrame at time k-2 is a keyframe. In this case, we take the next available
    // slot to store the new frame and increase the version number.
    version_t& v = versions_[end_];
    if (isValidNFrameHandle(nframe_h_km1_))
    {
      DEBUG_CHECK_NE(NFrameHandle(end_, v), nframe_h_km1_);
      DEBUG_CHECK_NE(NFrameHandle(end_, v), nframe_h_k_);
      DEBUG_CHECK_NE(NFrameHandle(end_, v), nframe_h_lkf_);
    }
    v = (v == NFrameHandle::maxVersion()) ? c_nframe_version_min_valid : v + 1u;
    handle_k = NFrameHandle(end_, v);
  }
  DEBUG_CHECK(isValidNFrameHandle(handle_k)) << "Handle is not valid " << handle_k;

  // Reset the states for this handle:
  T_B_W(handle_k) = Transformation();
  v_W(handle_k) = Vector3::Zero();
  nframe(handle_k).reset();

  // Update internal bookkeeping.
  nframe_h_km1_ = nframe_h_k_;
  nframe_h_k_   = handle_k;

  return nframe_h_k_;
}

void NFrameTable::setKeyframe(const NFrameHandle handle)
{
  const NFrame::Ptr& nf = nframe(handle);
  CHECK_NOTNULL(nf.get());
  if (!nf->isKeyframe())
  {
    nf->setKeyframe();
  }
  nframe_h_lkf_ = handle;

  // Save last N keyframes.
  //! @todo: use more efficient datastructure, e.g. ringbuffer.
  last_keyframes_.push_back(nframe_h_lkf_);
  if (last_keyframes_.size() >= c_num_keyframes_)
  {
    last_keyframes_.erase(last_keyframes_.begin());
  }
}

// -----------------------------------------------------------------------------
void NFrameTable::reset()
{
  nframe_h_k_   = NFrameHandle(c_capacity_-1, 0);
  nframe_h_km1_ = NFrameHandle(c_capacity_-2, 0);
  nframe_h_lkf_.reset();
  end_ = c_capacity_ - 1u;
  gyr_bias_k_ = Vector3::Zero();
  acc_bias_k_ = Vector3::Zero();
  versions_.fill(c_nframe_version_invalid);
  T_B_W_.fill(Transformation());
  nframes_.fill(nullptr);
  last_keyframes_.clear();
}

} // namespace ze
