// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <map>
#include <iostream>
#include <string>

#include <imp/core/image.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <ze/common/macros.hpp>
#include <ze/common/noncopyable.hpp>
#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>
#include <ze/vio_common/frame.hpp>
#include <ze/vio_common/nframe_handle.hpp>

namespace ze {

class NFrame : Noncopyable
{
public:
  ZE_POINTER_TYPEDEFS(NFrame);

  NFrame() = delete;

  //! Default constructor.
  NFrame(const NFrameHandle handle,
         const StampedImages& stamped_images,
         const uint32_t max_num_features);

protected:
  //! Only used for Unit-Tests. Use makeEmptyTestNFrame().
  NFrame(const NFrameHandle handle);

public:
  //! Get const reference to frame at index i.
  inline const Frame& at(uint32_t i) const
  {
    DEBUG_CHECK_LT(i, frames_.size());
    return *frames_[i];
  }

  //! Get reference to frame at index i.
  inline Frame& at(uint32_t i)
  {
    DEBUG_CHECK_LT(i, frames_.size());
    return *frames_[i];
  }

  //! Get unique handle of frame.
  inline NFrameHandle handle() const noexcept
  {
    return handle_;
  }

  //! Get nanoseconds timestamp.
  inline int64_t timestamp() const noexcept
  {
    return timestamp_;
  }

  //! Get sequence-id (id that increases by one with every new frame).
  inline int32_t seq() const noexcept
  {
    return seq_;
  }

  //! Set sequence id.
  inline void setSeq(int32_t seq) noexcept
  {
    seq_ = seq;
  }

  //! Number of frames in the NFrame.
  inline size_t size() const noexcept
  {
    return frames_.size();
  }

  //! Was this NFrame selected as keyframe, i.e. will it be stored in the map?
  inline bool isKeyframe() const noexcept
  {
    return is_keyframe_;
  }

  //! Mark this NFrame as keyframe.
  inline void setKeyframe() noexcept
  {
    is_keyframe_ = true;
  }

  //! Number of features in all frames. Includes outliers matches.
  inline uint32_t getNumKeypoints() const
  {
    uint32_t n = 0;
    std::for_each(cbegin(), cend(), [&](const Frame::Ptr& frame)
                  { n += frame->num_features_; });
    return n;
  }

  //! @name Making class iterable.
  //! @{
  typedef FrameVector::value_type value_type;
  typedef FrameVector::iterator iterator;
  typedef FrameVector::const_iterator const_iterator;
  FrameVector::iterator begin() { return frames_.begin(); }
  FrameVector::iterator end() { return frames_.end(); }
  FrameVector::const_iterator begin() const { return frames_.begin(); }
  FrameVector::const_iterator end() const { return frames_.end(); }
  FrameVector::const_iterator cbegin() const { return frames_.cbegin(); }
  FrameVector::const_iterator cend() const { return frames_.cend(); }
  //! @}

protected:
  const NFrameHandle handle_;     //!< Unique identifier.
  const int64_t timestamp_ = -1;  //!< Nanosecond timestamp.
  int32_t seq_ = -1;              //!< Sequence-id of the frame, i.e. a counter.
  FrameVector frames_;            //!< Vector of frames holding images and features.
  bool is_keyframe_ = false;      //!< Was this NFrame selected as keyframe?
};

std::ostream& operator<<(std::ostream& out, const NFrame& nframe);

//! Only used for NFrame testing.
NFrame::Ptr makeEmptyTestNFrame(const NFrameHandle handle);

} // namespace ze
