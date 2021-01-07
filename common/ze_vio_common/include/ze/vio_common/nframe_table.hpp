// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <array>

#include <ze/common/transformation.hpp>
#include <ze/common/types.hpp>
#include <ze/vio_common/nframe.hpp>
#include <ze/vio_common/nframe_handle.hpp>

namespace ze {

class NFrameTable
{
public:
  static constexpr uint32_t c_capacity_ = 20u;
  static constexpr uint32_t c_num_keyframes_ = c_capacity_ - 4;

  using version_t = uint8_t;

  NFrameTable();

  //----------------------------------------------------------------------------
  // Handle utils
  NFrame::Ptr makeAndStoreNewNFrame(
      const StampedImages& stamped_images,
      const uint32_t max_num_features);

  //! Setting as keyframe means that the frame will be stored in the table.
  void setKeyframe(const NFrameHandle handle);

  //! Check if the NFrame with this handle is stored in the table.
  inline bool isStored(const NFrameHandle handle) const
  {
    DEBUG_CHECK(isValidNFrameHandle(handle)) << "Handle is not valid " << handle;
    DEBUG_CHECK_LT(handle.slot, c_capacity_);
    return (handle.version == versions_[handle.slot]);
  }

  //----------------------------------------------------------------------------
  // Past NFrames
  inline NFrameHandle nframeHandleK() const { return nframe_h_k_; }
  inline NFrameHandle nframeHandleKm1() const { return nframe_h_km1_; }
  inline NFrameHandle nframeHandleLkf() const { return nframe_h_lkf_; }
  inline NFrame::Ptr nframeK()   const { return nframe(nframe_h_k_); }
  inline NFrame::Ptr nframeKm1() const { return nframe(nframe_h_km1_); }
  inline NFrame::Ptr nframeLkf() const { return nframe(nframe_h_lkf_); }

  //! Get last keyframes sorted such that last added keyframe is first.
  NFrameHandles getNLastKeyframes(uint32_t num_keyframes = c_num_keyframes_) const;

  //----------------------------------------------------------------------------
  // Pose
  inline const Transformation& T_B_W(const NFrameHandle handle) const
  {
    DEBUG_CHECK(isStored(handle)) << "Not stored: " << handle;
    return T_B_W_[handle.slot];
  }

  inline Transformation& T_B_W(const NFrameHandle handle)
  {
    DEBUG_CHECK(isStored(handle)) << "Not stored: " << handle;
    return T_B_W_[handle.slot];
  }

  //! Get pose of current frame.
  inline const Transformation& T_Bk_W() const
  {
    DEBUG_CHECK(isStored(nframe_h_k_)) << "Not stored: " << nframe_h_k_;
    return T_B_W_[nframe_h_k_.slot];
  }

  inline Transformation& T_Bk_W()
  {
    DEBUG_CHECK(isStored(nframe_h_k_)) << "Not stored: " << nframe_h_k_;
    return T_B_W_[nframe_h_k_.slot];
  }

  //! Get pose of last frame.
  inline const Transformation& T_Bkm1_W() const
  {
    DEBUG_CHECK(isStored(nframe_h_km1_)) << "Not stored: " << nframe_h_km1_;
    return T_B_W_[nframe_h_km1_.slot];
  }

  inline Transformation& T_Bkm1_W()
  {
    DEBUG_CHECK(isStored(nframe_h_km1_)) << "Not stored: " << nframe_h_km1_;
    return T_B_W_[nframe_h_km1_.slot];
  }

  //! Get pose of last key-frame (lkf)
  inline const Transformation& T_Blkf_W() const
  {
    DEBUG_CHECK(isStored(nframe_h_lkf_)) << "Not stored: " << nframe_h_lkf_;
    return T_B_W_[nframe_h_lkf_.slot];
  }

  inline Transformation& T_Blkf_W()
  {
    DEBUG_CHECK(isStored(nframe_h_lkf_)) << "Not stored: " << nframe_h_lkf_;
    return T_B_W_[nframe_h_lkf_.slot];
  }

  inline const std::array<Transformation, c_capacity_>& T_B_W_data() const
  {
    return T_B_W_;
  }

  inline TransformationVector& T_B_W_backend() { return T_B_W_in_backend_; }
  inline const TransformationVector& T_B_W_backend() const { return T_B_W_in_backend_; }

  //----------------------------------------------------------------------------
  // Velocity
  inline const Vector3& v_W(const NFrameHandle handle) const
  {
    DEBUG_CHECK(isStored(handle)) << "Not stored: " << handle;
    return v_W_[handle.slot];
  }

  inline Vector3& v_W(const NFrameHandle handle)
  {
    DEBUG_CHECK(isStored(handle)) << "Not stored: " << handle;
    return v_W_[handle.slot];
  }

  //----------------------------------------------------------------------------
  // Biases
  inline const Vector3& accBias() const
  {
    return acc_bias_k_;
  }

  inline Vector3& accBias()
  {
    return acc_bias_k_;
  }

  inline const Vector3& gyrBias() const
  {
    return gyr_bias_k_;
  }

  inline Vector3& gyrBias()
  {
    return gyr_bias_k_;
  }

  //----------------------------------------------------------------------------
  // Frame
  inline const NFrame::Ptr& nframe(const NFrameHandle handle) const
  {
    if (!isValidNFrameHandle(handle) || !isStored(handle))
    {
      LOG(WARNING) << "Accessing nframe that is not valid.";
    }
    return nframes_[handle.slot];
  }

  inline NFrame::Ptr& nframe(const NFrameHandle handle)
  {
    if (!isValidNFrameHandle(handle) || !isStored(handle))
    {
      LOG(WARNING) << "Accessing nframe that is not valid.";
    }
    return nframes_[handle.slot];
  }

  //----------------------------------------------------------------------------
  // Storage utils
  void reset();

  //----------------------------------------------------------------------------
  // Testing
  NFrame::Ptr makeAndStoreNewEmptyTestNFrame();

private:
  NFrameHandle getNewHandle();

  NFrameHandle nframe_h_k_;     //!< NFrame at time k.
  NFrameHandle nframe_h_km1_;   //!< NFrame at time k-1.
  NFrameHandle nframe_h_lkf_;   //!< Last Key-NFrame.
  Vector3 gyr_bias_k_ = Vector3::Zero();
  Vector3 acc_bias_k_ = Vector3::Zero();

  uint32_t end_ = 0u;
  std::array<version_t, c_capacity_> versions_;
  std::array<Transformation, c_capacity_> T_B_W_;
  std::array<Vector3, c_capacity_> v_W_;
  std::array<NFrame::Ptr, c_capacity_> nframes_;
  NFrameHandles last_keyframes_;
  TransformationVector T_B_W_in_backend_;
};

} // namespace ze
