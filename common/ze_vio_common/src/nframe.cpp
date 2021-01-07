// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_common/nframe.hpp>

#include <ze/common/time_conversions.hpp>
#include <ze/vio_common/frame.hpp>

namespace ze {

NFrame::NFrame(
    const NFrameHandle handle,
    const StampedImages& stamped_imgs,
    const uint32_t max_num_features)
  : handle_(handle)
  , timestamp_(stamped_imgs.at(0).first)
{
  DEBUG_CHECK(isValidNFrameHandle(handle));
  for (const std::pair<int64_t, ImageBase::Ptr>& it : stamped_imgs)
  {
    // Make sure we received a gray-scale image.
    CHECK(it.second->pixelType() == PixelType::i8uC1);
    CHECK(it.second->pixelOrder() == PixelOrder::gray);
    Image8uC1::Ptr img = it.second->as<Image8uC1>();
    CHECK(img);

    // Create frame and check that all frames in bundle have close timestamp.
    frames_.emplace_back(std::make_shared<Frame>(img, max_num_features));
    real_t dt_millisec = nanosecToMillisecTrunc(std::abs(it.first - timestamp_));
    CHECK_LT(dt_millisec, 2.0);
  }
}

NFrame::NFrame(const NFrameHandle handle)
  : handle_(handle)
{}

std::ostream& operator<<(std::ostream& out, const NFrame& nframe)
{
  out << "NFrame:\n"
      << "  seq = " << nframe.seq() << "\n"
      << "  handle = " << nframe.handle().handle
      << " (slot = " << nframe.handle().slot << ", " << nframe.handle().version << ")\n"
      << "  timestamp  = " << nframe.timestamp() << "\n";
  for (size_t i = 0u; i < nframe.size(); ++i)
  {
    out << "- Frame " << i << ":\n"
        << nframe.at(i) << "\n";
  }
  return out;
}

namespace {
class NFrameProtectedAccess : public NFrame {
public:
  NFrameProtectedAccess(const NFrameHandle handle)
    : NFrame(handle)
  { }
};
}

NFrame::Ptr makeEmptyTestNFrame(const NFrameHandle handle)
{
  return std::make_shared<NFrameProtectedAccess>(handle);
}

} // namespace ze
