// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_common/frame.hpp>

namespace ze {

DEFINE_uint64(vio_frame_pyramid_levels, 3,
            "Number of levels in image pyramid");

Frame::Frame(const Image8uC1::Ptr& img, const int max_num_features)
{
  pyr_ = createImagePyramidCpu<Pixel8uC1>(img, 0.5, FLAGS_vio_frame_pyramid_levels);
  resizeFeatureStorage(max_num_features);

  // For debugging, ensure that when wrong frame is accessed, it fails.
  level_vec_.setConstant(std::numeric_limits<KeypointLevel>::max());
}

void Frame::ensureFeatureCapacity(uint32_t num_additional_features)
{
  if (num_additional_features > featureCapacity())
  {
    LOG(WARNING) << "Feature storage capacity is not enough. Need "
                 << (num_additional_features - featureCapacity()) << " more.";
    resizeFeatureStorage(num_features_ + num_additional_features);
  }
}

void Frame::resizeFeatureStorage(int size)
{
  if(px_vec_.cols() < size)
  {
    const int num_new = size - num_features_;

    px_vec_.conservativeResize(Eigen::NoChange, size);
    f_vec_.conservativeResize(Eigen::NoChange, size);
    level_vec_.conservativeResize(size);
    type_vec_.conservativeResize(size);
    angle_vec_.conservativeResize(size);
    score_vec_.conservativeResize(size);
    landmark_handles_.resize(size, LandmarkHandle());

    // initial values
    level_vec_.tail(num_new).setConstant(-1);
    type_vec_.tail(num_new).setConstant(-1);
    angle_vec_.tail(num_new).setConstant(-1.0);
    score_vec_.tail(num_new).setConstant(-1.0);
  }
  else if(size < px_vec_.cols())
  {
    LOG(FATAL) << "Downsizing storage not implemented. "
               << "current storage = " << px_vec_.cols() << ", "
               << "desired storage = " << size << ", "
               << "num features = " << num_features_;
  }
}

KeypointsWrapper Frame::getKeypointsWrapper()
{
  return KeypointsWrapper(
        px_vec_, score_vec_, level_vec_, angle_vec_, type_vec_,
        descriptors_, num_features_);
}

std::ostream& operator<<(std::ostream& out, const Frame& frame)
{
  out << "  num pyr levels = " << frame.pyr_->numLevels() << "\n"
      << "  num features = " << frame.num_features_ << "\n"
      << "  num feature storage = " << frame.px_vec_.cols() << "\n"
      << "  seed range = (" << frame.seeds_index_from_ << ", "
                            << frame.seeds_index_to_ << ")\n"
      << "  seed depth range = " << frame.seeds_mean_range_;
  return out;
}

} // namespace ze
