// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <imp/imgproc/image_pyramid.hpp>
#include <imp/features/keypoints_wrapper.hpp>
#include <ze/common/macros.hpp>
#include <ze/common/noncopyable.hpp>
#include <ze/common/types.hpp>
#include <ze/vio_common/landmark_handle.hpp>

namespace ze {

DECLARE_uint64(vio_frame_pyramid_levels);

class Frame : Noncopyable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ZE_POINTER_TYPEDEFS(Frame);

  Frame() = delete;

  Frame(const Image8uC1::Ptr& img, const int max_n_features);

  void resizeFeatureStorage(int size);

  inline uint32_t featureCapacity() const
  {
    return px_vec_.cols() - num_features_;
  }

  void ensureFeatureCapacity(uint32_t num_additional_features);

  inline int observesLandmark(const LandmarkHandle h_query) const
  {
    //! @todo: Use Sentinel for more efficient search!

    for (uint32_t i = 0; i < num_features_; ++i)
    {
      if (landmark_handles_[i] == h_query)
      {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  inline Eigen::Map<const Eigen::Matrix<LandmarkHandle::value_t, Eigen::Dynamic, 1>>
  getLandmarkHandlesAsVector() const
  {
    return Eigen::Map<const Eigen::Matrix<LandmarkHandle::value_t, Eigen::Dynamic, 1>>(
      reinterpret_cast<const LandmarkHandle::value_t*>(landmark_handles_.data()),
      num_features_);
  }

  KeypointsWrapper getKeypointsWrapper();

  inline uint32_t numSeeds() const
  {
    DEBUG_CHECK_GE(seeds_index_to_, seeds_index_from_);
    return seeds_index_to_ - seeds_index_from_;
  }

  // Data:
  ImagePyramid8uC1::Ptr pyr_;

  // Features:
  Keypoints px_vec_;
  Bearings f_vec_;
  KeypointLevels level_vec_;
  KeypointTypes type_vec_;
  KeypointAngles angle_vec_;
  KeypointScores score_vec_;
  LandmarkHandles landmark_handles_;
  Descriptors descriptors_;
  uint32_t num_features_ { 0u };
  bool is_keyframe_ { false };

  // Seed specific:
  uint32_t seeds_index_from_   {  0u  };
  uint32_t seeds_index_to_     {  0u  };
  real_t seeds_mean_range_  { -1.0 };

  // Statistics:
  real_t min_depth_         { -1.0 };
  real_t max_depth_         { -1.0 };
  real_t median_depth_      { -1.0 };
  uint32_t num_tracked_cached_ {  0u  };
};

std::ostream& operator<<(std::ostream& out, const Frame& frame);

// convenience typedefs
using FrameVector = std::vector<std::shared_ptr<Frame>>;

} // namespace ze
