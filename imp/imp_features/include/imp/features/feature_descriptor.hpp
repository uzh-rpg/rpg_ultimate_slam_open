#pragma once

#include <imp/features/keypoints_wrapper.hpp>
#include <imp/imgproc/image_pyramid.hpp>
#include <ze/common/config.hpp>
#include <ze/common/types.hpp>
#include <ze/common/macros.hpp>

namespace ze {

// fwd
class CameraRig;

// -----------------------------------------------------------------------------
enum class DescriptorType : int8_t
{
  Undefined,
  Dummy,
  Brisk
};

// -----------------------------------------------------------------------------
class AbstractDescriptorExtractor
{
public:
  ZE_POINTER_TYPEDEFS(AbstractDescriptorExtractor);

  AbstractDescriptorExtractor() = delete;
  virtual ~AbstractDescriptorExtractor() = default;

  AbstractDescriptorExtractor(DescriptorType type);

  void setKeypointOrientationWithGravity(
      const Vector3& gravity,
      KeypointsWrapper& keypoints);

  virtual void extract(const ImagePyramid8uC1& pyr,
                        KeypointsWrapper& keypoints) = 0;

protected:
  DescriptorType type_;
};

// -----------------------------------------------------------------------------
class DummyDescriptorExtractor : public AbstractDescriptorExtractor
{
public:
  ZE_POINTER_TYPEDEFS(DummyDescriptorExtractor);

  DummyDescriptorExtractor();
  virtual ~DummyDescriptorExtractor() = default;

  virtual void extract(const ImagePyramid8uC1& pyr,
                        KeypointsWrapper& keypoints) override;
};

// -----------------------------------------------------------------------------
//! Loads the feature detector as specified with gflag variables.
std::vector<std::shared_ptr<AbstractDescriptorExtractor>>
loadVioDescriptorExtractorFromGflags(const CameraRig& rig);


} // namespace ze
