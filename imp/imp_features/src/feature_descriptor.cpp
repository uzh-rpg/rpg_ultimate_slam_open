#include <imp/features/feature_descriptor.hpp>

#ifdef ZE_USE_BRISK
#include <imp/features/brisk_descriptor.hpp>
#endif
#include <ze/cameras/camera_rig.hpp>

DEFINE_bool(vio_descriptor_use_dummy, false, "");

namespace ze {

// -----------------------------------------------------------------------------
AbstractDescriptorExtractor::AbstractDescriptorExtractor(DescriptorType type)
  : type_(type)
{}

void AbstractDescriptorExtractor::setKeypointOrientationWithGravity(
    const Vector3& /*gravity*/,
    KeypointsWrapper& /*keypoints*/)
{
  //! @todo
}

// -----------------------------------------------------------------------------
DummyDescriptorExtractor::DummyDescriptorExtractor()
  : AbstractDescriptorExtractor(DescriptorType::Dummy)
{}

void DummyDescriptorExtractor::extract(
      const ImagePyramid8uC1& /*pyr*/, KeypointsWrapper& keypoints)
{
  keypoints.descriptors = Descriptors(1, keypoints.num_detected);
}

// -----------------------------------------------------------------------------
std::vector<std::shared_ptr<AbstractDescriptorExtractor>>
loadVioDescriptorExtractorFromGflags(const CameraRig& rig)
{
  std::vector<std::shared_ptr<AbstractDescriptorExtractor>> descriptors;

  if (FLAGS_vio_descriptor_use_dummy)
  {
    for (const Camera::Ptr& cam : rig)
    {
      descriptors.emplace_back(std::make_shared<DummyDescriptorExtractor>());
    }
  }
#ifdef ZE_USE_BRISK
  else
  {
    BriskDescriptorOptions brisk_options;
    for (const Camera::Ptr& cam : rig)
    {
      descriptors.emplace_back(
            std::make_shared<BriskDescriptorExtractor>(brisk_options));
    }
  }
#endif

  return descriptors;
}

} // namespace ze
