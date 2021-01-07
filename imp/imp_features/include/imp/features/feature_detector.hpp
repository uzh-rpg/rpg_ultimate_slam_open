#pragma once

#include <ze/common/config.hpp>
#include <ze/common/types.hpp>
#include <ze/common/macros.hpp>
#include <imp/imgproc/image_pyramid.hpp>
#include <imp/features/keypoints_wrapper.hpp>
#include <imp/features/occupancy_grid_2d.hpp>
#include <opencv2/features2d/features2d.hpp>

DECLARE_string(imp_detector_name);
DECLARE_int32(imp_detector_grid_size);
DECLARE_int32(imp_detector_border_margin);
DECLARE_int32(imp_detector_num_octaves);
DECLARE_int32(imp_detector_max_features_per_frame);

namespace ze {

// fwd
class AbstractDetector;
class CameraRig;
using DetectionStrategy = std::vector<std::shared_ptr<AbstractDetector>>;

// -----------------------------------------------------------------------------
//! Available feature detector types. The value is stored together with the features.
enum class DetectorType : int8_t
{
  Undefined,
  Fast,    //!< Fast Corner Detector by Edward Rosten.
  Grad,    //!< Gradient detector for edgelets.
  Brisk,   //!< Brisk Harris Detector.
  Sift,    //!< Sift Detector.
  Orb,     //!< ORB Detector.
};

// -----------------------------------------------------------------------------
//! Feature detector interface that should be used. Can take a list of detectors
//! that are applied in sequence.
class FeatureDetectionHandler
{
public:
  ZE_POINTER_TYPEDEFS(FeatureDetectionHandler);

  FeatureDetectionHandler(const DetectionStrategy& detectors);

  void detect(
      const ImagePyramid8uC1& pyr,
      KeypointsWrapper& keypoints,
      bool mask_existing_keypoints = true);

  inline void setMask(const Image8uC1::ConstPtr& mask) { mask_ = mask; }

  void setGrid(const OccupancyGrid2D& grid);

private:
  FeatureDetectionHandler() = default;
  DetectionStrategy detectors_;
  Image8uC1::ConstPtr mask_ = nullptr;
};

// -----------------------------------------------------------------------------
//! All detectors should derive from this abstract class.
class AbstractDetector
{
public:
  ZE_POINTER_TYPEDEFS(AbstractDetector);
  ZE_DELETE_COPY_ASSIGN(AbstractDetector);

protected:
  //! Default constructor used by derived detectors.
  AbstractDetector(const Size2u& image_size, DetectorType type);

  DetectorType type_;
  Size2u image_size_;

public:
  AbstractDetector() = delete;
  virtual ~AbstractDetector() = default;

  //! Detect features. Returns number of detected keypoints.
  virtual uint32_t detect(const ImagePyramid8uC1& pyr, KeypointsWrapper& keypoints) = 0;

  //! Set previously detected features. This can be used to mask regions that
  //! have already features.
  inline virtual void setExistingKeypoints(const Eigen::Ref<const Keypoints>& keypoints)
  {
    // Default implementation does nothing.
  }

  //! Set mask.
  inline virtual void setMask(const Image8uC1::ConstPtr& /*mask*/)
  {
    // Default implementation does nothing.
  }

  //! Set occupancy grid.
  inline virtual void setGrid(const OccupancyGrid2D& /*grid*/)
  {
    // Default implementation does nothing.
  }

  //! Reset the detector. May be necessary to undo the masking.
  inline virtual void reset()
  {
    // Default implementation does nothing.
  }
};

// -----------------------------------------------------------------------------
//! Loads the feature detector as specified with gflag variables.
std::vector<std::shared_ptr<FeatureDetectionHandler>>
loadVioDetectorsFromGflags(const CameraRig& rig);

} // namespace ze
