#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/benchmark.hpp>
#include <imp/features/brisk_detector.hpp>
#include <imp/features/brisk_descriptor.hpp>
#include <imp/features/feature_detector.hpp>
#include <imp/features/feature_descriptor.hpp>
#include <imp/features/fast_detector.hpp>

#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/imgproc/draw.hpp>

#ifdef ZE_USE_BRISK
TEST(ImpFeatureTests, testBrisk)
{
  using namespace ze;

  // Load an image:
  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  ImageCv8uC1::Ptr img;
  cvBridgeLoad(img, joinPath(data_path, "img", "1.png"), PixelOrder::gray);
  ImagePyramid8uC1::Ptr pyr = createImagePyramidCpu<Pixel8uC1>(img, 0.5, 5, 8);

  // Create detector:
  BriskDetectorOptions brisk_options;
  DetectionStrategy strategy;
  strategy.emplace_back(
        std::make_shared<BriskDetector>(brisk_options, Size2u(640, 480)));
  FeatureDetectionHandler detector(strategy);

  // Create descriptor:
  AbstractDescriptorExtractor::Ptr extractor = std::make_shared<BriskDescriptorExtractor>();

  // Detect:
  int max_fts = 750;
  Keypoints px_vec(2, max_fts);
  KeypointScores score_vec(max_fts);
  KeypointLevels level_vec(max_fts);
  KeypointAngles angle_vec(max_fts);
  KeypointTypes type_vec(max_fts);
  Descriptors descriptors;
  uint32_t num_detected = 0;
  KeypointsWrapper features(px_vec, score_vec, level_vec, angle_vec, type_vec,
                            descriptors, num_detected);
  detector.detect(*pyr, features);
  VLOG(1) << "Detected " << features.num_detected << " BRISK features.";

  // Benchmark BRISK Extractor
  auto computeDescriptorsLambda = [&](){
    extractor->extract(*pyr, features);
  };
  runTimingBenchmark(computeDescriptorsLambda, 1, 10, "BRISK Descriptor", true);

  // Visualize
  if(false)
  {
    drawFeatures<Pixel8uC1>(*img, px_vec, 3, 255);
    cvBridgeShow("img", ImageCv8uC1(*img));
    cv::waitKey(0);
  }
}
#endif

ZE_UNITTEST_ENTRYPOINT
