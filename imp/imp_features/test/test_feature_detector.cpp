#include <ze/common/config.hpp>
#ifdef ZE_USE_BRISK
#include <imp/features/brisk_detector.hpp>
#endif
#include <imp/features/feature_detector.hpp>
#include <imp/features/fast_detector.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/imgproc/draw.hpp>
#include <ze/cameras/camera_utils.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/benchmark.hpp>

TEST(ImpFeatureTests, testFast)
{
  using namespace ze;

  // Load an image:
  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  ImageCv8uC1::Ptr img;
  cvBridgeLoad(img, joinPath(data_path, "img", "1.png"), PixelOrder::gray);
  ImagePyramid8uC1::Ptr pyr = createImagePyramidCpu<Pixel8uC1>(img, 0.5, 5, 8);

  // Create grid
  Keypoints keypoints = generateRandomKeypoints(img->size(), 7u, 120u);
  OccupancyGrid2D grid(32, img->size());
  grid.fillWithKeypoints(keypoints);

  // Create detector:
  FastDetectorOptions fast_options;
  fast_options.use_masked_detection = true;
  DetectionStrategy strategy;
  strategy.emplace_back(
        std::make_shared<FastDetector>(fast_options, Size2u(640, 480)));
  FeatureDetectionHandler detector(strategy);

  // Detect:
  uint32_t max_fts = 300u;
  Keypoints px_vec(2, max_fts);
  KeypointScores score_vec(max_fts);
  KeypointLevels level_vec(max_fts);
  KeypointAngles angle_vec(max_fts);
  KeypointTypes type_vec(max_fts);
  Descriptors descriptors;
  uint32_t num_detected = 0u;
  KeypointsWrapper features(px_vec, score_vec, level_vec, angle_vec, type_vec,
                            descriptors, num_detected);
  detector.setGrid(grid);
  detector.detect(*pyr, features);
  EXPECT_EQ(features.num_detected, 202u);

  // Benchmark FAST
  auto detectAndCleanLambda = [&](){
      features.num_detected = 0u; // Reset.
      detector.setGrid(grid);
      detector.detect(*pyr, features);
    };
  runTimingBenchmark(detectAndCleanLambda, 10, 20, "FAST Detector", true);

  EXPECT_EQ(features.num_detected, 202u);

  // Visualize
  if (false)
  {
    drawFeatures<Pixel8uC1>(*img, px_vec, 3, 255);
    cvBridgeShow("img", ImageCv8uC1(*img));
    cv::waitKey(0);
  }
}

#ifdef ZE_USE_BRISK
TEST(ImpFeatureTests, testBrisk)
{
  using namespace ze;

  // Load an image:
  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  std::shared_ptr<ImageCv8uC1> img;
  cvBridgeLoad(img, data_path + "/img/1.png", PixelOrder::gray);
  ImagePyramid8uC1::Ptr pyr = createImagePyramidCpu<Pixel8uC1>(img, 0.5, 5, 8);

  // Create detector:
  BriskDetectorOptions brisk_options;
  DetectionStrategy strategy;
  strategy.emplace_back(
        std::make_shared<BriskDetector>(brisk_options, Size2u(640, 480)));
  FeatureDetectionHandler detector(strategy);

  // Detect:
  uint32_t max_fts = 300u;
  Keypoints px_vec(2, max_fts);
  KeypointScores score_vec(max_fts);
  KeypointLevels level_vec(max_fts);
  KeypointAngles angle_vec(max_fts);
  KeypointTypes type_vec(max_fts);
  Descriptors descriptors;
  uint32_t num_detected = 0u;
  KeypointsWrapper features(px_vec, score_vec, level_vec, angle_vec, type_vec,
                            descriptors, num_detected);
  detector.detect(*pyr, features);

  // Benchmark BRISK
  auto detectAndCleanLambda = [&](){
      features.num_detected = 0u; // Reset.
      detector.detect(*pyr, features);
    };
  runTimingBenchmark(detectAndCleanLambda, 10, 20, "BRISK Detector", true);

  // Visualize
  if (false)
  {
    drawFeatures<Pixel8uC1>(*img, px_vec, 3, 255);
    cvBridgeShow("img", ImageCv8uC1(*img));
    cv::waitKey(0);
  }
}
#endif



ZE_UNITTEST_ENTRYPOINT
