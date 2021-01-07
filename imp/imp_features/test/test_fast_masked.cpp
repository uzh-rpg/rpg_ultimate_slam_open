#include <fast/fast.h>
#include <imp/features/feature_detector.hpp>
#include <imp/features/fast_detector.hpp>
#include <imp/features/fast_masked.hpp>
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
  int level = 0;
  Keypoints keypoints = generateRandomKeypoints(img->size(), 7u, 120u);
  OccupancyGrid2D grid(32, img->size());

  ImageRaw8uC1 grid_img(pyr->at(level).size());
  grid.fillWithKeypoints(keypoints);
  grid.visualizeGrid(grid_img, 1.0 / pyr->scaleFactor(level));

  int threshold = 10;

  // Baseline fast detection:
  {
    std::vector<fast::fast_xy> corners;
    auto detectLambda = [&]()
    {
      corners.clear();
#ifdef __ARM_NEON__
      fast_corner_detect_9_neon(
            reinterpret_cast<fast::fast_byte*>(pyr->at(level).data()),
            pyr->at(level).width(), pyr->at(level).height(),
            pyr->at(level).stride(), threshold, corners);
#else
      fast_corner_detect_10_sse2(
            reinterpret_cast<fast::fast_byte*>(pyr->at(level).data()),
            pyr->at(level).width(), pyr->at(level).height(),
            pyr->at(level).stride(), threshold, corners);
#endif
    };
    runTimingBenchmark(detectLambda, 1, 20, "FAST original", true);
  }

  // Masked fast detection:
  std::vector<Corner> corners;
  auto detectMaskedLambda = [&]()
  {
    corners.clear();
    fasterCornerDetectMasked(
          reinterpret_cast<uint8_t*>(pyr->at(level).data()),
          pyr->at(level).width(), pyr->at(level).height(),
          pyr->at(level).stride(), threshold, grid, level, 31, corners);
  };
  runTimingBenchmark(detectMaskedLambda, 1, 20, "FAST masked", true);

  // Nonmaxima suppression
  std::vector<int> scores;
  auto nonmaxLambda = [&]()
  {
    scores.clear();
    scores.reserve(corners.size());
#ifdef __ARM_NEON__
    fast::fast_corner_score_9(
          reinterpret_cast<fast::fast_byte*>(pyr->at(level).data()),
          pyr->at(level).stride(), corners, threshold, scores);
#else
    fast::fast_corner_score_10(
          reinterpret_cast<fast::fast_byte*>(pyr->at(level).data()),
          pyr->at(level).stride(), corners, threshold, scores);
#endif
  };
  runTimingBenchmark(nonmaxLambda, 1, 20, "FAST nonmax", true);


  VLOG(1) << "Detected " << corners.size() << " corners";
  for (const Corner& corner : corners)
  {
    drawFeature(grid_img, Keypoint(corner.x, corner.y), 1, Pixel8uC1(255));
  }

  // Visualize
  if (false)
  {
    cvBridgeShow("grid_img", ImageCv8uC1(grid_img));
    cv::waitKey(0);
  }

}

ZE_UNITTEST_ENTRYPOINT
