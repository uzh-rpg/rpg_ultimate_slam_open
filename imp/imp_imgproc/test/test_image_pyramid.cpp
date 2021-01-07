#include <functional>

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/benchmark.hpp>

#include <imp/core/image.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/imgproc/image_pyramid.hpp>
#include <imp/imgproc/half_sample.hpp>

TEST(ImpPyramidTest, testHalfSample)
{
  using namespace ze;

  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  ImageCv8uC1::Ptr img = cvBridgeLoad8uC1(joinPath(data_path, "img", "1.png"));  

  ImageRaw8uC1 img_half1(img->width()/2, img->height()/2);
  ImageRaw8uC1 img_half2(img->width()/2, img->height()/2);

  // Without SIMD (SSE, NEON)
  halfSample(*img, img_half1, false);

  // With SIMD
  halfSample(*img, img_half2, true);

  // Check that SIMD works and images are equal:
  for(uint32_t y = 0; y < 10; ++y)
  {
    for(uint32_t x = 0; x < 10; ++x)
    {
      // TODO: they are not perfectly equal due to rounding errors!!
      EXPECT_NEAR(img_half1(x,y), img_half2(x,y), 2);
    }
  }

  // Benchmark
  auto fun1 = std::bind(halfSample, *img, std::ref(img_half1), false);
  runTimingBenchmark(fun1, 100, 100, "halfSample", true);

  auto fun2 = std::bind(halfSample, *img, std::ref(img_half2), true);
  runTimingBenchmark(fun2, 100, 100, "halfSample SIMD", true);

  if(false)
  {
    cvBridgeShow("img", *img);
    cvBridgeShow("img_half", ImageCv8uC1(img_half1));
    cvBridgeShow("img_half SIMD", ImageCv8uC1(img_half2));
    cv::waitKey(0);
  }
}

TEST(ImpPyramidTest, testImagePyramidCPU)
{
  using namespace ze;

  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  std::shared_ptr<ImageCv8uC1> img;
  cvBridgeLoad(img, joinPath(data_path, "img", "1.png"), PixelOrder::gray);

  auto pyr = createImagePyramidCpu<Pixel8uC1>(img, 0.5, 5, 8);
  if(false)
  {
    cvBridgeShow("img_pyr 1", ImageCv8uC1(pyr->at(1)));
    cvBridgeShow("img_pyr 2", ImageCv8uC1(pyr->at(2)));
    cvBridgeShow("img_pyr 3", ImageCv8uC1(pyr->at(3)));
    cvBridgeShow("img_pyr 4", ImageCv8uC1(pyr->at(4)));
    cv::waitKey(0);
  }
}

TEST(ImpPyramidTest, testCopyImagePyramidCPU)
{
  using namespace ze;

  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  std::shared_ptr<ImageCv8uC1> img;
  cvBridgeLoad(img, joinPath(data_path, "img", "1.png"), PixelOrder::gray);

  ImagePyramid8uC1::Ptr pyr = createImagePyramidCpu<Pixel8uC1>(img, 0.5, 5, 8);
  for (size_t lvl = 0; lvl < pyr->numLevels(); ++lvl)
  {
    ImageCv8uC1 ocv_img(pyr->at(lvl));
    cv::GaussianBlur(ocv_img.cvMat(), ocv_img.cvMat(),
                     cv::Size(7, 7), 2, 2,
                     cv::BORDER_REFLECT_101);
    pyr->at(lvl).copyFrom(ocv_img);
  }
}

ZE_UNITTEST_ENTRYPOINT
