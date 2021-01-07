#include <functional>

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/imgproc/draw.hpp>

TEST(ImpDrawTest, testLine)
{
  using namespace ze;

  // Test drawing interface
  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  ImageCv8uC1::Ptr img = cvBridgeLoad8uC1(joinPath(data_path, "img", "1.png"));  
  Pixel8uC1 color = 255;
  Keypoint px1(30, 50), px2(200, 40);
  drawLine<Pixel8uC1>(*img, 100.0, 100.0, 200.0, 200.0, color);
  drawLine<Pixel8uC1>(*img, px1, px2, color);
  drawFeature<Pixel8uC1>(*img, 10.0, 10.0, 5, color);
  drawFeature<Pixel8uC1>(*img, px1, 5, color);
  drawFeature<Pixel8uC1>(*img, px2, 5, 100);

  if(false)
  {
    cvBridgeShow("img", ImageCv8uC1(*img));
    cv::waitKey(0);
  }
}

ZE_UNITTEST_ENTRYPOINT
