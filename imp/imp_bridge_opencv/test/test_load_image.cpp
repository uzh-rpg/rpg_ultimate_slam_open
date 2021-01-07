#include <functional>

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <imp/bridge/opencv/cv_bridge.hpp>

TEST(ImpBridgeOpenCvTests, testLoad)
{
  using namespace ze;

  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  ImageCv8uC1::Ptr img = cvBridgeLoad8uC1(joinPath(data_path, "img", "1.png"));
  CHECK(img);
}

ZE_UNITTEST_ENTRYPOINT
