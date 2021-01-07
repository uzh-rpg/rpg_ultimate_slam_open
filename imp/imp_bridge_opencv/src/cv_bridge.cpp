#include <imp/bridge/opencv/cv_bridge.hpp>

namespace ze {

ImageCv8uC1::Ptr cvBridgeLoad8uC1(const std::string& filename)
{
  ImageCv8uC1::Ptr img;
  cvBridgeLoad<Pixel8uC1>(img, filename, PixelOrder::gray);
  return img;
}

} // namespace ze
