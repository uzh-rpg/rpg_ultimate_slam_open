#include <imp/bridge/opencv/cv_connector_pixel_types.hpp>

#include <opencv2/core/core.hpp>

namespace ze {

//------------------------------------------------------------------------------
ze::PixelType pixelTypeFromCv(int type)
{
  switch (type)
  {
  case CV_8UC1: return ze::PixelType::i8uC1;
  case CV_8UC2: return ze::PixelType::i8uC2;
  case CV_8UC3: return ze::PixelType::i8uC3;
  case CV_8UC4: return ze::PixelType::i8uC4;
  //
  case CV_16UC1: return ze::PixelType::i16uC1;
  case CV_16UC2: return ze::PixelType::i16uC2;
  case CV_16UC3: return ze::PixelType::i16uC3;
  case CV_16UC4: return ze::PixelType::i16uC4;
  //
  case CV_32SC1: return ze::PixelType::i32sC1;
  case CV_32SC2: return ze::PixelType::i32sC2;
  case CV_32SC3: return ze::PixelType::i32sC3;
  case CV_32SC4: return ze::PixelType::i32sC4; //
  case CV_32FC1: return ze::PixelType::i32fC1;
  case CV_32FC2: return ze::PixelType::i32fC2;
  case CV_32FC3: return ze::PixelType::i32fC3;
  case CV_32FC4: return ze::PixelType::i32fC4;
  //
  default: return ze::PixelType::undefined;
  }
}

//------------------------------------------------------------------------------
int pixelTypeToCv(ze::PixelType type)
{
  switch (type)
  {
  case ze::PixelType::i8uC1: return CV_8UC1;
  case ze::PixelType::i8uC2: return CV_8UC2;
  case ze::PixelType::i8uC3: return CV_8UC3;
  case ze::PixelType::i8uC4: return CV_8UC4;
  //
  case ze::PixelType::i16uC1: return CV_16UC1;
  case ze::PixelType::i16uC2: return CV_16UC2;
  case ze::PixelType::i16uC3: return CV_16UC3;
  case ze::PixelType::i16uC4: return CV_16UC4;
  //
  case ze::PixelType::i32sC1: return CV_32SC1;
  case ze::PixelType::i32sC2: return CV_32SC2;
  case ze::PixelType::i32sC3: return CV_32SC3;
  case ze::PixelType::i32sC4: return CV_32SC4;
  //
  case ze::PixelType::i32fC1: return CV_32FC1;
  case ze::PixelType::i32fC2: return CV_32FC2;
  case ze::PixelType::i32fC3: return CV_32FC3;
  case ze::PixelType::i32fC4: return CV_32FC4;
  //
  default: return 0;
  }
}

} // namespace ze
