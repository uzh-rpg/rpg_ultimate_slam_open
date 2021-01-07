#pragma once

#include <memory>

#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>


namespace ze {
namespace cu {

//------------------------------------------------------------------------------
template<typename Pixel>
void cvBridgeLoad(ze::cu::ImageGpuPtr<Pixel>& out, const std::string& filename,
                  ze::PixelOrder pixel_order)
{
  ImageCvPtr<Pixel> cv_img;
  ze::cvBridgeLoad<Pixel>(cv_img, filename, pixel_order);
  out = std::make_shared<ze::cu::ImageGpu<Pixel>>(*cv_img);
}

//------------------------------------------------------------------------------
template<typename Pixel>
void cvBridgeShow(const std::string& winname,
                  const ze::cu::ImageGpu<Pixel>& img, bool normalize=false)
{
  const ImageCv<Pixel> cv_img(img);
  ze::cvBridgeShow(winname, cv_img, normalize);
}

//------------------------------------------------------------------------------
template<typename Pixel, typename T>
void cvBridgeShow(const std::string& winname,
                  const ze::cu::ImageGpu<Pixel>& img,
                  const T& min, const T& max)
{
  const ImageCv<Pixel> cv_img(img);
  ze::cvBridgeShow(winname, cv_img, min, max);
}

} // namespace cu
} // namespace ze
