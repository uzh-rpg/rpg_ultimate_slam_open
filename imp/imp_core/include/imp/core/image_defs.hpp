#pragma once

#include <memory>
#include <imp/core/pixel_enums.hpp>

namespace ze {

template<typename Pixel, ze::PixelType pixel_type>
class ImageCv;

template<typename Pixel, ze::PixelType pixel_type>
using ImageCvPtr = typename std::shared_ptr<ImageCv<Pixel,pixel_type>>;

template<typename Pixel, ze::PixelType pixel_type>
using ConstImageCvPtrRef = const std::shared_ptr<ImageCv<Pixel,pixel_type>>&;



} // namespace ze

