#pragma once

#include <memory>
#include <vector>

#include <ze/common/types.hpp>
#include <ze/common/macros.hpp>
#include <imp/core/image_header.hpp>
#include <imp/core/pixel_enums.hpp>
#include <imp/core/roi.hpp>
#include <imp/core/size.hpp>
#include <imp/core/types.hpp>

namespace ze {

/**
 * @brief The ImageBase class is the base class of all our image representations.
 *
 * It defines the common interface that must be implemented for IMP images
 *
 */
class ImageBase: public std::enable_shared_from_this<ImageBase>
{
public:
  ZE_POINTER_TYPEDEFS(ImageBase);

public:
  virtual ~ImageBase() = default;

  ImageBase& operator= (const ImageBase &from)
  {
    // TODO operator==
    this->header_ = from.header_;
    return *this;
  }

  inline void setRoi(const ze::Roi2u& roi) {header_.roi = roi;}

  /** Returns the element types. */
  PixelType pixelType() const
  {
    return header_.pixel_type;
  }

  /** Returns the pixel's channel order. */
  inline PixelOrder pixelOrder() const {return header_.pixel_order;}

  inline Size2u size() const {return header_.size;}

  inline Roi2u roi() const {return header_.roi;}

  /** Returns the distance in bytes between starts of consecutive rows. */
  inline uint32_t pitch() const {return header_.pitch;}

  inline uint32_t width() const {return header_.size[0];}

  inline uint32_t height() const {return header_.size[1];}

  inline std::uint8_t nChannels() const
  {
    switch (header_.pixel_type)
    {
    case PixelType::i8uC1:
    case PixelType::i16uC1:
    case PixelType::i32uC1:
    case PixelType::i32sC1:
    case PixelType::i32fC1:
      return 1;
    case PixelType::i8uC2:
    case PixelType::i16uC2:
    case PixelType::i32uC2:
    case PixelType::i32sC2:
    case PixelType::i32fC2:
      return 2;
    case PixelType::i8uC3:
    case PixelType::i16uC3:
    case PixelType::i32uC3:
    case PixelType::i32sC3:
    case PixelType::i32fC3:
      return 3;
    case PixelType::i8uC4:
    case PixelType::i16uC4:
    case PixelType::i32uC4:
    case PixelType::i32sC4:
    case PixelType::i32fC4:
      return 4;
    default:
      return 0;
    }
  }

  /** Returns the number of pixels in the image. */
  inline size_t numel() const {return (header_.size[0]*header_.size[1]);}

  /** Returns the total amount of bytes saved in the data buffer. */
  inline size_t bytes() const {return header_.size[1]*header_.pitch;}

  /** Returns the length of a row (not including the padding!) in bytes. */
  inline size_t rowBytes() const {return header_.size[0]*header_.pixel_size;}

  /** Returns the distance in pixels between starts of consecutive rows. */
  inline size_t stride() const {return header_.pitch/header_.pixel_size;}

  /** Returns the bit depth of the data pointer. */
  inline std::uint8_t bitDepth() const {return 8*header_.pixel_size;}

  /** Returns flag if the image data resides on the device/GPU (TRUE) or host/GPU (FALSE) */
  inline bool isGpuMemory() const {return header_.isGpuMemory();}


  /** Cast between image types */
  template<class DERIVED>
  inline typename DERIVED::Ptr as()
  {
    return std::dynamic_pointer_cast<DERIVED>(shared_from_this());
  }

  friend std::ostream& operator<<(std::ostream &os, const ImageBase& image);

protected:
  ImageBase() = delete;

  ImageBase(
      PixelType pixel_type,
      uint8_t pixel_size,
      PixelOrder pixel_order = ze::PixelOrder::undefined)
    : header_(pixel_type, pixel_size, pixel_order)
  {
  }

  ImageBase(
      PixelType pixel_type,
      uint8_t pixel_size,
      PixelOrder pixel_order,
      const Size2u &size)
    : header_(pixel_type, pixel_size, pixel_order, size)
  {
  }

  ImageBase(
      PixelType pixel_type,
      uint8_t pixel_size,
      PixelOrder pixel_order,
      uint32_t width,
      uint32_t height)
    : ImageBase(pixel_type, pixel_size, pixel_order, {width, height})
  {
  }

  ImageBase(const ImageBase &from)
    : std::enable_shared_from_this<ImageBase>()
    , header_(from.header_)
  {
  }

protected:
  ImageHeader header_;
};

inline std::ostream& operator<<(std::ostream &os, const ImageBase& image)
{
  os << "size: " << image.width() << "x" << image.height()
     << "; roi=(" << image.roi().x() << "," << image.roi().y()
     << "+" << image.roi().width() << "+" << image.roi().height() << ")"
     << "; stride: " << image.stride() << "; pitch: " << image.pitch()
     << "; bitDepth: " << (int)image.bitDepth();
  if (image.isGpuMemory())
    os << "; (gpu)";
  else
    os << "; (cpu)";

  return os;
}

// convenience typedefs
using ImageBasePtr = std::shared_ptr<ImageBase>;
using StampedImage = std::pair<int64_t, ImageBase::Ptr>;
using StampedImages = std::vector<StampedImage>;

} // namespace ze

