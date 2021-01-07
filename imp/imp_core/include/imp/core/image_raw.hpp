#pragma once

#include <memory>
#include <algorithm>

#include <ze/common/types.hpp>
#include <ze/common/macros.hpp>
#include <imp/core/image.hpp>
#include <imp/core/memory_storage.hpp>

namespace ze {

/**
 * @brief The ImageRaw class is an image (surprise) holding raw memory
 *
 * The ImageRaw memory can be used to allocate raw memory of a given size, or
 * take external memory and hold a reference to that. Note that when external
 * memory is given to the class, the memory won't be managed! You have to take
 * care about the memory deletion. Instead when you let the class itself allocate
 * the memory it will take care of freeing the memory again. In addition the allocation
 * takes care about memory address alignment (default: 32-byte) for the beginning of
 * every row.
 *
 * The template parameters are as follows:
 *   - Pixel: The pixel's memory representation (e.g. imp::Pixel8uC1 for single-channel unsigned 8-bit images)
 *   - pixel_type: The internal enum for specifying the pixel's type more specificly
 */
template<typename Pixel>
class ImageRaw : public ze::Image<Pixel>
{
public:
  ZE_POINTER_TYPEDEFS(ImageRaw);

  using Base = Image<Pixel>;
  using Memory = ze::MemoryStorage<Pixel>;
  using Deallocator = ze::MemoryDeallocator<Pixel>;

public:
  ImageRaw() = default;
  virtual ~ImageRaw() = default;

  /**
   * @brief ImageRaw construcs an image of given \a size
   */
  ImageRaw(const ze::Size2u& size,
           PixelOrder pixel_order = ze::PixelOrder::undefined);

  /**
   * @brief ImageRaw construcs an image of given size \a width x \a height
   */
  ImageRaw(uint32_t width, uint32_t height,
           PixelOrder pixel_order = ze::PixelOrder::undefined)
    : ImageRaw({width, height}, pixel_order)
  {
  }

  /**
   * @brief ImageRaw copy constructs an image from the given image \a from
   */
  ImageRaw(const ImageRaw& from);

  /**
   * @brief ImageRaw copy constructs an arbitrary base image \a from (not necessarily am \a ImageRaw)
   */
  ImageRaw(const Base& from);

  /**
   * @brief ImageRaw constructs an image with the given data (copied or refererenced!)
   * @param data Pointer to the image data.
   * @param width Image width.
   * @param height Image height.
   * @param pitch Length of a row in bytes (including padding).
   * @param use_ext_data_pointer Flag controlling whether the image should be copied (false) or if the data is just referenced (true)
   */
  ImageRaw(Pixel* data, uint32_t width, uint32_t height,
           uint32_t pitch, bool use_ext_data_pointer = false,
           PixelOrder pixel_order = ze::PixelOrder::undefined);

  /**
   * @brief ImageRaw constructs an image with the given data shared with the given tracked object
   * @param data Pointer to the image data.
   * @param width Image width.
   * @param height Image height.
   * @param pitch Length of a row in bytes (including padding).
   * @param tracked Tracked object that shares the given image data
   * @note we assume that the tracked object takes care about memory deallocations
   */
  ImageRaw(Pixel* data, uint32_t width, uint32_t height,
           uint32_t pitch, const std::shared_ptr<void const>& tracked,
           PixelOrder pixel_order = ze::PixelOrder::undefined);


  /** Returns a pointer to the pixel data.
   * The pointer can be offset to position \a (ox/oy).
   * @param[in] ox Horizontal/Column offset of the pointer array.
   * @param[in] oy Vertical/Row offset of the pointer array.
   * @return Pointer to the pixel array.
   */
  virtual Pixel* data(uint32_t ox = 0, uint32_t oy = 0) override;
  virtual const Pixel* data(uint32_t ox = 0, uint32_t oy = 0) const override;

protected:
  std::unique_ptr<Pixel, Deallocator> data_; //!< the actual image data
  std::shared_ptr<void const> tracked_ = nullptr; //!< tracked object to share memory
};

//-----------------------------------------------------------------------------
// convenience typedefs
// (sync with explicit template class instantiations at the end of the cpp file)
typedef ImageRaw<ze::Pixel8uC1> ImageRaw8uC1;
typedef ImageRaw<ze::Pixel8uC2> ImageRaw8uC2;
typedef ImageRaw<ze::Pixel8uC3> ImageRaw8uC3;
typedef ImageRaw<ze::Pixel8uC4> ImageRaw8uC4;

typedef ImageRaw<ze::Pixel16sC1> ImageRaw16sC1;
typedef ImageRaw<ze::Pixel16sC2> ImageRaw16sC2;
typedef ImageRaw<ze::Pixel16sC3> ImageRaw16sC3;
typedef ImageRaw<ze::Pixel16sC4> ImageRaw16sC4;

typedef ImageRaw<ze::Pixel16uC1> ImageRaw16uC1;
typedef ImageRaw<ze::Pixel16uC2> ImageRaw16uC2;
typedef ImageRaw<ze::Pixel16uC3> ImageRaw16uC3;
typedef ImageRaw<ze::Pixel16uC4> ImageRaw16uC4;

typedef ImageRaw<ze::Pixel32sC1> ImageRaw32sC1;
typedef ImageRaw<ze::Pixel32sC2> ImageRaw32sC2;
typedef ImageRaw<ze::Pixel32sC3> ImageRaw32sC3;
typedef ImageRaw<ze::Pixel32sC4> ImageRaw32sC4;

typedef ImageRaw<ze::Pixel32fC1> ImageRaw32fC1;
typedef ImageRaw<ze::Pixel32fC2> ImageRaw32fC2;
typedef ImageRaw<ze::Pixel32fC3> ImageRaw32fC3;
typedef ImageRaw<ze::Pixel32fC4> ImageRaw32fC4;

// shared pointers
template <typename Pixel>
using ImageRawPtr = typename ImageRaw<Pixel>::Ptr;

template <typename Pixel>
using ImageRawConstPtr = typename ImageRaw<Pixel>::ConstPtr;

} // namespace ze

