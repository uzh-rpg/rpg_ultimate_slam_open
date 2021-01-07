#include <imp/core/image_raw.hpp>

#include <iostream>
#include <ze/common/logging.hpp>

namespace ze {

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageRaw<Pixel>::ImageRaw(const ze::Size2u& size, PixelOrder pixel_order)
  : Base(size, pixel_order)
{
  data_.reset(Memory::alignedAlloc(size, &this->header_.pitch));
  this->header_.memory_type = MemoryType::CpuAligned;
}

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageRaw<Pixel>::ImageRaw(const ImageRaw& from)
  : Base(from)
{
  data_.reset(Memory::alignedAlloc(this->size(), &this->header_.pitch));
  this->header_.memory_type = MemoryType::CpuAligned;
  from.copyTo(*this);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageRaw<Pixel>::ImageRaw(const Image<Pixel>& from)
  : Base(from)
{
  data_.reset(Memory::alignedAlloc(this->size(), &this->header_.pitch));
  this->header_.memory_type = MemoryType::CpuAligned;
  from.copyTo(*this);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageRaw<Pixel>
::ImageRaw(Pixel* data, uint32_t width, uint32_t height,
           uint32_t pitch, bool use_ext_data_pointer, PixelOrder pixel_order)
  : Base(width, height, pixel_order)
{
  CHECK(data);

  if(use_ext_data_pointer)
  {
    // This uses the external data pointer as internal data pointer.
    auto dealloc_nop = [](Pixel*) { ; };
    data_ = std::unique_ptr<Pixel, Deallocator>(data, Deallocator(dealloc_nop));
    this->header_.pitch = pitch;
    this->header_.memory_type = (Memory::isAligned(data)) ?
                                   MemoryType::CpuAligned : MemoryType::Cpu;
  }
  else
  {
    data_.reset(Memory::alignedAlloc(this->size(), &this->header_.pitch));
    this->header_.memory_type = MemoryType::CpuAligned;

    if (this->bytes() == pitch*height)
    {
      std::copy(data, data+this->stride()*height, data_.get());
    }
    else
    {
      for (uint32_t y=0; y<height; ++y)
      {
        for (uint32_t x=0; x<width; ++x)
        {
          data_.get()[y*this->stride()+x] = data[y*this->stride() + x];
        }
      }
    }
  }
}

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageRaw<Pixel>::ImageRaw(Pixel* data,
                          uint32_t width, uint32_t height,
                          uint32_t pitch,
                          const std::shared_ptr<void const>& tracked,
                          PixelOrder pixel_order)
  : Base(width, height, pixel_order)
{
  CHECK(data);
  CHECK(tracked);

  auto dealloc_nop = [](Pixel*) { ; };
  data_ = std::unique_ptr<Pixel, Deallocator>(data, Deallocator(dealloc_nop));
  this->header_.pitch = pitch;
  tracked_ = tracked;
  this->header_.memory_type = (Memory::isAligned(data)) ?
                                 MemoryType::CpuAligned : MemoryType::Cpu;
}

//-----------------------------------------------------------------------------
template<typename Pixel>
Pixel* ImageRaw<Pixel>::data(uint32_t ox, uint32_t oy)
{
  CHECK_LT(ox, this->width());
  CHECK_LT(oy, this->height());
  return &data_.get()[oy*this->stride() + ox];
}

//-----------------------------------------------------------------------------
template<typename Pixel>
const Pixel* ImageRaw<Pixel>::data(uint32_t ox, uint32_t oy) const
{
  CHECK_LT(ox, this->width());
  CHECK_LT(oy, this->height());
  return reinterpret_cast<const Pixel*>(&data_.get()[oy*this->stride() + ox]);
}

//=============================================================================
// Explicitely instantiate the desired classes
// (sync with typedefs at the end of the hpp file)
template class ImageRaw<ze::Pixel8uC1>;
template class ImageRaw<ze::Pixel8uC2>;
template class ImageRaw<ze::Pixel8uC3>;
template class ImageRaw<ze::Pixel8uC4>;

template class ImageRaw<ze::Pixel16sC1>;
template class ImageRaw<ze::Pixel16sC2>;
template class ImageRaw<ze::Pixel16sC3>;
template class ImageRaw<ze::Pixel16sC4>;

template class ImageRaw<ze::Pixel16uC1>;
template class ImageRaw<ze::Pixel16uC2>;
template class ImageRaw<ze::Pixel16uC3>;
template class ImageRaw<ze::Pixel16uC4>;

template class ImageRaw<ze::Pixel32sC1>;
template class ImageRaw<ze::Pixel32sC2>;
template class ImageRaw<ze::Pixel32sC3>;
template class ImageRaw<ze::Pixel32sC4>;

template class ImageRaw<ze::Pixel32fC1>;
template class ImageRaw<ze::Pixel32fC2>;
template class ImageRaw<ze::Pixel32fC3>;
template class ImageRaw<ze::Pixel32fC4>;

} // namespace ze
