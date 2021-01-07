#include <imp/bridge/opencv/image_cv.hpp>

#include <iostream>
#include <imp/core/memory_storage.hpp>
#include <imp/bridge/opencv/cv_connector_pixel_types.hpp>

namespace ze {

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageCv<Pixel>::ImageCv(const ze::Size2u& size, ze::PixelOrder pixel_order)
  : Base(size, pixel_order)
  , mat_(size[1], size[0], ze::pixelTypeToCv(pixel_type<Pixel>::type))
{
  this->header_.pitch = mat_.step;
  this->header_.memory_type = (MemoryStorage<Pixel>::isAligned(data())) ?
        MemoryType::CpuAligned : MemoryType::Cpu;}


//-----------------------------------------------------------------------------
template<typename Pixel>
ImageCv<Pixel>::ImageCv(uint32_t width, uint32_t height,
                        ze::PixelOrder pixel_order)
  : ImageCv(ze::Size2u(width, height), pixel_order)
{
}


//-----------------------------------------------------------------------------
template<typename Pixel>
ImageCv<Pixel>::ImageCv(const ImageCv<Pixel>& from)
  : Base(from)
  , mat_(from.cvMat())
{
  this->header_.pitch = mat_.step;
  this->header_.memory_type = (MemoryStorage<Pixel>::isAligned(data())) ?
        MemoryType::CpuAligned : MemoryType::Cpu;
}

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageCv<Pixel>::ImageCv(const Image<Pixel>& from)
  : Base(from)
  , mat_(from.height(), from.width(), ze::pixelTypeToCv(pixel_type<Pixel>::type))
{
  this->header_.pitch = mat_.step;
  this->header_.memory_type = (MemoryStorage<Pixel>::isAligned(data())) ?
        MemoryType::CpuAligned : MemoryType::Cpu;
  from.copyTo(*this);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageCv<Pixel>::ImageCv(cv::Mat mat, ze::PixelOrder pixel_order)
  : Base(ze::Size2u(mat.cols, mat.rows), pixel_order)
  , mat_(mat)
{
  this->header_.pitch = mat_.step;
  this->header_.memory_type = (MemoryStorage<Pixel>::isAligned(data())) ?
        MemoryType::CpuAligned : MemoryType::Cpu;

  CHECK(this->pixelType() == ze::pixelTypeFromCv(mat_.type()))
      << "OpenCV pixel type does not match to the internally used one.";

  if (this->pixelOrder() == ze::PixelOrder::undefined)
  {
    switch (this->pixelType())
    {
    case ze::PixelType::i8uC1:
    case ze::PixelType::i16uC1:
    case ze::PixelType::i32fC1:
    case ze::PixelType::i32sC1:
      this->header_.pixel_order = ze::PixelOrder::gray;
      break;
    case ze::PixelType::i8uC3:
    case ze::PixelType::i16uC3:
    case ze::PixelType::i32fC3:
    case ze::PixelType::i32sC3:
      this->header_.pixel_order = ze::PixelOrder::bgr;
      break;
    case ze::PixelType::i8uC4:
    case ze::PixelType::i16uC4:
    case ze::PixelType::i32fC4:
    case ze::PixelType::i32sC4:
      this->header_.pixel_order = ze::PixelOrder::bgra;
      break;
    default:
      // if we have something else than 1,3 or 4-channel images, we do not set the
      // pixel order automatically.
      VLOG(100) << "Undefined default order for given pixel type. "
                << "Only 1, 3 and 4 channel images have a default order.";
      break;
    }
  }
}


////-----------------------------------------------------------------------------
//template<typename Pixel, imp::PixelType pixel_type>
//ImageCv<Pixel>
//::ImageCv(Pixel* data, uint32_t width, uint32_t height,
//           uint32_t pitch, bool use_ext_data_pointer)
//  : Base(width, height)
//{
//  if (data == nullptr)
//  {
//    throw imp::Exception("input data not valid", __FILE__, __FUNCTION__, __LINE__);
//  }

//  if(use_ext_data_pointer)
//  {
//    // This uses the external data pointer as internal data pointer.
//    auto dealloc_nop = [](Pixel* p) { ; };
//    data_ = std::unique_ptr<pixel_storage_t, Deallocator>(
//          data, Deallocator(dealloc_nop));
//    pitch_ = pitch;
//  }
//  else
//  {
//    data_.reset(MemoryStorage<Pixel>::alignedAlloc(this->width(), this->height(), &pitch_));
//    size_t stride = pitch / sizeof(pixel_storage_t);

//    if (this->bytes() == pitch*height)
//    {
//      std::copy(data, data+stride*height, data_.get());
//    }
//    else
//    {
//      for (uint32_t y=0; y<height; ++y)
//      {
//        for (uint32_t x=0; x<width; ++x)
//        {
//          data_.get()[y*this->stride()+x] = data[y*stride + x];
//        }
//      }
//    }
//  }
//}

//-----------------------------------------------------------------------------
template<typename Pixel>
cv::Mat& ImageCv<Pixel>::cvMat()
{
  return mat_;
}

//-----------------------------------------------------------------------------
template<typename Pixel>
const cv::Mat& ImageCv<Pixel>::cvMat() const
{
  return mat_;
}

//-----------------------------------------------------------------------------
template<typename Pixel>
Pixel* ImageCv<Pixel>::data(uint32_t ox, uint32_t oy)
{
  CHECK_LT(ox, this->width());
  CHECK_LT(oy, this->height());

  Pixel* buffer = (Pixel*)mat_.data;
  return &buffer[oy*this->stride() + ox];
}

//-----------------------------------------------------------------------------
template<typename Pixel>
const Pixel* ImageCv<Pixel>::data(uint32_t ox, uint32_t oy) const
{
  CHECK_LT(ox, this->width());
  CHECK_LT(oy, this->height());

  Pixel* buffer = (Pixel*)mat_.data;
  return reinterpret_cast<const Pixel*>(&buffer[oy*this->stride() + ox]);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void ImageCv<Pixel>::setValue(const Pixel& value)
{
  mat_ = cv::Scalar::all(value);
}


//=============================================================================
// Explicitely instantiate the desired classes
// (sync with typedefs at the end of the hpp file)
template class ImageCv<ze::Pixel8uC1>;
template class ImageCv<ze::Pixel8uC2>;
template class ImageCv<ze::Pixel8uC3>;
template class ImageCv<ze::Pixel8uC4>;

template class ImageCv<ze::Pixel16uC1>;
template class ImageCv<ze::Pixel16uC2>;
template class ImageCv<ze::Pixel16uC3>;
template class ImageCv<ze::Pixel16uC4>;

template class ImageCv<ze::Pixel32sC1>;
template class ImageCv<ze::Pixel32sC2>;
template class ImageCv<ze::Pixel32sC3>;
template class ImageCv<ze::Pixel32sC4>;

template class ImageCv<ze::Pixel32fC1>;
template class ImageCv<ze::Pixel32fC2>;
template class ImageCv<ze::Pixel32fC3>;
template class ImageCv<ze::Pixel32fC4>;


} // namespace ze
