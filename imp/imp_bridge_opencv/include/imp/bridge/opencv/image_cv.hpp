#pragma once

#include <memory>
#include <opencv2/core/core.hpp>
#include <imp/core/pixel_enums.hpp>
#include <imp/core/image.hpp>

namespace ze {

/**
 * @brief The ImageCv class is an image holding an OpenCV matrix (cv::Mat) for the image data
 *
 * The ImageCv can be used to interface with OpenCV. The matrix can be directly
 * accessed and used for calling OpenCV functions. Furthermore all getters/setters
 * for the IMP image representations are available. You can also construct an
 * ImageCv with a given cv::Mat in order to have a common data representation in
 * your code.
 *
 */
template<typename Pixel>
class ImageCv : public ze::Image<Pixel>
{
public:
  using Base = Image<Pixel>;

  using Ptr = typename std::shared_ptr<ImageCv<Pixel>>;
  using ConstPtrRef = const Ptr&;
  using ConstPtr = typename std::shared_ptr<ImageCv<Pixel> const>;

public:
  ImageCv() = delete;
  virtual ~ImageCv() = default;

  ImageCv(const ze::Size2u& size, ze::PixelOrder pixel_order=ze::PixelOrder::undefined);
  ImageCv(uint32_t width, uint32_t height, ze::PixelOrder pixel_order=ze::PixelOrder::undefined);
  ImageCv(const ImageCv<Pixel>& from);
  ImageCv(const Base& from);
  ImageCv(cv::Mat mat, ze::PixelOrder pixel_order=ze::PixelOrder::undefined);
//  ImageCv(Pixel* data, uint32_t width, uint32_t height,
//          uint32_t pitch, bool use_ext_data_pointer = false);

  /** Returns the internal OpenCV image/mat
   */
  virtual cv::Mat& cvMat();
  virtual const cv::Mat& cvMat() const;

  /** Returns a pointer to the pixel data.
   * The pointer can be offset to position \a (ox/oy).
   * @param[in] ox Horizontal offset of the pointer array.
   * @param[in] oy Vertical offset of the pointer array.
   * @return Pointer to the pixel array.
   */
  virtual Pixel* data(uint32_t ox = 0, uint32_t oy = 0) override;
  virtual const Pixel* data(uint32_t ox = 0, uint32_t oy = 0) const override;

  /**
   * @brief setValue Sets image data to the specified \a value.
   * @param value Value to be set to the whole image data.
   */
  virtual void setValue(const Pixel& value) override;

protected:
  cv::Mat mat_;
};

//-----------------------------------------------------------------------------
// convenience typedefs
// (sync with explicit template class instantiations at the end of the cpp file)
typedef ImageCv<ze::Pixel8uC1> ImageCv8uC1;
typedef ImageCv<ze::Pixel8uC2> ImageCv8uC2;
typedef ImageCv<ze::Pixel8uC3> ImageCv8uC3;
typedef ImageCv<ze::Pixel8uC4> ImageCv8uC4;

typedef ImageCv<ze::Pixel16uC1> ImageCv16uC1;
typedef ImageCv<ze::Pixel16uC2> ImageCv16uC2;
typedef ImageCv<ze::Pixel16uC3> ImageCv16uC3;
typedef ImageCv<ze::Pixel16uC4> ImageCv16uC4;

typedef ImageCv<ze::Pixel32sC1> ImageCv32sC1;
typedef ImageCv<ze::Pixel32sC2> ImageCv32sC2;
typedef ImageCv<ze::Pixel32sC3> ImageCv32sC3;
typedef ImageCv<ze::Pixel32sC4> ImageCv32sC4;

typedef ImageCv<ze::Pixel32fC1> ImageCv32fC1;
typedef ImageCv<ze::Pixel32fC2> ImageCv32fC2;
typedef ImageCv<ze::Pixel32fC3> ImageCv32fC3;
typedef ImageCv<ze::Pixel32fC4> ImageCv32fC4;


//typedef ImageCv<std::uint8_t, imp::PixelType::i8uC1> ImageCv8uC1;
//typedef ImageCv<std::uint16_t, imp::PixelType::i8uC1> ImageCv16uC1;
//typedef ImageCv<sstd::int32_t, imp::PixelType::i8uC1> ImageCv32sC1;
//typedef ImageCv<float, imp::PixelType::i8uC1> ImageCv32fC1;

// shared pointers

template <typename Pixel>
using ImageCvPtr = typename std::shared_ptr<ImageCv<Pixel>>;

//template <typename Pixel>
//using ConstImageCvPtrRef = typename ImageCv<Pixel>::ConstPtrRef;


template <typename Pixel>
using ImageCvConstPtr = typename ImageCv<Pixel>::ConstPtr;


} // namespace ze

