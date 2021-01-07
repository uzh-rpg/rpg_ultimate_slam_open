#pragma once

namespace ze {

/**
 * @brief The PixelType enum defines a pixel's bit depth and number of channels.
 */
enum class PixelType
{
  undefined = -1,
  i8uC1,  //!< interleaved, 8-bit unsigned, 1 channel
  i8uC2,  //!< interleaved, 8-bit unsigned, 2 channel
  i8uC3,  //!< interleaved, 8-bit unsigned, 3 channel
  i8uC4,  //!< interleaved, 8-bit unsigned, 4 channel
  i16sC1, //!< interleaved, 16-bit signed, 1 channel
  i16sC2, //!< interleaved, 16-bit signed, 2 channel
  i16sC3, //!< interleaved, 16-bit signed, 3 channel
  i16sC4, //!< interleaved, 16-bit signed, 4 channel
  i16uC1, //!< interleaved, 16-bit unsigned, 1 channel
  i16uC2, //!< interleaved, 16-bit unsigned, 2 channel
  i16uC3, //!< interleaved, 16-bit unsigned, 3 channel
  i16uC4, //!< interleaved, 16-bit unsigned, 4 channel
  i32uC1, //!< interleaved, 32-bit unsigned, 1 channel
  i32uC2, //!< interleaved, 32-bit unsigned, 2 channel
  i32uC3, //!< interleaved, 32-bit unsigned, 3 channel
  i32uC4, //!< interleaved, 32-bit unsigned, 4 channel
  i32sC1, //!< interleaved, 32-bit signed, 1 channel
  i32sC2, //!< interleaved, 32-bit signed, 2 channel
  i32sC3, //!< interleaved, 32-bit signed, 3 channel
  i32sC4, //!< interleaved, 32-bit signed, 4 channel
  i32fC1, //!< interleaved, 32-bit float, 1 channel
  i32fC2, //!< interleaved, 32-bit float, 2 channel
  i32fC3, //!< interleaved, 32-bit float, 3 channel
  i32fC4  //!< interleaved, 32-bit float, 4 channel
};

/**
 * @brief The PixelOrder enum defines a pixel's channel ordering (for interleaved pixels).
 */
enum class PixelOrder
{
  undefined = -1,
  gray,  //!< single-channel grayscale
  rgb,   //!< 3-channel RGB
  bgr,   //!< 3-channel BGR
  rgba,  //!< 3-channel RGBA
  bgra   //!< 3-channel BGRA
};


} // namespace ze

