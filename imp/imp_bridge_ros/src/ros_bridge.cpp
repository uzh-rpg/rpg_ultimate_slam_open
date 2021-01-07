#include <imp/bridge/ros/ros_bridge.hpp>

#include <sensor_msgs/image_encodings.h>

#include <imp/core/image_raw.hpp>
#include <ze/common/logging.hpp>
#include <ze/common/types.hpp>

namespace ze {

namespace imgenc = sensor_msgs::image_encodings;

//------------------------------------------------------------------------------
std::pair<PixelType, PixelOrder> getPixelTypeFromRosImageEncoding(
    const std::string& encoding)
{
  //! @todo (MWE) we do not support bayer or YUV images yet.
  if (encoding == imgenc::BGR8)
  {
    return std::make_pair(PixelType::i8uC3, PixelOrder::bgr);
  }
  else if (encoding == imgenc::MONO8)
  {
    return std::make_pair(PixelType::i8uC1, PixelOrder::gray);
  }
  else if (encoding == imgenc::RGB8)
  {
    return std::make_pair(PixelType::i8uC3, PixelOrder::rgb);
  }
  else if (encoding == imgenc::MONO16)
  {
    return std::make_pair(PixelType::i16uC1, PixelOrder::gray);
  }
  else if (encoding == imgenc::BGR16)
  {
    return std::make_pair(PixelType::i16uC3, PixelOrder::bgr);
  }
  else if (encoding == imgenc::RGB16)
  {
    return std::make_pair(PixelType::i16uC3, PixelOrder::rgb);
  }
  else if (encoding == imgenc::BGRA8)
  {
    return std::make_pair(PixelType::i8uC4, PixelOrder::bgra);
  }
  else if (encoding == imgenc::RGBA8)
  {
    return std::make_pair(PixelType::i8uC4, PixelOrder::rgba);
  }
  else if (encoding == imgenc::BGRA16)
  {
    return std::make_pair(PixelType::i16uC4, PixelOrder::bgra);
  }
  else if (encoding == imgenc::RGBA16)
  {
    return std::make_pair(PixelType::i16uC4, PixelOrder::rgba);
  }
  LOG(FATAL) << "Unsupported image encoding " + encoding + ".";
  return std::make_pair(PixelType::undefined, PixelOrder::undefined);
}

//------------------------------------------------------------------------------
ImageBase::Ptr toImageCpu(
    const sensor_msgs::Image& src, PixelOrder /*pixel_order*/)
{
  PixelType src_pixel_type;
  PixelOrder src_pixel_order;
  std::tie(src_pixel_type, src_pixel_order) =
      getPixelTypeFromRosImageEncoding(src.encoding);

  int bit_depth = imgenc::bitDepth(src.encoding);
  int num_channels = imgenc::numChannels(src.encoding);
  uint32_t width = src.width;
  uint32_t height = src.height;
  uint32_t pitch = src.step;

  // sanity check
  CHECK_GE(pitch, width * num_channels * bit_depth/8) << "Input image seem to wrongly formatted";

  switch (src_pixel_type)
  {
    case PixelType::i8uC1:
    {
      ImageRaw8uC1 src_wrapped(
          reinterpret_cast<Pixel8uC1*>(const_cast<uint8_t*>(&src.data[0])),
          width, height, pitch, true, PixelOrder::gray);
      ImageRaw8uC1::Ptr dst =
          std::make_shared<ImageRaw8uC1>(src_wrapped); // Deep copy of the image data.
      return dst;
    }
//  case imp::PixelType::i8uC2:
//  { } break;
//  case imp::PixelType::i8uC3:
//  { } break;
//  case imp::PixelType::i8uC4:
//  { } break;
//  case imp::PixelType::i16uC1:
//  { } break;
//  case imp::PixelType::i16uC2:
//  { } break;
//  case imp::PixelType::i16uC3:
//  { } break;
//  case imp::PixelType::i16uC4:
//  { } break;
//  case imp::PixelType::i32uC1:
//  { } break;
//  case imp::PixelType::i32uC2:
//  { } break;
//  case imp::PixelType::i32uC3:
//  { } break;
//  case imp::PixelType::i32uC4:
//  { } break;
//  case imp::PixelType::i32sC1:
//  { } break;
//  case imp::PixelType::i32sC2:
//  { } break;
//  case imp::PixelType::i32sC3:
//  { } break;
//  case imp::PixelType::i32sC4:
//  { } break;
//  case imp::PixelType::i32fC1:
//  { } break;
//  case imp::PixelType::i32fC2:
//  { } break;
//  case imp::PixelType::i32fC3:
//  { } break;
//  case imp::PixelType::i32fC4:
//  { } break;
    default:
    {
      LOG(FATAL) << "Unsupported pixel type" + src.encoding + ".";
      break;
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
sensor_msgs::Image cvMatToImageMsg(const cv::Mat& image, const ros::Time& stamp, const std::string& encoding)
{
  sensor_msgs::Image ros_image;

  std_msgs::Header header;
  header.stamp = stamp;

  ros_image.header = header;
  ros_image.height = image.rows;
  ros_image.width = image.cols;
  ros_image.encoding = encoding;
  ros_image.is_bigendian = true; //@TODO: do this properly! WARNING!
  //ros_image.is_bigendian = (boost::endian::order::native == boost::endian::order::big);
  ros_image.step = image.cols * image.elemSize();
  size_t size = ros_image.step * image.rows;
  ros_image.data.resize(size);

  if (image.isContinuous())
  {
    memcpy((char*)(&ros_image.data[0]), image.data, size);
  }
  else
  {
    // Copy by row by row
    uchar* ros_data_ptr = (uchar*)(&ros_image.data[0]);
    uchar* cv_data_ptr = image.data;
    for (int i = 0; i < image.rows; ++i)
    {
      memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
      ros_data_ptr += ros_image.step;
      cv_data_ptr += image.step;
    }
  }
  return ros_image;
}

} // namespace ze
