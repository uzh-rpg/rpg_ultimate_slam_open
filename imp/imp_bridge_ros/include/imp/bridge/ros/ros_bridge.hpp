#pragma once

#include <sensor_msgs/Image.h>
#include <imp/core/image.hpp>
#include <opencv2/core/core.hpp>

namespace ze {

std::pair<PixelType, PixelOrder> getPixelTypeFromRosImageEncoding(
    const std::string& encoding);

ImageBase::Ptr toImageCpu(
    const sensor_msgs::Image& src,
    PixelOrder pixel_order = PixelOrder::undefined);

ImageBase::Ptr toImageGpu(
    const sensor_msgs::Image& src,
    PixelOrder pixel_order = PixelOrder::undefined);

sensor_msgs::Image cvMatToImageMsg(const cv::Mat& image,
                                   const ros::Time& stamp,
                                   const std::string& encoding);

} // namespace ze
