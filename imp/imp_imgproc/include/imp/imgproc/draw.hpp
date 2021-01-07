#pragma once

#include <glog/logging.h>
#include <ze/common/types.hpp>
#include <imp/core/image.hpp>

namespace ze {

template <typename Pixel>
void drawLine(Image<Pixel>& im,
              real_t x1, real_t y1, real_t x2, real_t y2,
              const Pixel& color)
{
  real_t dx = x2-x1;
  real_t dy = y2-y1;
  int w = im.width();
  int h = im.height();
  real_t len = std::abs(dx) + std::abs(dy);
  for (int t = 0; t <= len; ++t)
  {
    int x = static_cast<int>(x1 + t/(len)*dx + 0.5);
    int y = static_cast<int>(y1 + t/(len)*dy + 0.5);
    if (x >=0 && x < w && y >= 0 && y < h)
    {
      im[y][x] = color;
    }
  }
}

template <typename Pixel>
void drawLine(Image<Pixel>& im,
              const Eigen::Ref<const Keypoint>& px1,
              const Eigen::Ref<const Keypoint>& px2,
              const Pixel& color)
{
  drawLine(im, px1(0), px1(1), px2(0), px2(1), color);
}

template <typename Pixel>
void drawLines(Image<Pixel>& im,
              const Keypoints& px1_vec, const Keypoints& px2_vec,
              const Pixel& color)
{
  CHECK_EQ(px1_vec.cols(), px2_vec.cols());
  for(int i = 0; i < px1_vec.cols(); ++i)
  {
    drawLine(im, px1_vec(0,i), px1_vec(1,i), px2_vec(0,i), px2_vec(1,i), color);
  }
}

template <typename Pixel>
void drawFeature(Image<Pixel>& im, real_t x, real_t y,
                 uint32_t halfwidth, const Pixel& color)
{
  int x1 = x - halfwidth;
  int y1 = y - halfwidth;
  int x2 = x + halfwidth;
  int y2 = y + halfwidth;
  int w = im.width();
  int h = im.height();
  if (x1 < 0 || x2 >= w || y1 < 0 || y2 >= h)
    return;

  for (int r = y1; r < y2; ++r)
  {
    Pixel* p = im[r];
    for (int c = x1; c < x2; ++c)
    {
      p[c] = color;
    }
  }
}

template <typename Pixel>
void drawFeature(Image<Pixel>& im,
                 const Eigen::Ref<const Keypoint>& px,
                 uint32_t halfwidth, const Pixel& color)
{
  drawFeature(im, px(0), px(1), halfwidth, color);
}

template <typename Pixel>
void drawFeatures(Image<Pixel>& im,
                 const Keypoints& px_vec,
                 uint32_t halfwidth, const Pixel& color)
{
  for(int i = 0; i < px_vec.cols(); ++i)
  {
    drawFeature(im, px_vec(0,i), px_vec(1,i), halfwidth, color);
  }
}

} // namespace ze
