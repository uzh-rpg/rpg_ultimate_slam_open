#include <gtest/gtest.h>

#include <imp/core/image.hpp>
#include <imp/core/image_raw.hpp>
#include <ze/common/random.hpp>

namespace ze {
ImageRaw8uC1 generateRandomImage(size_t width, size_t height)
{
  auto random_val = uniformDistribution<uint8_t>(ZE_DETERMINISTIC);
  ImageRaw8uC1 img(width, height);
  for (size_t y = 0; y < img.height(); ++y)
  {
    for (size_t x = 0; x < img.width(); ++x)
    {
      uint8_t random_value = random_val();
      img[y][x] = random_value;
    }
  }
  return img;
}

} // ze namespace

TEST(IMPCoreTestSuite, testCopy8uC1)
{
  using namespace ze;
  ImageRaw8uC1 img = generateRandomImage(1024, 768);
  ImageRaw8uC1 img_copy(img.width(), img.height());
  img_copy.copyFrom(img);
  for (uint32_t x = 0; x < img_copy.width(); ++x)
  {
    for (uint32_t y = 0; y < img_copy.height(); ++y)
    {
      EXPECT_EQ(img(x, y), img_copy(x, y));
    }
  }
}

TEST(IMPCoreTestSuite, testCopy8uC1DifferentBytes)
{
  using namespace ze;

  constexpr uint32_t width{980};
  constexpr uint32_t padded_width{1024};
  constexpr uint32_t height{768};
  constexpr uint32_t padded_numel{padded_width * height};

  //! Allocate pitched memory and use external data pointer
  auto random_val = uniformDistribution<uint8_t>(ZE_DETERMINISTIC);
  Pixel8uC1 data_array[padded_numel];
  for (uint32_t i = 0; i < padded_numel; ++i)
  {
    data_array[i] = random_val();
  }
  ImageRaw8uC1 img(
        data_array,
        width, height,
        padded_width * sizeof(Pixel8uC1),
        true);

  //! Perform copy and test result
  ImageRaw8uC1 img_copy(img.width(), img.height());
  img_copy.copyFrom(img);
  for (uint32_t x = 0; x < img_copy.width(); ++x)
  {
    for (uint32_t y = 0; y < img_copy.height(); ++y)
    {
      EXPECT_EQ(img(x, y), img_copy(x, y));
    }
  }
}
