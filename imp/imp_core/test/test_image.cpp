#include <gtest/gtest.h>

// system includes
#include <assert.h>
#include <cstdint>
#include <iostream>
#include <random>
#include <functional>
#include <type_traits>

#include <ze/common/random.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/types.hpp>

#include <imp/core/memory_storage.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/core/roi.hpp>
#include <imp/core/size.hpp>


DEFINE_bool(visualize, false, "Show input images and results");

template <typename Pixel>
class ImageRawTest : public ::testing::Test
{
 protected:
  ImageRawTest()
    : image_512_(size_512_)
    , image_511_(size_511_)
  {
    using T = typename Pixel::T;
    auto random_val_generator = ze::uniformDistribution<T>(ZE_DETERMINISTIC);

    // initialize two random value and ensure that they are reasonably different
    T val1 = random_val_generator();
    T val2;
    do
    {
      val2 = random_val_generator();
    } while(std::fabs(val1-val2) < std::numeric_limits<T>::epsilon());

    random_value1_ = Pixel(val1);
    random_value2_ = Pixel(val2);
  }

  void setValue()
  {
    image_512_.setValue(random_value1_);
    image_511_.setValue(random_value1_);
  }

  void setRoi()
  {
    image_512_.setRoi(roi_);
    image_511_.setRoi(roi_);
  }

  void setValueRoi()
  {
    this->setRoi();
    image_512_.setValue(random_value2_);
    image_511_.setValue(random_value2_);
  }

protected:
  uint8_t pixel_size_ = sizeof(Pixel);
  size_t pixel_bit_depth_ = 8*sizeof(Pixel);

  ze::Size2u size_512_{512u,512u};
  ze::Size2u size_511_{511u,512u};
  ze::Roi2u roi_{128,128,256,256};

  ze::ImageRaw<Pixel> image_512_;
  ze::ImageRaw<Pixel> image_511_;


  Pixel random_value1_;
  Pixel random_value2_;
};

// The list of types we want to test.
typedef testing::Types<
ze::Pixel8uC1, ze::Pixel8uC2, ze::Pixel8uC3, ze::Pixel8uC4,
ze::Pixel16uC1, ze::Pixel16uC2, ze::Pixel16uC3, ze::Pixel16uC4,
ze::Pixel16sC1, ze::Pixel16sC2, ze::Pixel16sC3, ze::Pixel16sC4,
ze::Pixel32sC1, ze::Pixel32sC2, ze::Pixel32sC3, ze::Pixel32sC4,
ze::Pixel32fC1, ze::Pixel32fC2, ze::Pixel32fC3, ze::Pixel32fC4> PixelTypes;

TYPED_TEST_CASE(ImageRawTest, PixelTypes);

TYPED_TEST(ImageRawTest, CheckMemforyAlignment)
{
  EXPECT_EQ(0u, (std::uintptr_t)reinterpret_cast<void*>(this->image_512_.data()) % 32);
//  EXPECT_TRUE(ze::MemoryStorage::isAligned(this->image_512_.data()));

  EXPECT_EQ(0u, (std::uintptr_t)reinterpret_cast<void*>(this->image_511_.data()) % 32);
//  EXPECT_TRUE(ze::MemoryStorage::isAligned(this->image_511_.data()));

  EXPECT_EQ(512u, this->image_512_.stride());
  EXPECT_EQ(512u, this->image_511_.stride());

  EXPECT_EQ(512u*this->pixel_size_, this->image_512_.pitch());
  EXPECT_EQ(512u*this->pixel_size_, this->image_511_.pitch());
}

TYPED_TEST(ImageRawTest, CheckSize)
{
  EXPECT_EQ(this->size_512_, this->image_512_.size());
  EXPECT_EQ(this->size_511_, this->image_511_.size());
}

TYPED_TEST(ImageRawTest, CheckNoRoi)
{
  EXPECT_EQ(ze::Roi2u(0,0,512,512), this->image_512_.roi());
  EXPECT_EQ(ze::Roi2u(this->size_511_), this->image_511_.roi());
}

TYPED_TEST(ImageRawTest, CheckRoi)
{
  this->setRoi();
  EXPECT_EQ(this->roi_, this->image_512_.roi());
  EXPECT_EQ(this->roi_, this->image_511_.roi());
}

TYPED_TEST(ImageRawTest, CheckBytes)
{
  EXPECT_EQ(512u*512u*this->pixel_size_, this->image_512_.bytes());
  EXPECT_EQ(512u*512u*this->pixel_size_, this->image_511_.bytes());

  EXPECT_EQ(this->size_512_[0]*this->pixel_size_, this->image_512_.rowBytes());
  EXPECT_EQ(this->size_511_[0]*this->pixel_size_, this->image_511_.rowBytes());
}

TYPED_TEST(ImageRawTest, CheckPixelBitDepth)
{
  EXPECT_EQ(this->pixel_bit_depth_, this->image_512_.bitDepth());
  EXPECT_EQ(this->pixel_bit_depth_, this->image_511_.bitDepth());
}

TYPED_TEST(ImageRawTest, ReturnsFalseForNonGpuMemory)
{
  ASSERT_FALSE(this->image_512_.isGpuMemory());
  ASSERT_FALSE(this->image_511_.isGpuMemory());
}

TYPED_TEST(ImageRawTest, CheckValues)
{
  this->setValue();
  for (ze::uint32_t y=0u; y<512; ++y)
  {
    for (ze::uint32_t x=0u; x<512; ++x)
    {
      EXPECT_EQ(this->image_512_(x,y), this->random_value1_);
      EXPECT_EQ(this->image_512_[y][x], this->random_value1_);
      EXPECT_EQ(*this->image_512_.data(x,y), this->random_value1_);

      if (x<511)
      {
        EXPECT_EQ(this->image_511_(x,y), this->random_value1_);
        EXPECT_EQ(this->image_511_[y][x], this->random_value1_);
        EXPECT_EQ(*this->image_511_.data(x,y), this->random_value1_);
      }
    }
  }
}

//-----------------------------------------------------------------------------
TYPED_TEST(ImageRawTest, CheckRoiValues)
{
  this->setValue();
  this->setValueRoi();

  for (ze::uint32_t y=0u; y<512; ++y)
  {
    for (ze::uint32_t x=0u; x<512; ++x)
    {
      if (x>=this->roi_.x() && x<(this->roi_.x()+this->roi_.width()) &&
          y>=this->roi_.y() && y<(this->roi_.y()+this->roi_.height()))
      {
        EXPECT_EQ(this->image_512_(x,y), this->random_value2_);
        EXPECT_EQ(this->image_512_[y][x], this->random_value2_);
        EXPECT_EQ(*this->image_512_.data(x,y), this->random_value2_);

        if (x<511)
        {
          EXPECT_EQ(this->image_511_(x,y), this->random_value2_);
          EXPECT_EQ(this->image_511_[y][x], this->random_value2_);
          EXPECT_EQ(*this->image_511_.data(x,y), this->random_value2_);
        }
      }
      else
      {
        EXPECT_EQ(this->image_512_(x,y), this->random_value1_);
        EXPECT_EQ(this->image_512_[y][x], this->random_value1_);
        EXPECT_EQ(*this->image_512_.data(x,y), this->random_value1_);

        if (x<511)
        {
          EXPECT_EQ(this->image_511_(x,y), this->random_value1_);
          EXPECT_EQ(this->image_511_[y][x], this->random_value1_);
          EXPECT_EQ(*this->image_511_.data(x,y), this->random_value1_);
        }
      }
    }
  }
}

ZE_UNITTEST_ENTRYPOINT
