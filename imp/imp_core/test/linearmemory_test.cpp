#include <gtest/gtest.h>

// system includes
#include <assert.h>
#include <cstdint>
#include <iostream>
#include <functional>
#include <limits>
#include <type_traits>

#include <imp/core/linearmemory.hpp>
#include <ze/common/random.hpp>
#include <ze/common/test_utils.hpp>


template <typename Pixel>
class LinearMemoryTest : public ::testing::Test
{
 protected:
  LinearMemoryTest() :
    linmem_(numel_)
  {
    using T = typename Pixel::T;
    auto random_val_generator = ze::uniformDistribution<T>(ZE_DETERMINISTIC);

    T val1 = random_val_generator();
    T val2;
    do
    {
      val2 = random_val_generator();
    } while(std::fabs(val1-val2) < std::numeric_limits<T>::epsilon());

    pixel1_ = Pixel(val1);
    pixel2_ = Pixel(val2);
  }

  void setValue()
  {
    linmem_.setValue(pixel1_);
  }

  void setRoi()
  {
    linmem_.setRoi(roi_);
  }

  void setValueRoi()
  {
    this->setRoi();
    linmem_.setValue(pixel2_);
  }

  uint8_t pixel_size_ = sizeof(Pixel);
  size_t pixel_bit_depth_ = 8*sizeof(Pixel);

  size_t numel_ = 123;
  ze::Roi1u roi_ = ze::Roi1u(numel_/3, numel_/3);
  ze::LinearMemory<Pixel> linmem_;

  Pixel pixel1_;
  Pixel pixel2_;
};

// The list of types we want to test.
typedef testing::Types<
ze::Pixel8uC1, ze::Pixel8uC2, ze::Pixel8uC3, ze::Pixel8uC4,
ze::Pixel16uC1, ze::Pixel16uC2, ze::Pixel16uC3, ze::Pixel16uC4,
ze::Pixel32sC1, ze::Pixel32sC2, ze::Pixel32sC3, ze::Pixel32sC4,
ze::Pixel32uC1, ze::Pixel32uC2, ze::Pixel32uC3, ze::Pixel32uC4,
ze::Pixel32fC1, ze::Pixel32fC2, ze::Pixel32fC3, ze::Pixel32fC4> PixelTypes;

TYPED_TEST_CASE(LinearMemoryTest, PixelTypes);

TYPED_TEST(LinearMemoryTest, CheckMemforyAlignment)
{
  EXPECT_EQ(0u, (std::uintptr_t)reinterpret_cast<void*>(this->linmem_.data()) % 32);
}

TYPED_TEST(LinearMemoryTest, CheckLength)
{
  EXPECT_EQ(this->numel_, this->linmem_.length());
}

TYPED_TEST(LinearMemoryTest, CheckNoRoi)
{
  EXPECT_EQ(0u, this->linmem_.roi().x());
  EXPECT_EQ(this->numel_, this->linmem_.roi().length());
}

TYPED_TEST(LinearMemoryTest, CheckRoi)
{
  this->setRoi();
  EXPECT_EQ(this->roi_.x(), this->linmem_.roi().x());
  EXPECT_EQ(this->roi_.length(), this->linmem_.roi().length());
}

TYPED_TEST(LinearMemoryTest, CheckNumBytes)
{
  EXPECT_EQ(this->numel_*this->pixel_size_, this->linmem_.bytes());
}

TYPED_TEST(LinearMemoryTest, CheckNumRoiBytes)
{
  this->setRoi();
  EXPECT_EQ(this->roi_.length()*this->pixel_size_, this->linmem_.roiBytes());
}

TYPED_TEST(LinearMemoryTest, CheckPixelBitDepth)
{
  EXPECT_EQ(this->pixel_bit_depth_, this->linmem_.bitDepth());
}

TYPED_TEST(LinearMemoryTest, ReturnsFalseForNonGpuMemory)
{
  ASSERT_FALSE(this->linmem_.isGpuMemory());
}

TYPED_TEST(LinearMemoryTest, CheckValues)
{
  this->setValue();
  for (uint32_t i=0u; i<this->numel_; ++i)
  {
    EXPECT_EQ(this->linmem_[i], this->pixel1_);
  }
}

TYPED_TEST(LinearMemoryTest, CheckValuesInConstLinearMemory)
{
  this->setValue();
  const ze::LinearMemory<TypeParam> const_linmem(this->linmem_);
  for (uint32_t i=0u; i<this->numel_; ++i)
  {
    EXPECT_EQ(const_linmem[i], this->pixel1_);
  }
}


TYPED_TEST(LinearMemoryTest, CheckRoiValues)
{
  this->setValue();
  this->setValueRoi();

  for (uint32_t i=0u; i<this->numel_; ++i)
  {
    if (i>=this->roi_.x() && i<(this->roi_.x()+this->roi_.length()))
    {
      EXPECT_EQ(this->pixel2_, this->linmem_[i]);
    }
    else
    {
      EXPECT_EQ(this->pixel1_, this->linmem_[i]);
    }
  }
}

