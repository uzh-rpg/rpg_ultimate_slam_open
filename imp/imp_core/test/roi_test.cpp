#include <gtest/gtest.h>

// system includes
#include <assert.h>
#include <cstdint>
#include <iostream>

#include <imp/core/roi.hpp>


TEST(IMPCoreTestSuite,roiTest)
{
  //
  // 1D case uint
  //
  {
    uint32_t x=1, len=10;
    ze::Roi1u roi(x,len);
    ASSERT_TRUE(x == roi.x());
    ASSERT_TRUE(len == roi.length());

    ASSERT_TRUE(x == roi.lu()[0]);
    ASSERT_TRUE(len == roi.size()[0]);
  }

  //
  // 2D case
  //
  {
    std::int32_t x=1, y=2, w=10, h=13;
    ze::Roi2i roi(x,y,w,h);
    ASSERT_TRUE(x == roi.x());
    ASSERT_TRUE(y == roi.y());
    ASSERT_TRUE(w == roi.width());
    ASSERT_TRUE(h == roi.height());

    ASSERT_TRUE(x == roi.lu()[0]);
    ASSERT_TRUE(y == roi.lu()[1]);
    ASSERT_TRUE(w == roi.size()[0]);
    ASSERT_TRUE(h == roi.size()[1]);
  }

  // operator==, etc.
  {
    uint32_t x=1, y=2, w=10, h=13;
    ze::Roi2u roi1(x,y,w,h);
    ze::Roi2u roi2(x,y,w,h);
    ze::Roi2u roi3(x+1,y,w,h);

    ASSERT_TRUE(roi1 == roi2);
    ASSERT_FALSE(roi1 != roi2);
    ASSERT_FALSE(roi1 == roi3);
    ASSERT_TRUE(roi1 != roi3);
  }
}
