#include <gtest/gtest.h>

// system includes
#include <assert.h>
#include <cstdint>
#include <iostream>

#include <imp/core/size.hpp>

TEST(IMPCoreTestSuite,testSize)
{
  {
    ze::Size1u sz1u;
    EXPECT_EQ(0u, sz1u.length());
    uint32_t len=101;
    ze::Size1u sz(len);
    EXPECT_EQ(len, sz.length());
    EXPECT_EQ(len, sz.data()[0]);
    EXPECT_EQ(len, sz);
  }
  {
    ze::Size2u sz2u;
    EXPECT_EQ(0u, sz2u.width());
    EXPECT_EQ(0u, sz2u.height());

    // 2D sizes
    const std::int32_t w=10, h=13;
    const std::int32_t area = w*h;

    ze::Size2i sz(w,h);
    EXPECT_EQ(w, sz.width());
    EXPECT_EQ(h, sz.height());

    EXPECT_EQ(w, sz.data()[0]);
    EXPECT_EQ(h, sz.data()[1]);

    EXPECT_EQ(area, sz.area());
    EXPECT_EQ(area, sz.prod());

    // comparison operator tests
    ze::Size2i a(123,456);
    ze::Size2i b(123,456);
    ze::Size2i c(124,456);
    ze::Size2i d(124,457);

    ASSERT_TRUE((a == b));
    ASSERT_FALSE((a != b));
    ASSERT_FALSE((a == c));
    ASSERT_TRUE((a != c));
  }
}
