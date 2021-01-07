#include <gtest/gtest.h>

// system includes
#include <assert.h>
#include <cstdint>
#include <iostream>

#include <imp/core/pixel.hpp>

template <typename T>
void testNormalize()
{
  T tmp(1);
  T vec = tmp * 2;
  auto normalized = ze::normalize(vec);
  ASSERT_FLOAT_EQ(1.f, ze::length(normalized));
}

TEST(IMPCoreTestSuite,vecTest)
{
  testNormalize<ze::Vec8uC2>();
  testNormalize<ze::Vec8uC3>();
  testNormalize<ze::Vec8uC4>();

  testNormalize<ze::Vec16uC2>();
  testNormalize<ze::Vec16uC3>();
  testNormalize<ze::Vec16uC4>();

  testNormalize<ze::Vec32sC2>();
  testNormalize<ze::Vec32sC3>();
  testNormalize<ze::Vec32sC4>();

  testNormalize<ze::Vec32fC2>();
  testNormalize<ze::Vec32fC3>();
  testNormalize<ze::Vec32fC4>();
}
