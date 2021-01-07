// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <gtest/gtest.h>
#include <gflags/gflags.h>
#include <ze/common/logging.hpp>
#include <eigen-checks/gtest.h>
#include <ze/common/types.hpp>

//! @file test_entrypoint.hpp
//! Macros for unit tests.

// Floating-point precision checks.
#ifdef ZE_SINGLE_PRECISION_FLOAT
# define EXPECT_FLOATTYPE_EQ(a, b) EXPECT_FLOAT_EQ(a, b)
#else
# define EXPECT_FLOATTYPE_EQ(a, b) EXPECT_DOUBLE_EQ(a, b)
#endif

// Let the Eclipse parser see the macro.
#ifndef TEST
# define TEST(a, b) int Test_##a##_##b()
#endif

#ifndef TEST_F
# define TEST_F(a, b) int Test_##a##_##b()
#endif

#ifndef TEST_P
# define TEST_P(a, b) int Test_##a##_##b()
#endif

#ifndef TYPED_TEST_CASE
# define TYPED_TEST_CASE(a, b) int Test_##a##_##b()
#endif

#ifndef TYPED_TEST
# define TYPED_TEST(a, b) int Test_##a##_##b()
#endif

#define ZE_UNITTEST_ENTRYPOINT\
  int main(int argc, char** argv) {\
  ::testing::InitGoogleTest(&argc, argv);\
  google::InitGoogleLogging(argv[0]);\
  google::ParseCommandLineFlags(&argc, &argv, false);\
  google::InstallFailureSignalHandler();\
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";\
  FLAGS_alsologtostderr = true; \
  FLAGS_colorlogtostderr = true; \
  return RUN_ALL_TESTS();\
}
