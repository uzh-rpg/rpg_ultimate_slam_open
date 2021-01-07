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

#include <string>
#include <vector>

#include <ze/common/string_utils.hpp>
#include <ze/common/test_entrypoint.hpp>


TEST(StringUtilsTest, testTrim)
{
  std::string a = " abc ";
  a = ze::trimString(a);
  EXPECT_EQ(a, "abc");
}

TEST(StringUtilsTest, testSplit)
{
  std::string a = "one,two,three";
  std::vector<std::string> vec = ze::splitString(a, ',');
  EXPECT_EQ(vec.size(), 3u);
  EXPECT_EQ(vec[0], "one");
  EXPECT_EQ(vec[1], "two");
  EXPECT_EQ(vec[2], "three");
}

TEST(StringUtilsTest, testEnsureSlash)
{
  std::string a = "a";
  a = ze::ensureLeftSlash(a);
  EXPECT_EQ(a, "/a");

  std::string b = "/b";
  b = ze::ensureLeftSlash(b);
  EXPECT_EQ(b, "/b");
}

ZE_UNITTEST_ENTRYPOINT
