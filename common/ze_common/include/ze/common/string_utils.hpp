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

#include <sstream>
#include <string>
#include <algorithm>
#include <vector>

#include <ze/common/logging.hpp>

//! @file string_utilties.hpp
//! Various utilities to work with std::string.

namespace ze {

inline std::string& leftTrimString(std::string& s)
{
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
  return s;
}

inline std::string& rightTrimString(std::string& s)
{
  s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
  return s;
}

inline std::string& trimString(std::string& s)
{
  return leftTrimString(rightTrimString(s));
}

inline std::string& ensureLeftSlash(std::string& s)
{
  CHECK_GE(s.size(), 1u);
  if(s[0] != '/')
  {
    s.insert(0, "/");
  }
  return s;
}

inline std::string ensureLeftSlash(const std::string& s)
{
  CHECK_GE(s.size(), 1u);
  std::string s_copy = s;
  if(s_copy[0] != '/')
  {
    s_copy.insert(0, "/");
  }
  return s_copy;
}

inline std::string ensureNoLeftSlash(const std::string& s)
{
  CHECK_GE(s.size(), 1u);
  if(s[0] != '/')
  {
    return s;
  }
  std::string s_copy = s;
  s_copy.erase(0, 1);
  CHECK(s_copy[0] != '/');
  return s_copy;
}

inline std::string ensureRightSlash(const std::string& s)
{
  if(s.size() == 0)
  {
    return "/";
  }
  if(s[s.size()-1] == '/')
  {
    return s;
  }
  return s + "/";
}

inline std::vector<std::string> splitString(const std::string& s, char delim)
{
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> items;
  while (std::getline(ss, item, delim))
  {
    items.push_back(trimString(item));
  }
  return items;
}

inline std::vector<int> readIntsFromString(
    const std::string& s, char delim = ',')
{
  std::vector<std::string> items = splitString(s, delim);
  std::vector<int> values;
  for (const std::string s: items)
  {
    values.push_back(stoi(s));
  }
  return values;
}

inline bool replaceInString(
    std::string& str, const std::string& from, const std::string& to)
{
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
    {
      return false;
    }
    str.replace(start_pos, from.length(), to);
    return true;
}

} // namespace ze
