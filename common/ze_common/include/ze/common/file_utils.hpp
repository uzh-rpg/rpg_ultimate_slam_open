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

#include <fstream>
#include <iostream>
#include <ze/common/path_utils.hpp>
#include <ze/common/string_utils.hpp>

//! @file file_utils.hpp
//! Utility to open read or write filestreams and do the appropriate checks.

namespace ze {

inline void openFileStream(
    const std::string& filename,
    std::ifstream* fs)
{
  CHECK_NOTNULL(fs);
  CHECK(fileExists(filename)) << "File does not exist: " << filename;
  fs->open(filename.c_str(), std::ios::in);
  CHECK(*fs);
  CHECK(fs->is_open()) << "Failed to open file: " << filename;
  CHECK(!fs->eof()) << "File seems to contain no content!";
}

inline void openFileStreamAndCheckHeader(
    const std::string& filename,
    const std::string& header,
    std::ifstream* fs)
{
  openFileStream(filename, fs);
  std::string line;
  std::getline(*fs, line);
  CHECK_EQ(line, header) << "Invalid header.";
}

inline void openOutputFileStream(
    const std::string& filename,
    std::ofstream* fs)
{
  CHECK_NOTNULL(fs);
  fs->open(filename.c_str(), std::ios::out);
  CHECK(*fs) << "Failed to open filestream " << filename;
  CHECK(fs->is_open()) << "Failed to open file: " << filename;
}

} // namespace ze
