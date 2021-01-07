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

#include <ze/common/test_utils.hpp>

#include <cstdlib>
#include <ze/common/logging.hpp>
#include <ze/common/types.hpp>
#include <ze/common/path_utils.hpp>
#include <ze/common/string_utils.hpp>
#include <ze/common/file_utils.hpp>

namespace ze {

std::string getTestDataDir(const std::string& dataset_name)
{
  const char* datapath_dir = std::getenv("ZE_TEST_DATA_PATH");
  CHECK(datapath_dir != nullptr)
      << "Did you download the ze_test_data repository and set "
      << "the ZE_TEST_DATA_PATH environment variable?";

  std::string path(datapath_dir);
  CHECK(isDir(path)) << "Folder does not exist: " << path;
  path = path + "/data/" + dataset_name;
  CHECK(isDir(path)) << "Dataset does not exist: " << path;
  return path;
}

std::map<int64_t, Transformation> loadIndexedPosesFromCsv(const std::string& filename)
{
  std::map<int64_t, Transformation> poses;
  std::ifstream fs;
  openFileStream(filename, &fs);
  std::string line;
  while(std::getline(fs, line))
  {
    std::vector<std::string> items = splitString(line, ',');
    CHECK_EQ(items.size(), 8u);
    int64_t stamp = std::stoll(items[0]);
    Vector3 t(std::stod(items[1]), std::stod(items[2]), std::stod(items[3]));
    Quaternion q(std::stod(items[7]), std::stod(items[4]), std::stod(items[5]), std::stod(items[6]));
    poses[stamp] = Transformation(q, t);
  }
  return poses;
}

void loadDepthmapFromFile(
    const std::string& filename, const size_t data_size, float* data)
{
  CHECK_NOTNULL(data);
  std::ifstream fs;
  openFileStream(filename, &fs);
  float* data_ptr = data;
  for(size_t i = 0; i < data_size; ++i, ++data_ptr)
  {
    fs >> (*data_ptr);
    CHECK(fs.peek() != '\n' || i == data_size - 1)
        << "Did not read full depthmap. Num elements read = " <<  i
        << " of expected " << data_size;
  }
}

} // namespace ze
