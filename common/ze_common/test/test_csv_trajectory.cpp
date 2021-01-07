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

#include <ze/common/buffer.hpp>
#include <ze/common/csv_trajectory.hpp>
#include <ze/common/file_utils.hpp>
#include <ze/common/path_utils.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/time_conversions.hpp>
#include <ze/common/types.hpp>

using namespace ze;

TEST(TimeStampMatcher, matchTest)
{
  std::string test_data_dir = getTestDataDir("ze_ts_matching");
  constexpr real_t offset_sec = 0.0;
  constexpr real_t max_difference_sec = 0.02;

  // Load groundtruth.
  PoseSeries gt_poses;
  gt_poses.load(joinPath(test_data_dir, "traj_gt.csv"));

  // Load estimated trajectory.
  SWEResultSeries es_poses;
  es_poses.load(joinPath(test_data_dir, "traj_es.csv"));

  // Load matches to compare against
  std::map<int64_t, int64_t> matches;
  std::ifstream fs;
  openFileStream(joinPath(test_data_dir, "python_matches.csv"), &fs);
  std::string line;
  while (std::getline(fs, line))
  {
    if ('%' != line.at(0) && '#' != line.at(0))
    {
      std::vector<std::string> items = splitString(line, ',');
      EXPECT_GE(items.size(), 2u);
      int64_t stamp_es = std::stoll(items[0]);
      int64_t stamp_gt = std::stoll(items[1]);
      matches[stamp_es] = stamp_gt;
    }
  }

  // Now loop through all estimated poses and find closest groundtruth-stamp.
  auto& gt_buffer = gt_poses.getBuffer();
  auto es_poses_data = es_poses.getStampedTransformationVector();
  int n_skipped = 0, n_checked = 0;
  for (const std::pair<int64_t, Transformation>& stampd_pose : es_poses_data)
  {
    Vector7 matched_gt_pose;
    int64_t matched_gt_stamp;
    if (!findNearestTimeStamp(gt_buffer, stampd_pose.first, matched_gt_stamp,
                              matched_gt_pose, max_difference_sec, offset_sec))
    {
      ++n_skipped;
    }
    else
    {
      EXPECT_LE(std::abs(stampd_pose.first - matched_gt_stamp),
                secToNanosec(max_difference_sec));
      EXPECT_EQ(matches.find(stampd_pose.first)->second, matched_gt_stamp);
      ++n_checked;
    }
  }
  EXPECT_EQ(es_poses_data.size(), n_checked+n_skipped);
}

ZE_UNITTEST_ENTRYPOINT
