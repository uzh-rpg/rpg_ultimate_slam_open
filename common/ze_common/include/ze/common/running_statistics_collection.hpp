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

#include <array>
#include <string>
#include <sstream>

#include <ze/common/file_utils.hpp>
#include <ze/common/logging.hpp>
#include <ze/common/string_utils.hpp>
#include <ze/common/types.hpp>
#include <ze/common/running_statistics.hpp>

namespace ze {

/*! Collect samples and iteratively compute statistics.
 *
 * Usage:
\code{.cpp}
  DECLARE_STATISTICS(StatisticsName, stats, foo, bar);
  ze::StatisticsCollection stats;
  stats[StatisticsName::foo].addSample(12);
  stats[StatisticsName::foo].addSample(10);
  real_t variance = stats[StatisticsName::foo].var();
\endcode
*/
template<typename StatisticsEnum>
class StatisticsCollection
{
public:
  using Collection = std::array<RunningStatistics,
                                static_cast<uint32_t>(StatisticsEnum::dimension)>;
  using CollectionNames = std::vector<std::string>;

  StatisticsCollection() = delete;

  //! This constructor is used by the macro DECLARE_TIMER() below.
  StatisticsCollection(const std::string& statistics_names_comma_separated)
    : names_(splitString(statistics_names_comma_separated, ','))
  {
    CHECK_EQ(names_.size(), collection_.size());
  }

  StatisticsCollection(const std::vector<std::string>& timer_names)
    : names_(timer_names)
  {
    CHECK_EQ(names_.size(), collection_.size());
  }

  ~StatisticsCollection() = default;

  inline RunningStatistics& operator[](StatisticsEnum s)
  {
    return collection_[static_cast<uint32_t>(s)];
  }

  inline const RunningStatistics& operator[](StatisticsEnum s) const
  {
    return collection_[static_cast<uint32_t>(s)];
  }

  constexpr size_t size() const noexcept { return collection_.size(); }

  //! Saves statistics to file in YAML format.
  void saveToFile(const std::string& directory, const std::string& filename)
  {
    std::ofstream fs;
    CHECK(isDir(directory));
    openOutputFileStream(joinPath(directory, filename), &fs);
    fs << *this;
  }

  inline const Collection& collection() const { return collection_; }

  inline const CollectionNames& names() const { return names_; }

private:
  Collection collection_;
  CollectionNames names_;
};

//! Print statistics collection.
//! Print Timer Collection:
template<typename StatisticsEnum>
std::ostream& operator<<(std::ostream& out,
                         const StatisticsCollection<StatisticsEnum>& statistics)
{
  for (size_t i = 0u; i < statistics.size(); ++i)
  {
    out << statistics.names().at(i) << ":\n"
        << statistics.collection().at(i);
  }
  return out;
}

#define DECLARE_STATISTICS(classname, membername, ...)              \
  enum class classname : uint32_t { __VA_ARGS__, dimension };  \
  ze::StatisticsCollection<classname> membername { #__VA_ARGS__ }

} // end namespace ze
