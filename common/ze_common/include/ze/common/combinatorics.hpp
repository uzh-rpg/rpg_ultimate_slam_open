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

#include <algorithm>
#include <functional>
#include <type_traits>
#include <vector>

#include <glog/logging.h>

#include <ze/common/types.hpp>

//! @file combinatorics.hpp
//! Some useful combinatorial functions. E.g. find matching index-entries in
//! two vectors.

namespace ze {

// -----------------------------------------------------------------------------
//! Returns indices (1. A, 2. B) of matching values in provided vectors.
template<typename T>
std::vector<std::pair<uint32_t, uint32_t>> getMatchIndices(
    const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 1>>& A,
    const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 1>>& B,
    const std::function<bool (T)>& isValidId)
{
  // First, create a sorted vector of the reference ids with corresponding index.
  std::vector<std::pair<T, uint32_t>> B_indexed;
  B_indexed.reserve(B.size());
  for(int32_t i = 0; i < B.size(); ++i)
  {
    if(isValidId(B(i)))
    {
      B_indexed.push_back(std::make_pair(B(i), i));
    }
  }
  std::sort(B_indexed.begin(), B_indexed.end(),
             [](const std::pair<T, uint32_t>& lhs, const std::pair<T, uint32_t>& rhs)
             { return lhs.first < rhs.first; });

  // For each current id, find matching reference id.
  std::vector<std::pair<uint32_t, uint32_t>> matches_AB;
  matches_AB.reserve(B_indexed.size());
  for(int32_t i = 0; i < A.size(); ++i)
  {
    if(isValidId(A(i)))
    {
      // Efficient search for matching id in sorted range.
      auto it = std::lower_bound(B_indexed.begin(),
                                 B_indexed.end(), A(i),
                                 [](const std::pair<T, uint32_t>& lhs, T rhs)
                                 { return lhs.first < rhs; });
      if(it != B_indexed.end() && it->first == A(i))
      {
        // Success.
        matches_AB.push_back(std::make_pair(i, it->second));
      }
    }
  }
  return matches_AB;
}

// -----------------------------------------------------------------------------
//! Returns indices of A that don't have a matching index in B.
template<typename T>
std::vector<uint32_t> getUnmatchedIndices(
    const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 1>>& A,
    const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 1>>& B,
    const std::function<bool (T)>& isValidId)
{
  // First, create a sorted vector of the reference ids with corresponding index.
  std::vector<std::pair<T, uint32_t>> B_indexed;
  B_indexed.reserve(B.size());
  for (int32_t i = 0; i < B.size(); ++i)
  {
    if (isValidId(B(i)))
    {
      B_indexed.push_back(std::make_pair(B(i), i));
    }
  }
  std::sort(B_indexed.begin(), B_indexed.end(),
             [](const std::pair<T, uint32_t>& lhs, const std::pair<T, uint32_t>& rhs)
             { return lhs.first < rhs.first; });

  // For each current id, find matching reference id.
  std::vector<uint32_t> unmached_A;
  unmached_A.reserve(B_indexed.size());
  for(int32_t i = 0; i < A.size(); ++i)
  {
    if(isValidId(A(i)))
    {
      // Efficient search for matching id in sorted range.
      auto it = std::lower_bound(B_indexed.begin(),
                                 B_indexed.end(), A(i),
                                 [](const std::pair<T, uint32_t>& lhs, T rhs)
                                 { return lhs.first < rhs; });
      if(it == B_indexed.end() || it->first != A(i))
      {
        // No match found:
        unmached_A.push_back(i);
      }
    }
  }
  return unmached_A;
}

// -----------------------------------------------------------------------------
template<typename T>
std::vector<T> getOutlierIndicesFromInlierIndices(
    std::vector<T>& inliers, const T size)
{
  static_assert(std::is_integral<T>::value, "Type T must be an integral type");

  std::sort(inliers.begin(), inliers.end(), std::less<T>());
  std::vector<T> outliers;

  if ((size - inliers.size()) >= 0) {
    try {
      outliers.reserve(size - inliers.size());
    } catch (...) {
      LOG(ERROR) << "Could not reserve enough memory for outliers...\n"
                 << "Requested size is: " << size - inliers.size() << " \n"
                 << "With size being: "   << size                  << " \n"
                 << "And inliers size being: " << inliers.size()   << " \n";
    }
  } else {
    LOG(ERROR) << "Requested to reserve a negative memory amount..."
               << "Requested size is: " << size - inliers.size() << " \n"
               << "With size being: "   << size                  << " \n"
               << "And inliers size being: " << inliers.size()   << " \n";
  }

  T k = 0;
  for (T i = 0; i < size; ++i)
  {
    if (k < static_cast<T>(inliers.size()) && inliers.at(k) < i)
    {
      ++k;
    }

    if (k >= size || (k < static_cast<T>(inliers.size()) && inliers.at(k) != i))
    {
      outliers.push_back(i);
    }
  }

  return outliers;
}

} // namespace ze
