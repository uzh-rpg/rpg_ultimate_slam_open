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

#include <numeric>
#include <vector>
#include <ze/common/types.hpp>
#include <ze/common/logging.hpp>

//! @file stl_utils.hpp
//! Various utilities to work with the standard template library.

namespace ze {

// -----------------------------------------------------------------------------
//! Transform Eigen::Vector to std::vector.
template <typename DerivedVec>
std::vector<typename DerivedVec::Scalar> eigenVectorToStlVector(
    const Eigen::MatrixBase<DerivedVec>& v)
{
  //! @todo: both data is continuous, can we do this more efficiently?
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedVec);
  std::vector<typename DerivedVec::Scalar> rv(v.size());
  for(int i = 0; i < v.size(); ++i)
  {
    rv[i] = v(i);
  }
  return rv;
}

// -----------------------------------------------------------------------------
//! @return Returns a vector of indices form start to stop.
inline std::vector<uint32_t> range(uint32_t start, uint32_t stop)
{
  DEBUG_CHECK_GE(stop, start);
  std::vector<uint32_t> vec(stop - start);
  std::iota(vec.begin(), vec.end(), start);
  return vec;
}

// -----------------------------------------------------------------------------
//! @return Returns a vector of indices form 0 to stop.
inline std::vector<uint32_t> range(uint32_t stop)
{
  return range(0u, stop);
}

} // namespace ze
