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

#include <ze/common/types.hpp>

namespace ze {

//! Utilities for working with timestamps.
//!
//! Important: Always store int64_t nanosecond timestamps! We use signed type
//!            to avoid errors when taking differences and we use nanoseconds
//!            when saving to file to have a unique type for lookups in
//!            dictionaries/maps.

//! Seconds to nanoseconds.
inline constexpr int64_t secToNanosec(real_t seconds)
{
  return static_cast<int64_t>(seconds * 1e9);
}

//! Milliseconds to nanoseconds.
inline constexpr int64_t millisecToNanosec(real_t milliseconds)
{
  return static_cast<int64_t>(milliseconds * 1e6);
}

//! Nanoseconds to seconds.
//! WARNING: Don't pass very large or small numbers to this function as the
//!          representability of the float value does not capture nanoseconds
//!          resolution. The resulting accuracy will be in the order of
//!          hundreds of nanoseconds.
inline constexpr real_t nanosecToSecTrunc(int64_t nanoseconds)
{
  return static_cast<real_t>(nanoseconds) / 1e9;
}

//! Nanoseconds to milliseconds.
//! WARNING: Don't pass very large or very small numbers to this function as the
//!          representability of the float value does not capture nanoseconds
//!          resolution.
inline constexpr real_t nanosecToMillisecTrunc(int64_t nanoseconds)
{
  return static_cast<real_t>(nanoseconds) / 1e6;
}

//! Return total nanoseconds from seconds and nanoseconds pair.
inline constexpr int64_t nanosecFromSecAndNanosec(int32_t sec, int32_t nsec)
{
  return static_cast<int64_t>(sec) * 1000000000ll + static_cast<int64_t>(nsec);
}

} // namespace ze
