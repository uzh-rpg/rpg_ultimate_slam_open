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

//! A template for a data handle that is composed of the slot index (in an array)
//! plus a version number. The indexed array stores the version number of each
//! object and can therefore tell if the object at the slot is at the same version we
//! request.
//! http://seanmiddleditch.com/data-structures-for-game-developers-the-slot-map/
//! http://blog.molecular-matters.com/2013/05/17/adventures-in-data-oriented-design-part-3b-internal-references/
template <typename T, int NumSlotBits, int NumVersionBits>
union VersionedSlotHandle
{
  using value_t = T;

  struct
  {
    // The first NumSlotBits in this bit-field store the slot, the next
    // NumVersionBits store the version number of the handle.
    T slot    : NumSlotBits;
    T version : NumVersionBits;
  };
  T handle;

  // Always initialize handle first to zero to make sure that all bits are really 0.
  VersionedSlotHandle() : handle(0) {}
  VersionedSlotHandle(T handle) : handle(handle) {}
  VersionedSlotHandle(T _slot, T _version) : handle(0) { slot = _slot; version = _version; }

  static constexpr T maxSlot() { return (1 << NumSlotBits) - 1; }
  static constexpr T maxVersion() { return (1 << NumVersionBits) - 1; }

  void reset() { handle = T{0}; }
};

template <typename T, int NumSlotBits, int NumVersionBits>
inline bool operator==(const VersionedSlotHandle<T, NumSlotBits, NumVersionBits> lhs,
                       const VersionedSlotHandle<T, NumSlotBits, NumVersionBits> rhs)
{
  return lhs.handle == rhs.handle;
}

template <typename T, int NumSlotBits, int NumVersionBits>
inline bool operator!=(const VersionedSlotHandle<T, NumSlotBits, NumVersionBits> lhs,
                       const VersionedSlotHandle<T, NumSlotBits, NumVersionBits> rhs)
{
  return !(lhs.handle == rhs.handle);
}

template <typename T, int NumSlotBits, int NumVersionBits>
inline std::ostream& operator<<(
    std::ostream& out, const VersionedSlotHandle<T, NumSlotBits, NumVersionBits> handle)
{
  out << handle.handle
      << " (slot: " << handle.slot << ", version: " << handle.version << ")";
  return out;
}

//! Compares the handle as an integer. I.e. Compare slot number first, then version.
template <typename T, int NumSlotBits, int NumVersionBits>
inline bool operator<(const VersionedSlotHandle<T, NumSlotBits, NumVersionBits> lhs,
                      const VersionedSlotHandle<T, NumSlotBits, NumVersionBits> rhs)
{
  return lhs.handle < rhs.handle;
}

} // namespace ze
