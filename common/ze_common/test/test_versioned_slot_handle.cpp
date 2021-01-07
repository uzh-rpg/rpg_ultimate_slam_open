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

#include <cmath>
#include <bitset>

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/versioned_slot_handle.hpp>

TEST(VersionedSlotHandle, test)
{
  using VIdx = ze::VersionedSlotHandle<uint32_t, 8, 24>;
  VIdx v1;
  EXPECT_EQ(sizeof(VIdx), 4);

  v1.slot = 1;
  v1.version = 1;
  VLOG(1) << std::bitset<32>(v1.handle);
  EXPECT_EQ(v1.handle, 1 + (1<<8));

  v1.slot = VIdx::maxSlot();
  v1.version = VIdx::maxVersion();
  VLOG(1) << std::bitset<32>(v1.handle);
  EXPECT_EQ(v1.handle, std::numeric_limits<uint32_t>::max());

  VIdx v2;
  EXPECT_EQ(v2.handle, 0);

  VIdx v3(10);
  EXPECT_EQ(v3.handle, 10);

  VIdx v4(10, 2);
  EXPECT_EQ(v4.slot, 10);
  EXPECT_EQ(v4.version, 2);
}

ZE_UNITTEST_ENTRYPOINT
