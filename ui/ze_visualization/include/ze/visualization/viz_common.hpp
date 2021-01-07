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
//
// Modified: Robotics and Perception Group

#pragma once

#include <ze/common/types.hpp>

namespace ze {

struct Color
{
  real_t r = 0.0f;
  real_t g = 0.0f;
  real_t b = 0.0f;
  real_t a = 1.0f;

  constexpr Color(real_t r, real_t g, real_t b)
    : r(r), g(g), b(b)
  {}

  constexpr Color(real_t r, real_t g, real_t b, real_t a)
    : r(r), g(g), b(b), a(a)
  {}
};

struct Colors
{
  static constexpr Color Red {1.0, 0.0, 0.0};
  static constexpr Color Green {0.0, 1.0, 0.0};
  static constexpr Color Blue {0.0, 0.0, 1.0};
  static constexpr Color DarkRed {0.5, 0.0, 0.0};
  static constexpr Color DarkGreen {0.0, 0.5, 0.0};
  static constexpr Color DarkBlue {0.0, 0.0, 0.5};
  static constexpr Color White {1.0, 1.0, 1.0};
  static constexpr Color LightGray {0.8, 0.8, 0.8};
  static constexpr Color Yellow {1.0, 1.0, 0.0};
  static constexpr Color Magenta {1.0, 0.0, 1.0};
  static constexpr Color Orange {1.0, 0.5, 0.0};
};

} // namespace ze
