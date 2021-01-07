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

#include <random>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/matrix.hpp>
#include <ze/common/manifold.hpp>
#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>
#include <ze/geometry/lsq_state.hpp>

TEST(StateTests, testTupleFixedSize)
{
  using namespace ze;
  using Tuple1 = std::tuple<Transformation, real_t, Vector3>;
  using Tuple2 = std::tuple<Transformation, VectorX>;
  EXPECT_TRUE(internal::TupleIsFixedSize<Tuple1>::is_fixed_size);
  EXPECT_FALSE(internal::TupleIsFixedSize<Tuple2>::is_fixed_size);
}

TEST(StateTests, testStateFixedSize)
{
  using namespace ze;

  using MyState = State<Transformation,Vector3,real_t>;
  EXPECT_EQ(MyState::dimension, 10);

  MyState state;
  state.print();

  MyState::TangentVector v;
  state.retract(v);

  EXPECT_EQ(State<Transformation>::dimension, 6);
}

TEST(StateTests, testStateDynamicSize)
{
  using namespace ze;
  using MyState = State<Transformation,VectorX>;

  // Test constructor of dynamic-sized state.
  MyState state;
  VectorX& x = state.at<1>();
  x.resize(5);
  x.setConstant(0.5);
  state.print();

  EXPECT_EQ(state.getDimension(), 11);
  EXPECT_TRUE(state.isDynamicSize());
  EXPECT_FALSE(state.isElementDynamicSize<0>());
  EXPECT_TRUE(state.isElementDynamicSize<1>());

  // Test retract.
  traits<MyState>::TangentVector v;
  v.resize(state.getDimension());
  v.setConstant(1.0);
  state.retract(v);
  state.print();
}

ZE_UNITTEST_ENTRYPOINT
