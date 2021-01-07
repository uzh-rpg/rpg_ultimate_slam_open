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

#include <ze/common/benchmark.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/random_matrix.hpp>
#include <ze/common/running_statistics.hpp>

TEST(RandomMatrixTests, testRandomVectorSampler)
{
  using namespace ze;

  Vector2 var_vector;
  var_vector << 2, 3;
  RandomVectorSampler<2>::Ptr sampler = RandomVectorSampler<2>::variances(var_vector, true);
  RandomVectorSampler<3>::Ptr sampler2 = RandomVectorSampler<3>::sigmas(Vector3(1, 2, 3));

  Vector2 sample = sampler->sample();
  Vector3 sample2 = sampler2->sample();
}

TEST(RandomMatrixTests, testRandomVector)
{
  using namespace ze;

  VLOG(1) << "Deterministic:";
  Vector2 v1 = randomVectorUniformDistributed<2>(true);
  EXPECT_NEAR(v1(0), 0.592845, 1e-5);
  EXPECT_NEAR(v1(1), 0.844266, 1e-5);
  Vector2 v2 = randomVectorUniformDistributed<2>(true);
  EXPECT_NEAR(v2(0), 0.857946, 1e-5);
  EXPECT_NEAR(v2(1), 0.847252, 1e-5);

  VLOG(1) << "\n" << randomMatrixUniformDistributed<2,3>(true);
  VLOG(1) << "\n" << randomMatrixNormalDistributed<2,3>(true);
  VLOG(1) << "\n" << randomVectorNormalDistributed<4>(true).transpose();
  VLOG(1) << "\n" << randomMatrixNormalDistributed(2, 3, true);
  VLOG(1) << "Nondeterministic:";
  VLOG(1) << "\n" << randomMatrixUniformDistributed<2,3>();
  VLOG(1) << "\n" << randomMatrixNormalDistributed<2,3>();
  VLOG(1) << "\n" << randomVectorNormalDistributed<4>().transpose();
  VLOG(1) << "\n" << randomMatrixNormalDistributed(2, 3);
}

ZE_UNITTEST_ENTRYPOINT
