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

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/matrix.hpp>
#include <ze/common/types.hpp>

TEST(MatrixTests, testVectorSlice)
{
  using namespace ze;

  VectorX M(5);
  M << 1, 2, 3, 4, 6;
  std::vector<uint32_t> indices { 0, 2, 3 };

  M = getVectorElements(M, indices);

  VectorX A_expected(3);
  A_expected << 1, 3, 4;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(M, A_expected));
}

TEST(MatrixTests, testColumnSlice)
{
  using namespace ze;

  Matrix2X M(2, 5);
  M << 1, 2, 3, 4, 6,
       7, 8, 9, 10, 11;
  std::vector<uint32_t> indices { 0, 2, 3 };

  M = getMatrixCols(M, indices);

  Matrix2X A_expected(2,3);
  A_expected << 1, 3, 4, 7, 9, 10;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(M, A_expected));
}

ZE_UNITTEST_ENTRYPOINT
