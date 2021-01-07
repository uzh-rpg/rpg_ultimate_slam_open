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

#include <Eigen/Core>

#include <ze/common/logging.hpp>
#include <yaml-cpp/yaml.h>

namespace YAML {

template <typename T>
T extractChild(const YAML::Node& node, const std::string& key)
{
  if (!node.IsMap())
  {
    throw YAML::Exception(node.Mark(), "Node is not a map");
  }

  const YAML::Node child = node[key];
  if (!child)
  {
    throw YAML::Exception(node.Mark(), "key '" + key + "' does not exist");
  }

  return child.as<T>();
}

/**
 * yaml serialization helper function for the Eigen3 Matrix object.
 * The matrix is a base class for dense matrices.
 * http://eigen.tuxfamily.org/dox-devel/TutorialMatrixClass.html
 *
 * see yaml-cpp for how the extraction of complex types works
 *
 * Inspired by ETHZ-ASL ASLAM_CV_COMMON
 */
template <class Scalar, int A, int B, int C, int D, int E>
struct convert<Eigen::Matrix<Scalar, A, B, C, D, E> >
{
  static bool decode(const Node& node,
                     Eigen::Matrix<Scalar, A, B, C, D, E>& M)
  {
    if(!node.IsMap())
    {
      LOG(ERROR) << "Unable to parse the matrix because the node is not a map.";
      return false;
    }

    typedef typename Eigen::Matrix<Scalar, A, B, C, D, E>::Index IndexType;
    IndexType rows = node["rows"].as<IndexType>();
    IndexType cols = node["cols"].as<IndexType>();

    if((A != Eigen::Dynamic && rows != A) ||
       (B != Eigen::Dynamic && cols != B))
    {
      LOG(ERROR) << "The matrix is the wrong size (rows, cols). Wanted: (" << (A==Eigen::Dynamic?rows:A) << ","
          << (B==Eigen::Dynamic?cols:B) << "), got (" << rows << ", " << cols << ")";
      return false;
    }

    M.resize(rows, cols);

    size_t expected_size = M.rows() * M.cols();
    if (!node["data"].IsSequence())
    {
      LOG(ERROR) << "The matrix data is not a sequence.";
      return false;
    }
    if(node["data"].size() != expected_size)
    {
      LOG(ERROR) << "The data sequence is the wrong size. Wanted: " << expected_size <<
          ", got: " << node["data"].size();
      return false;
    }

    YAML::const_iterator it = node["data"].begin();
    YAML::const_iterator it_end = node["data"].end();
    if(rows > 0 && cols > 0)
    {
      for(IndexType i = 0; i < rows; ++i)
      {
        for(IndexType j = 0; j < cols; ++j)
        {
          CHECK(it != it_end);
          M(i, j) = it->as<Scalar>();
          ++it;
        }
      }
    }
    return true;
  }
};

}  // namespace YAML
