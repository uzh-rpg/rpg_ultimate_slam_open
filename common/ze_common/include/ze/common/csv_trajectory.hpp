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

#include <ze/common/buffer.hpp>
#include <ze/common/file_utils.hpp>
#include <ze/common/macros.hpp>
#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>

namespace ze {

//! Reading of various csv trajectory file formats (e.g. swe, euroc, pose).
//! Reads the result in a buffer that allows accessing the pose via the
//! timestamps.
class CSVTrajectory
{
public:
  ZE_POINTER_TYPEDEFS(CSVTrajectory);

  virtual void load(const std::string& in_file_path) = 0;
  virtual int64_t getTimeStamp(const std::string& ts_str) const;

protected:
  CSVTrajectory() = default;

  void readHeader(const std::string& in_file_path);
  Vector3 readTranslation(const std::vector<std::string>& items);
  Vector4 readOrientation(const std::vector<std::string>& items);
  Vector7 readPose(const std::vector<std::string>& items);

  std::ifstream in_str_;
  std::map<std::string, int> order_;
  std::string header_;
  const char delimiter_{','};
  size_t num_tokens_in_line_;
};

class PositionSeries : public CSVTrajectory
{
public:
  ZE_POINTER_TYPEDEFS(PositionSeries);

  PositionSeries();
  virtual void load(const std::string& in_file_path) override;
  const Buffer<real_t, 3>& getBuffer() const;
  Buffer<real_t, 3>& getBuffer();

protected:
  Buffer<real_t, 3> position_buf_;
};

class PoseSeries : public CSVTrajectory
{
public:
  ZE_POINTER_TYPEDEFS(PoseSeries);

  PoseSeries();

  virtual void load(const std::string& in_file_path) override;
  virtual const Buffer<real_t, 7>& getBuffer() const;
  virtual Buffer<real_t, 7>& getBuffer();
  virtual StampedTransformationVector getStampedTransformationVector();

  static Transformation getTransformationFromVec7(const Vector7& data);

protected:
  Buffer<real_t, 7> pose_buf_;
};

class SWEResultSeries : public PoseSeries
{
public:
  SWEResultSeries();
};

class SWEGlobalSeries : public PoseSeries
{
public:
  SWEGlobalSeries();
};

class EurocResultSeries : public PoseSeries
{
public:
  EurocResultSeries();
};

} // ze namespace
