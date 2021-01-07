#pragma once

#include <cstdint>

namespace ze {

enum class InterpolationMode
{
  Point,
  Linear,
  Cubic,
  CubicSpline
};

enum class MemoryType
{
  Undefined,
  Cpu,
  CpuAligned,
  Gpu,
  GpuAligned,
  Unified,
  UnifiedAligned,
  Managed,
  ManagedAligned
};

} // namespace ze

