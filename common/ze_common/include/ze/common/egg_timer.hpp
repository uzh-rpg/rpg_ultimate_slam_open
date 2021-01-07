#pragma once

#include <memory>
#include <chrono>

namespace ze {
class EggTimer {
public:
  explicit EggTimer(uint64_t duration_nanoseconds)
    : duration_ns_(duration_nanoseconds),
      start_time_(std::chrono::high_resolution_clock::now()) {}

  void reset() {
    start_time_ = std::chrono::high_resolution_clock::now();
  }

  bool finished() const {
    return (std::chrono::high_resolution_clock::now() - start_time_) >=
        duration_ns_;
  }

private:
  const std::chrono::duration<uint64_t, std::nano> duration_ns_;
  std::chrono::high_resolution_clock::time_point start_time_;
};
} // End ze namespace
