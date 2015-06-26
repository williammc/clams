#pragma once
#include <chrono>
#include <string>

namespace clams {
class ScopedTimer {
public:
  ScopedTimer(const std::string &description = "ScopedTimer");

  void StopAndPrint();
  ~ScopedTimer();

  using Timepoint = std::chrono::high_resolution_clock::time_point;
  using DurationT = std::chrono::duration<double>;

  Timepoint start_;
  std::string desc_;
  bool printed_;

};

} // namespace clams