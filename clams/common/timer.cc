#include "clams/common/timer.h"
#include <iostream>

namespace clams {
ScopedTimer::ScopedTimer(const std::string &description) : desc_(description) {
  start_ = std::chrono::high_resolution_clock::now();;
  printed_ = false;
}

ScopedTimer::~ScopedTimer() {
  if (!printed_)
    StopAndPrint();
}

void ScopedTimer::StopAndPrint() {
  auto now = std::chrono::high_resolution_clock::now();;
  auto t = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_);
  std::cout << desc_ << ":" << t.count() << "milliseconds" << std::endl;
  printed_ = true;
}

} // namespace clams