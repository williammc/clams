#pragma once
#include <functional>

namespace clams {
// source: http://blog.korfuri.fr/post/go-defer-in-cpp
class DeferredAction {
 public:
  DeferredAction(DeferredAction&& l)
      : func_(std::forward<std::function<void()>>(l.func_)) {
    l.func_ = nullptr;
  }

  ~DeferredAction() {
    if (func_) {
      func_();
    }
  }

  DeferredAction(DeferredAction const& l) = delete;
  DeferredAction& operator=(DeferredAction const&) = delete;
  DeferredAction& operator=(DeferredAction&&) = delete;

 private:
  template <typename... Tpack>
  DeferredAction(Tpack&&... p)
      : func_(std::bind(std::forward<Tpack>(p)...)) {}

  std::function<void()> func_;

  template <typename... Tpack>
  friend DeferredAction defer(Tpack&&... p);
};

template <typename... Tpack>
DeferredAction defer(Tpack&&... p) {
  return DeferredAction(std::forward<Tpack>(p)...);
}

}  // namespace clams