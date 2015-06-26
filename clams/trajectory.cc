#include "clams/trajectory.h"
#include <iostream>

namespace clams {

Trajectory::Trajectory(const Trajectory &other) { *this = other; }

Trajectory &Trajectory::operator=(const Trajectory &other) {
  resize(other.size());
  for (size_t i = 0; i < transforms_.size(); ++i)
    if (other.exists(i))
      transforms_[i] = new Eigen::Affine3d(other.get(i));

  return *this;
}

Trajectory::~Trajectory() { clear(); }

void Trajectory::clear() {
  for (size_t i = 0; i < transforms_.size(); ++i)
    if (transforms_[i])
      delete transforms_[i];
  transforms_.clear();
}

void Trajectory::resize(size_t num) {
  clear();
  transforms_.resize(num, NULL);
}

void Trajectory::set(size_t idx, const Eigen::Affine3d &transform) {
  if (transforms_[idx])
    *transforms_[idx] = transform;
  else
    transforms_[idx] = new Eigen::Affine3d(transform);
}

const Eigen::Affine3d &Trajectory::get(size_t idx) const {
  if (idx > transforms_.size()) {
    std::cout << "Tried to get transform " << idx << " / "
                                               << transforms_.size();
    abort();
  }
  return *transforms_[idx];
}

bool Trajectory::exists(size_t idx) const { return transforms_[idx]; }

void Trajectory::remove(size_t idx) {
  if (transforms_[idx])
    delete transforms_[idx];

  transforms_[idx] = NULL;
}

size_t Trajectory::numValid() const {
  size_t num = 0;
  for (size_t i = 0; i < transforms_.size(); ++i)
    if (transforms_[i])
      ++num;
  return num;
}

std::string Trajectory::status(const std::string &prefix) const {
  std::ostringstream oss;
  oss << prefix << "Num valid: " << numValid() << " / " << size() << "\n";
  // for(size_t i = 0; i < transforms_.size(); ++i) {
  //   if(transforms_[i]) {
  //     oss << prefix << "Transform " << i << endl;
  //     for(int j = 0; j < 4; ++j)
  //         oss << prefix << "  " << transforms_[i]->matrix().row(j) << endl;
  //   }
  // }

  return oss.str();
}

} // namespace clams
