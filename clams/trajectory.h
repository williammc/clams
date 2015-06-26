#pragma once
#include <Eigen/Dense>
#include <vector>

namespace clams {

class Trajectory {
public:
  Trajectory() {}
  //! Deep copy
  Trajectory(const Trajectory &other);
  //! Deep copy
  Trajectory &operator=(const Trajectory &other);
  ~Trajectory();

  void clear();
  //! Destroys everything inside.
  void resize(size_t num);
  void set(size_t idx, const Eigen::Affine3d &transform);
  const Eigen::Affine3d &get(size_t idx) const;
  bool exists(size_t idx) const;
  void remove(size_t idx);
  size_t size() const { return transforms_.size(); }
  size_t numValid() const;
  std::string status(const std::string &prefix) const;

protected:
  std::vector<Eigen::Affine3d *> transforms_;

public:
  // for serialization
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar& transforms_;
  }
};
} // namespace clams