#pragma once
#include <Eigen/Geometry>
#include <array>
#include <memory>
#include <vector>
#include "clams/common/clams_macros.h"

namespace clams {

/// [tx, ty, tz, rx, ry, rz, rw] to affine transformation
Eigen::Affine3d to_affine(const std::array<float, 7>& cam);

/// follow TUM format [tx, ty, tz, rx, ry, rz, rw]
void read_pose_line_TUM(std::istream& is, double& timestamp,
  Eigen::Affine3d& pose, bool left_handed);

// affine to [tx, ty, tz, rx, ry, rz, rw]
std::array<double, 7> to_cam(const Eigen::Affine3d& m);

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
  void set(size_t idx, const Eigen::Affine3d &transform, double timestamp);
  const Eigen::Affine3d &get(size_t idx) const;
  bool exists(size_t idx) const;
  void remove(size_t idx);
  size_t size() const { return transforms_.size(); }
  size_t numValid() const;
  std::string status(const std::string &prefix) const;

  bool LoadExternalTUM(std::string filename);
  void SaveExternalTUM(std::string filename);

protected:
  std::vector<double> timestamps_;
  std::vector<std::shared_ptr<Eigen::Affine3d>> transforms_;

public:
  // for serialization
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar& timestamps_;
    ar& transforms_;
  }
};
} // namespace clams