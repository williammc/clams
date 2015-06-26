#include "clams/trajectory.h"
#include <fstream>
#include <iostream>
#include "clams/common/clams_macros.h"
#include "clams/serialization/serialization.h"

namespace clams {

/// [tx, ty, tz, rx, ry, rz, rw] to affine transformation
Eigen::Affine3d to_affine(const std::array<float, 7>& cam) {
  Eigen::Affine3d pose;
  pose.setIdentity();
  Eigen::Quaterniond rotation(cam[6], cam[3], cam[4], cam[5]);
  
  pose = rotation * pose;

  pose.translation()(0) = cam[0];
  pose.translation()(1) = cam[1];
  pose.translation()(2) = cam[2];
  return pose;
}

/// follow TUM format [tx, ty, tz, rx, ry, rz, rw]
void ReadPoseLineTUM(std::istream& is, double& timestamp,
  Eigen::Affine3d& pose, bool left_handed) {
  std::array<float, 7> cam;  // tx, ty, tz, qx, qy, qz, qw
  is >> timestamp >> cam[0] >> cam[1] >> cam[2] >> cam[3] >> cam[4] >> cam[5] >> cam[6];
#ifdef _DEBUG  
  std::cout << "line:" << timestamp << " " << cam[0] << " " << cam[1] << " " << cam[2] 
  << " " << cam[3] << " " << cam[4] << " " << cam[5] << " " << cam[6] << "\n";
#endif
  pose = to_affine(cam);
}

// affine to [tx, ty, tz, rx, ry, rz, rw]
std::array<double, 7> to_cam(const Eigen::Affine3d& m) {
  std::array<double, 7> cam;
  Eigen::Vector3d t = m.translation();
  for (int i = 0; i < 3; ++i)
    cam[i] = t[i];
  Eigen::Quaterniond quad;
  quad = m.rotation();
  cam[3] = quad.x();
  cam[4] = quad.y();
  cam[5] = quad.z();
  cam[6] = quad.w();
  return cam;
}

Trajectory::Trajectory(const Trajectory &other) { *this = other; }

Trajectory &Trajectory::operator=(const Trajectory &other) {
  resize(other.size());
  for (size_t i = 0; i < transforms_.size(); ++i)
    if (other.exists(i))
      transforms_[i].reset(new Eigen::Affine3d(other.get(i)));

  return *this;
}

Trajectory::~Trajectory() { clear(); }

void Trajectory::clear() {
  transforms_.clear();
}

void Trajectory::resize(size_t num) {
  clear();
  transforms_.resize(num);
}

void Trajectory::set(size_t idx, const Eigen::Affine3d &transform) {
  if (transforms_[idx])
    *transforms_[idx] = transform;
  else
    transforms_[idx].reset(new Eigen::Affine3d(transform));
}

const Eigen::Affine3d &Trajectory::get(size_t idx) const {
  if (idx > transforms_.size()) {
    std::cout << "Tried to get transform " << idx << " / "
                                               << transforms_.size();
    abort();
  }
  return *transforms_[idx];
}

bool Trajectory::exists(size_t idx) const {
  if (transforms_[idx]) return true;
  return false;
}

void Trajectory::remove(size_t idx) {
  transforms_[idx].reset();
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

bool Trajectory::LoadExternalTUM(std::string filename) {
  std::ifstream ifs(filename);
  if (ifs.fail()) {
    printf("ReadPerframeDataVec() Cannot open file from %s\n", filename.c_str());
    return false;
  }

  transforms_.reserve(500);
  while (!ifs.eof()) {
    timestamps_.push_back(0.0);
    transforms_.push_back(std::unique_ptr<Eigen::Affine3d>(new Eigen::Affine3d()));
    ReadPoseLineTUM(ifs, timestamps_.back(), *transforms_.back(), false);
  }
  return true;
}

void Trajectory::SaveExternalTUM(std::string filename) {
  std::ofstream ofs(filename);
  if (ofs.fail()) {
    printf("ReadPerframeDataVec() Cannot open file to write %s\n", filename.c_str());
    return;
  }

  for (int i = 0; i < timestamps_.size(); ++i) {
    auto cam = to_cam(*transforms_[i]);
    ofs << timestamps_[i] << " " << cam [ 0] << " "
    << cam [ 1] << " "
    << cam [ 2] << " "
    << cam [ 3] << " "
    << cam [ 4] << " "
    << cam [ 5] << " "
    << cam [ 6];
    if (i < int(timestamps_.size()-1)) ofs << "\n";
  }
}

CLAMS_INSTANTIATE_SERIALIZATION_T(Trajectory)
} // namespace clams
