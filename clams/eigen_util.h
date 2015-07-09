#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "slick/math/se3.h"

namespace clams {

inline slick::SE3d to_se3(const Eigen::Affine3d& pose) {
  Eigen::Matrix3d m = pose.matrix().block<3, 3>(0, 0);
  Eigen::Vector3d t = pose.matrix().block<3, 1>(0, 3);
  return slick::SE3d(slick::SO3d(m), t);
}

inline Eigen::Affine3d to_affine(const Eigen::Isometry3d& pose) {
  Eigen::Affine3d p(pose.rotation());
  p.translation() = pose.translation();

  return p;
}

inline Eigen::Affine3d to_affine(const slick::SE3d& pose) {
  Eigen::Affine3d p(pose.get_rotation().get_matrix());
  p.translation() = pose.get_translation();

  return p;
}

// @cam: tx, ty, tz, qx, qy, qz, qw
inline Eigen::Affine3d to_affine(const std::array<double, 7>& cam) {
  Eigen::Quaternion<double> rotation(cam[6], cam[3], cam[4], cam[5]);
  Eigen::Translation<double, 3> translation(cam[0], cam[1], cam[2]);
  Eigen::Affine3d transform = translation * rotation;
  return transform;
}

// @cam: width, height, fx, fy, cx, cy, k1, k2, k3
inline Eigen::Matrix3d to_projection(const std::array<double, 9>& cam) {
  Eigen::Matrix3d proj;
  proj << cam[2], 0, cam[4],
          0, cam[3], cam[5],
          0, 0, 1;
  return proj;
}

} // namespace clams