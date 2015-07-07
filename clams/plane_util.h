#pragma once
#include "clams/eigen_util.h"

namespace clams {

/// intersection of ray going through 2D pixel location (@index)
/// and 3D plane @pln, resulting 3D point
template <class VecType>
inline Eigen::Vector3d projection(const Eigen::Vector3d& cam_p,
                                  const VecType &pln) {
  auto const d = -pln[3] / cam_p.dot(Eigen::Vector3d(pln[0], pln[1], pln[2]));
  return cam_p * d;
}

/// distance between 3D projection on plane and 3D point
template <class VecType>
inline float calc_dist_projection_to_point(const cv::Point pn,
                                  const float depth,
                                  const Eigen::Vector3d& cam_p,
                                  const VecType &pln) {
  auto const d = -pln[3] / cam_p.dot(Eigen::Vector3d(pln[0], pln[1], pln[2]));
  return cam_p.norm()*std::fabs(d- depth);
}

template <class VecType>
inline float calc_dist_projection_to_point(const int index, const int w,
                                  const float depth,
                                  const Eigen::Vector3d& cam_p,
                                  const VecType &pln) {
  auto const d = -pln[3] / cam_p.dot(Eigen::Vector3d(pln[0], pln[1], pln[2]));
  return cam_p.norm()*std::fabs(d- depth);
}

/// transform plane equation
inline Eigen::Vector4d transform(const Eigen::Vector4d& pl, const Eigen::Affine3d& pose) {
  Eigen::Hyperplane<double, 3> hyper_pl(pl.head<3>(), pl[3]);
  hyper_pl = hyper_pl.transform(pose);
  const Eigen::Vector4d v4 = hyper_pl.coeffs();
  return Eigen::Vector4d(v4[0], v4[1], v4[2], v4[3]);
}

} // namespace clams