#pragma once

#include "slick/scene/three_point_pose.h"
#include "slick/util/wls.h"

namespace clams {

template <class Precision, typename OtherDerived, int O>
static inline Eigen::Matrix<Precision, 2, 1> project_transformed_point(
  const  slick::SO3Group<Precision> & pose,
    const Eigen::MatrixBase<OtherDerived>& in_frame,
    Eigen::Matrix<Precision, 2, 3, O>& J_pose) {
  const Precision z_inv = 1.0/in_frame(2, 0);
  const Precision x_z_inv = in_frame(0, 0)*z_inv;
  const Precision y_z_inv = in_frame(1, 0)*z_inv;
  const Precision cross = x_z_inv * y_z_inv;
  J_pose(0, 0) = -cross;
  J_pose(0, 1) = 1 + x_z_inv*x_z_inv;
  J_pose(0, 2) = -y_z_inv;
  J_pose(1, 0) = -1 - y_z_inv*y_z_inv;
  J_pose(1, 1) =  cross;
  J_pose(1, 2) =  x_z_inv;

  return Eigen::Matrix<Precision,2,1>(x_z_inv, y_z_inv);
}

template <typename Precision, int O1, int O2>
static inline Eigen::Matrix<Precision,2,1> transform_and_project(
  const  slick::SO3Group<Precision>& pose, const Eigen::Matrix<Precision, 3, 1, O1>& x,
    Eigen::Matrix<Precision, 2, 3, O2>& J_pose) {
  return project_transformed_point(pose, pose * x, J_pose);
}

template <typename OtherDerived>
inline Eigen::Matrix<typename OtherDerived::Scalar, 2, 1> perspective_project(
    const Eigen::MatrixBase <OtherDerived>& x ) {
  return Eigen::Matrix<typename OtherDerived::Scalar, 2, 1>(x(0, 0)/x(2, 0),x(1, 0)/x(2, 0));
}

template <typename P, int O1, int O2>
inline Eigen::Matrix<P, 2, 1> transform_and_project(
  const  slick::SE3Group<P>& pose, const Eigen::Matrix<P, 4, 1, O1>& x,
    Eigen::Matrix<P, 2, 6, O2>& J_pose) {
  const Eigen::Matrix<P,4,1> in_frame =  pose * x;

  const P z_inv = 1.0/in_frame[2];
  const P x_z_inv = in_frame[0]*z_inv;
  const P y_z_inv = in_frame[1]*z_inv;
  const P cross = x_z_inv * y_z_inv;
  J_pose(0, 0) = J_pose(1, 1) = z_inv * in_frame[3];
  J_pose(0, 1) = J_pose(1, 0) = 0;
  J_pose(0, 2) = -x_z_inv * z_inv * in_frame[3];
  J_pose(1, 2) = -y_z_inv * z_inv * in_frame[3];
  J_pose(0, 3) = -cross;
  J_pose(0, 4) = 1 + x_z_inv*x_z_inv;
  J_pose(0, 5) = -y_z_inv;
  J_pose(1, 3) = -1 - y_z_inv*y_z_inv;
  J_pose(1, 4) =  cross;
  J_pose(1, 5) =  x_z_inv;

  return Eigen::Matrix<P, 2, 1>(x_z_inv, y_z_inv);
}

inline std::pair<slick::SE3d, double>
optimise_pose(const std::vector<std::pair<Eigen::Vector4d, Eigen::Vector2d>> &
                  observations,
              slick::SE3d& pose, std::vector<bool> &inliers, double threshold) {
  inliers.resize(observations.size());
  unsigned counter = 0;
  slick::WLS<double, 6> wls;
  Eigen::Matrix<double, 2, 6> J_pose;
  Eigen::Matrix<double, 6, 1> delta;
  const Eigen::Matrix<double, 2, 2> W = Eigen::Matrix<double, 2, 2>::Identity();
  std::vector<double> sq_errors;
  double sum_squared_error = 0;
  do {
    wls.clear();
    wls.add_prior(1e-8);
    // calculate sigma
    sq_errors.clear();
    for (unsigned i = 0; i < observations.size(); ++i) {
      const Eigen::Vector2d v2_error =
          observations[i].second - perspective_project(pose * observations[i].first);
      sq_errors.push_back(v2_error.squaredNorm());
    }

    slick::Tukey<double> est;
    est.compute_sigma_squared(sq_errors);
    sum_squared_error = 0.0;
    // do optimisation
    for (unsigned i = 0; i < observations.size(); ++i) {
      const Eigen::Vector2d v2_error =
          observations[i].second - transform_and_project(pose, observations[i].first, J_pose);
      const double e = v2_error.squaredNorm();
      const double w = est.weight(e);
      sum_squared_error += e*w;
      if (w > threshold) {
        inliers[i] = true;
      } else {
        inliers[i] = false;
      }
      Eigen::Matrix<double, 2, 2> Ww = W*w;
      wls.add_mJ_rows(v2_error, J_pose, Ww);
    }
    wls.compute();
    delta = wls.get_mu();
    pose = slick::SE3d(delta) * pose;
    ++counter;
  } while (delta.squaredNorm() > 1e-9 && counter < 10);
  return std::make_pair(pose, sum_squared_error/observations.size());
}

// Find best camera pose in 2 steps
// 1st: use 3-point-pose algorithm to compute inital poses
// 2nd: optimise the best inital pose (from the 1st step) using WLS
inline std::pair<slick::SE3d, double>
FindBestPose2Steps(CenterLocPairVec &observations) {
  slick::SE3d se3_best;
  std::vector<std::pair<Eigen::Vector4d, Eigen::Vector2d>> good_observations;
  // first-step: applying RANSAC while using three-point-pose algorithm to get draft poses
  {
    std::vector<slick::SE3d > possible_poses;
    Eigen::Vector3d world_points[3];
    Eigen::Vector2d camera_points[3];
    int i_rand = 0;
    for (size_t i = 0; i < 50; i++) {
      std::random_shuffle(observations.begin(),observations.end());
      i_rand = std::rand() % observations.size();
      if (i_rand-3 > 0) {
        world_points[0] = slick::project(observations[i_rand].first);
        world_points[1] = slick::project(observations[i_rand - 1].first);
        world_points[2] = slick::project(observations[i_rand - 2].first);
        camera_points[0] = observations[i_rand].second;
        camera_points[1] = observations[i_rand-1].second;
        camera_points[2] = observations[i_rand-2].second;

        std::vector<slick::SE3d > vPoses;
        slick::CalcThreePointPoses(world_points,camera_points,vPoses);
        bool bValid = false;
        for (size_t i = 0; i < vPoses.size(); ++i) {
          slick::SE3d se3Pose = vPoses[i];
          bValid=false;
          Eigen::Vector3d v3Cam = se3Pose*slick::project(observations[i_rand].first);
          Eigen::Vector2d verror = observations[i_rand].second - slick::project(v3Cam);
          if (v3Cam[2] > 0 && verror.squaredNorm() < 1.e-9)
            possible_poses.push_back(se3Pose);
        }
      }
    }
    //get the best pose
    double min_errorsq = 1.e+9;
    double sum_errorsq = 0.0;
    slick::Tukey<double> est;
    for (size_t i = 0; i < possible_poses.size(); i++) {
      std::vector<double> sqerrors;
      sqerrors.reserve( observations.size());
      for (size_t j = 0; j < observations.size(); j++) {
        Eigen::Vector2d verror =
            observations[j].second - slick::project(possible_poses[i]*slick::project(observations[j].first));
        sqerrors.push_back(verror.squaredNorm());
      }

      est.compute_sigma_squared(sqerrors);

      sum_errorsq = 0.0;
      for (size_t j = 0; j < observations.size(); j++) {
        Eigen::Vector2d verror =
            observations[j].second - slick::project(possible_poses[i]*slick::project(observations[j].first));
        double dErrorSq = verror.squaredNorm();
        const double w = est.weight(dErrorSq);
        sum_errorsq += w*dErrorSq;
      }
      if (min_errorsq > sum_errorsq) {
        min_errorsq = sum_errorsq;
        se3_best = possible_poses[i];
      }
    }

    // add inlier observations to good list
    const double average_errorsq = min_errorsq/possible_poses.size();
    for (size_t j = 0; j < observations.size(); j++) {
      Eigen::Vector2d v2_error =
          observations[j].second - slick::project(se3_best*slick::project(observations[j].first));
      if(v2_error.squaredNorm()<average_errorsq)
        good_observations.push_back(observations[j]);
    }
  }
  // Second step: pose estimation (6DOF) using WLS
  std::vector<bool> inliers;
  return optimise_pose(good_observations, se3_best, inliers, 0.8);
}

} // namespace clams