// Copyright 2012, 2013, 2014 The Look3D Authors. All rights reserved.
#pragma once

#include <iostream>
#include <Eigen/Core>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "slick/math/se3.h"

namespace clams {

using CenterLocPair = std::pair<Eigen::Vector4d, Eigen::Vector2d>;
using CenterLocPairVec = std::vector<CenterLocPair>;

struct CalibPattern {
  enum struct Type : int {
    CHECKER_BOARD = 0,
    DOT_BOARD,
    ASYM_DOT_BOARD,
    UNDEFINE
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CalibPattern() = default;

  CalibPattern(Type t, cv::Size size, float cell_unit) 
  : type(t), size(size), cell_unit(cell_unit) {
    GenWorldpoints();
  }

  bool empty() const { return worldpoints.empty(); }

  /// Generate 3D position of corners of the chessboard
  /// Right-handed coordinate , & origin at the center of the chessboard
  /// Chessboard plane at z=1.
  /// NOTICE: order of detected chessboard corners in OpenCV is inconsistent
  ///
  /// @param  worldpoints[out]  list of 3D positions of corners of the chessboard
  /// @param  iW[in]  width of the chessboard (number of inner corners)
  /// @param  iH[in]  height of the chessboard
  /// @param  fpattern.cell_unit[in]  pattern.cell_unit of each cell of the
  /// chessboard (in milimeter)
  void GenWorldpoints();

  Type type;
  cv::Size size;
  float cell_unit;

  static Type get_type(std::string pattern_t) {
    if (pattern_t == "CHECKER_BOARD" || pattern_t == "0") {
      return Type::CHECKER_BOARD;
    } else if (pattern_t == "DOT_BOARD" || pattern_t == "1") {
      return Type::DOT_BOARD;
    } else if (pattern_t == "ASYM_DOT_BOARD" || pattern_t == "2") {
      return Type::ASYM_DOT_BOARD;
    }

    printf("Only support these pattern types: CHECKER_BOARD, DOT_BOARD, ASYM_DOT_BOARD \n");
    return Type::UNDEFINE;
  }

  std::vector<Eigen::Vector3d> worldpoints; // world coordinates center of pattern centers
  Eigen::Vector4d plane; // world coordinates plane equation of the target
};


/// Can track checkerboard, symmetric/asymmetric dots pattern
template <typename CameraModel>
bool TrackCalibPattern(const CameraModel &cam, const cv::Mat &gray,
                              cv::Mat &color_img,
                              CenterLocPairVec& out_meas,
                              slick::SE3d &pose, const CalibPattern& pattern);

/// Calculate camera parameter based on captured measurements(3D world position
/// & 2D image point)
/// @param  measurementss[in] list of MeasurementSet-s
/// @param  vPoses[in]  list of camera poses
/// @param  cameraModel[in]  camera model
template <class CameraModel>
int RunCalibration(std::vector<CenterLocPairVec> &measurements,
                          std::vector<slick::SE3d> poses, CameraModel &cam,
                          double &reprojection_error);
} // namespace clams
