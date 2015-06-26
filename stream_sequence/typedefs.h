#pragma once
#include <Eigen/Dense>
#include <memory>
#include <opencv2/core/core.hpp>

#define MAX_MULT 1.3
#define MIN_MULT 0.7

namespace clams {
  using DepthMat = cv::Mat_<uint16_t>; // Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic>;  
  using DepthMatPtr = std::shared_ptr<DepthMat>;
  using DepthMatConstPtr = std::shared_ptr<const DepthMat>;
}
