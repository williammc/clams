#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>

#define MAX_MULT 1.3
#define MIN_MULT 0.7

namespace clams {
  using DepthMat = cv::Mat_<uint16_t>;
  using Point = pcl::PointXYZRGB;
  using Cloud = pcl::PointCloud<Point>;
}
