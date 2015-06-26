#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace clams {
  using Point = pcl::PointXYZRGB;
  using Cloud = pcl::PointCloud<Point>;
}