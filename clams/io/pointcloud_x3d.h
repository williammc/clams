#pragma once
#include <iostream>
#include <array>
#include <Eigen/Core>
#include <opencv2/highgui/highgui.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "clams/common/clams_defer.h"
#include "clams/io/common_x3d.h"

namespace bfs = boost::filesystem;

namespace clams {
namespace io {
namespace x3d {

/// write pointcloud given 3D @points
template<typename T1, typename T2>
inline void write_points(const std::vector<std::array<T1, 3>>& points,
                         const std::array<T2, 3> &color,
                         std::ofstream &ofs) {
  ofs << "      <Shape> <!-- pointcloud -->\n"
      << "        <Appearance>\n"
      << "          <Material ambientIntensity='0.5' emissiveColor='" << color[0] << " " << color[1] << " "<< color[2] << " " << "'/>\n"
      << "        </Appearance>\n"
      << "        <PointSet>\n";

  // 3D points
  ofs << "          <Coordinate point='";
  int count = 0;
  for (auto pt : points) {
    ofs << pt[0] << " " << pt[1] << " " << pt[2] << " ";
    if (count++ == 10) {
      count = 0;
      ofs << "\n";
    }
  }
  ofs << "'/>\n";

  ofs << "        </PointSet>\n";
  ofs << "      </Shape> <!-- pointcloud -->\n";
}

/// write pointcloud given @incides of pixels of a color-depth image pair
inline void write_points(const std::vector<int>& indices,
                         const Eigen::Matrix3d& inv_proj,
                         const cv::Mat &color_img,
                         const cv::Mat &depthfloat_img,
                         std::ostream &ofs,
                         const Eigen::Vector3d &color,
                         bool use_pixel_color) {
  auto  pixel_loc = [](const int index, const int w) {
    cv::Point p;
    p.x = index % w;
    p.y = index / w;
    return p;
  };

  ofs << "      <Shape> <!-- pointcloud -->\n"
      << "        <Appearance>\n"
      << "          <Material ambientIntensity='0.5' ";
  if (!use_pixel_color)
    ofs << "emissiveColor='" << color.transpose() << "'";
  ofs << "          />\n"
      << "        </Appearance>\n";
  ofs << "        <PointSet>\n";

  std::cout << "inv_proj\n" << inv_proj << "\n";

  // 3D points
  ofs << "          <Coordinate point='";
  int count = 0;
  for (auto &id : indices) {
    auto const loc = pixel_loc(id, depthfloat_img.cols);
    const float d = depthfloat_img.at<float>(loc);
    if (d == 0) continue;
    const Eigen::Vector3d point = inv_proj * Eigen::Vector3d(loc.x, loc.y, 1.0) * d;
    ofs << point.transpose() << " ";
    if (count++ == 10) {
      count = 0;
      ofs << "\n";
    }
  }
  ofs << "'/>\n";

  // colors
  if (use_pixel_color) {
    ofs << "          <Color color='";
    for (auto &id : indices) {
    auto const loc = pixel_loc(id, depthfloat_img.cols);
      const float d = depthfloat_img.at<float>(loc);
      if (d == 0) continue;
      const auto pix = color_img.at<cv::Vec3b>(loc);
      ofs << float(pix[2])/255 << " " << float(pix[1])/255 << " " << float(pix[0])/255 << " ";
      if (count++ == 10) {
        count = 0;
        ofs << "\n";
      }
    }
    ofs << "'/>\n";
  }

  ofs << "        </PointSet>\n";
  ofs << "      </Shape> <!-- pointcloud -->\n";
}

inline void write_pointcloud(cv::Mat bgr, cv::Mat depth, 
  const Eigen::Matrix3d& inv_proj, std::ostream& os, int skip_pix = 7) {

    cv::Mat depthfloat_img;
    depth.convertTo(depthfloat_img, CV_32F);
    depthfloat_img = depthfloat_img*0.001;

    std::vector<int> indices;
    indices.reserve(bgr.rows*bgr.cols/skip_pix);
    for (int id = 0; id < bgr.rows*bgr.cols; id += skip_pix)
        indices.push_back(id);

    write_points(indices, inv_proj, bgr, depthfloat_img, os,
                 Eigen::Vector3d(1, 1, 1), true);
}

inline void write_pointcloud(cv::Mat bgr, cv::Mat depth, const Eigen::Matrix3d& inv_proj,
  std::string file) {
  auto working_path = bfs::path(file).parent_path().string();
  if (!bfs::exists(working_path))
    bfs::create_directory(working_path);

  auto const filename = bfs::path(file).filename().string();

  const bfs::path pre_path = bfs::current_path();
  auto d1 = defer([&pre_path] () {bfs::current_path(pre_path);});

  bfs::current_path(working_path);  // cd to working path

  std::ofstream ofs(file);

  ofs.setf(std::ios::fixed, std::ios::floatfield);
  ofs.precision(30);

  // write header
  write_header("Point cloud", "Pointcloud Exporter", "", ofs);

  /// write pointcloud
  write_pointcloud(bgr, depth, inv_proj, ofs);

  // write footer
  write_footer(ofs);

}

template<typename PCLCloud> 
inline void write_pointcloud(const PCLCloud& cloud, std::ostream& ofs) {

  ofs << "      <Shape> <!-- pointcloud -->\n"
      << "        <Appearance>\n"
      << "          <Material ambientIntensity='0.5' "
      << "          />\n"
      << "        </Appearance>\n";
  ofs << "        <PointSet>\n";

  // 3D points
  ofs << "          <Coordinate point='";
  int count = 0;
  for (size_t i = 0; i < cloud.size(); ++i) {
    ofs << cloud[i].x << " " << cloud[i].y << " " << cloud[i].z << " ";
    if (count++ == 10) {
      count = 0;
      ofs << "\n";
    }
  }
  ofs << "'/>\n";

  // colors
  {
    ofs << "          <Color color='";
    for (size_t i = 0; i < cloud.size(); ++i) {
      ofs << float(cloud[i].r) / 255 << " " 
          << float(cloud[i].g) / 255 << " "
          << float(cloud[i].b) / 255 << " ";
      if (count++ == 10) {
        count = 0;
        ofs << "\n";
      }
    }
    ofs << "'/>\n";
  }

  ofs << "        </PointSet>\n";
  ofs << "      </Shape> <!-- pointcloud -->\n";
}

} // namespace x3d
} // namespace io
} // namespace clams