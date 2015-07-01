#pragma once
#include <iostream>
#include <Eigen/Core>
#include "clams/io/common_x3d.h"
#include "clams/io/pointcloud_x3d.h"
#include "clams/slam_map.h"

namespace clams {
namespace io {
namespace x3d {

inline void write_trajectory(const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>& poses,
                             std::ostream& ofs, const Eigen::Vector3d& color) {
  ofs << "      <Shape> <!-- trajectory -->\n"
      << "        <Appearance>\n"
      << "          <Material ambientIntensity='0.5' " 
                            << "emissiveColor='" << color.transpose() << "'/>\n"
      << "        </Appearance>\n";

  ofs << "        <IndexedLineSet coordIndex='";
  int id = 0;
  int count = 0;
  for (auto& pose : poses) {
    ofs << id++ << " ";
    if (count++ >= 50) {
      count = 0;
      ofs << " \n";
    }
  }
  ofs << " -1'>\n";
  // 3D points
  ofs << "          <Coordinate point='";
  for (auto& pose : poses) {
    ofs << pose.translation().transpose() << " ";
    if (count++ >= 10) {
      count = 0;
      ofs << " \n";
    }
  }
  ofs << "'/>\n";
  ofs << "        </IndexedLineSet>\n"
      << "      </Shape> <!-- Keyframes trajectory -->\n";
}

inline void write_slammap(const SlamMap& slmap, std::string file) {
  auto working_path = bfs::path(file).parent_path().string();
  if (!bfs::exists(working_path))
    bfs::create_directory(working_path);

  auto const filename = bfs::path(file).filename().string();

  const bfs::path pre_path = bfs::current_path();
  auto d1 = defer([&pre_path] () {bfs::current_path(pre_path);});

  bfs::current_path(working_path);  // cd to working path

  std::ofstream slm_ofs(file);

  slm_ofs.setf(std::ios::fixed, std::ios::floatfield);
  slm_ofs.precision(30);

  // write header
  std::stringstream ss;
  ss << slmap.traj_timeposes().size() << " poses";
  write_header("Slam Map", "Slam Map Exporter", ss.str(), slm_ofs);

  // world axises
  write_axis(0.1, slm_ofs);
  
  {
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> poses;
    for (auto& t : slmap.traj_timeposes()) {
      // std::cout << "pose:\n" << t.second.matrix() << std::endl;
      poses.push_back(t.second);
    }

    if (poses.size() > 1)
      write_trajectory(poses, slm_ofs, Eigen::Vector3d(0, 1, 0));
  }

  /// write pointcloud
  auto cloud = slmap.pointcloud();
  if (cloud) {
    write_pointcloud(*cloud, slm_ofs);
  } else {
    printf("NO CLOUD\n");
  }

  // write footer
  write_footer(slm_ofs);
}


} // namespace x3d
} // namespace io
} // namespace clams