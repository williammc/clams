#pragma once
#include "clams/io/pointcloud_x3d.h"
#include "clams/frame_projector.h"

namespace clams {
namespace io {
namespace x3d {

inline void write_frame(const Frame& frame, const FrameProjector& proj, std::string fn = "") {
  fn = (fn.empty()) ? "clams-frame-" + std::to_string(frame.timestamp) + ".x3d"
                    : fn;
  std::ofstream ofs(fn);

  ofs.setf(std::ios::fixed, std::ios::floatfield);
  ofs.precision(30);

  // write header
  std::stringstream ss;
  ss << frame.measurements.size() << " measurements";
 clams::io::x3d::write_header("Frame", "Frame Exporter", ss.str(), ofs);

  // world axises
  clams::io::x3d::write_axis(0.1, ofs);

  /// write pointcloud
  Cloud::Ptr cloud(new Cloud);
  proj.FrameToCloud(frame, cloud.get(), std::numeric_limits<float>::max());
  clams::io::x3d::write_pointcloud(*cloud, ofs);

  // write measurements
  if (!frame.measurements.empty()) {
    for (auto t : frame.measurements) {
      auto p = frame.target2cam * t.first;
      ofs << "      <Transform translation='" << p.head<3>().transpose() << "'>\n";
      ofs << "        <Shape>\n"
          << "          <Sphere radius='0.005'/>\n"
          << "            <Appearance>\n"
          << "              <Material diffuseColor='0 0 1' shininess='0.8' specularColor='.8 .8 .8'/>\n"
          << "            </Appearance>\n"
          << "        </Shape>\n"
          << "      </Transform>\n";
    }
  }

  // write footer
  clams::io::x3d::write_footer(ofs);
}

} // namespace x3d
} // namespace io
} // namespace clams