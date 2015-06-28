// Copyright 2015 The Syntactic Modeling Authors. All rights reserved.
#pragma once
#include <fstream>
#include <ctime>
#include <iomanip>

namespace clams {
namespace io {
namespace x3d {
inline void write_header(std::string title, std::string creator,
                                std::string description, std::ofstream &ofs) {
  std::time_t t = std::time(nullptr);

  // clang-format off
  ofs << "<?xml version='1.0' encoding='UTF-8'?>\n"
      << "<!DOCTYPE X3D PUBLIC 'ISO//Web3D//DTD X3D 3.0//EN' "
         "'http://www.web3d.org/specifications/x3d-3.0.dtd'>\n"
      << "<X3D profile='Interchange' version='3.0'  "
         "xmlns:xsd='http://www.w3.org/2001/XMLSchema-instance' "
         "xsd:noNamespaceSchemaLocation =' "
         "http://www.web3d.org/specifications/x3d-3.0.xsd '>\n"
      << "  <Head>\n"
      << "    <meta name='title' content='" << title << "'/>\n"
      << "    <meta name='creator' content='" << creator << "'/>\n"
      << "    <meta name='reference' content=''/>\n"
      << "    <meta name='created' content='"
      << std::put_time(std::localtime(&t), "%c %Z") << "'/>\n"
      << "    <meta name='description' content='" << description << "'/>\n"
      << "  </Head>\n"
      << "  <Scene>\n"
      << "    <Viewpoint description='Vision View' orientation='1 0 0 3.14159' "
         "position='0 0 0'/>\n"
      << "    <Group>\n";
  // clang-format on
}

inline void write_footer(std::ofstream &ofs) {
  ofs << "    </Group>\n"
      << "  </Scene>\n"
      << "</X3D>\n";
}

inline void write_axis(double scale, std::ofstream &ofs) {
  // clang-format off
  ofs << "      <Transform scale='" << scale << " " << scale << " " << scale << "'>\n"
      << "        <Inline DEF='CoordinateAxes' url='\"CoordinateAxes.x3d\" \n"
      << " \"http://X3dGraphics.com/examples/X3dForWebAuthors/"
         "Chapter03-Grouping/CoordinateAxes.x3d\" \n"
      << " \"https://savage.nps.edu/Savage/Tools/Authoring/"
         "CoordinateAxes.x3d\"'/>\n"
      << "      </Transform>\n";
  // clang-format on
}

inline void write_common_properties(std::ofstream &ofs) {
  // clang-format off
  ofs << "      <!-- Common Appearance Properties -->\n"
      << "      <Shape>\n"
      << "        <Appearance>\n"
      << "          <LineProperties DEF='Width15_Line' linetype='1' "
         "linewidthScaleFactor='15' applied='true' "
         "containerField='lineProperties15'>\n"
      << "          </LineProperties>\n"
      << "          <LineProperties DEF='Width5_Line' linetype='1' "
         "linewidthScaleFactor='5' applied='true' "
         "containerField='lineProperties5'>\n"
      << "          </LineProperties>\n"
      << "          <LineProperties DEF='Width3_Line' linetype='1' "
         "linewidthScaleFactor='3' applied='true' "
         "containerField='lineProperties3'>\n"
      << "          </LineProperties>\n"
      << "          <LineProperties DEF='Width2_Line' linetype='1' "
         "linewidthScaleFactor='2' applied='true' "
         "containerField='lineProperties2'>\n"
      << "          </LineProperties>\n"
      << "        </Appearance>\n"
      << "      </Shape>\n";
  // clang-format on
}


/// write line segment
template<typename LineSegment>
inline void write_linesegment(const LineSegment& line,
                              const Eigen::Vector3d &color,
                              std::ofstream &ofs) {
  // clang-format off
  ofs << "      <Shape> <!-- Line segment -->\n"
      << "        <Appearance>\n"
      << "          <Material ambientIntensity='1.0' emissiveColor='" << color.transpose() << "'  />\n"
      << "          <LineProperties USE='Width3_Line'> </LineProperties>\n"
      << "        </Appearance>\n"
      << "        <LineSet vertexCount='2'>\n"
      << "          <Coordinate point='" << line.point1().transpose() << " " << line.point2().transpose() << "'/>\n"
      << "        </LineSet>\n"
      << "      </Shape> <!-- Line segment -->\n";
  // clang-format on
}

} // namespace x3d
} // namespace io
} // namespace clams