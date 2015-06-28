#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include "clams/slam_calibrator.h"
#include "clams/io/pointcloud_x3d.h"

int main(int argc, char **argv) {
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  std::string model_path;
  std::string color_file, depth_file;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("intrinsics", bpo::value(&model_path)->required(), "Distortion model.")
    ("color_file", bpo::value(&color_file)->required(), "Color image file.")
    ("depth_file", bpo::value(&depth_file)->required(), "Depth image file.");

  p.add("intrinsics", 1);
  p.add("color_file", 1);
  p.add("depth_file", 1);

  bpo::variables_map opts;
  bool badargs = false;
  try {
    bpo::store(bpo::command_line_parser(argc, argv)
                   .options(opts_desc)
                   .positional(p)
                   .run(),
               opts);
    bpo::notify(opts);
  } catch (...) {
    badargs = true;
  }
  if (opts.count("help") || badargs) {
    std::cout << "Usage: " << bfs::basename(argv[0]) << " MODEL COLOR DEPTH" << std::endl;
    std::cout << "  Saves output visualizations to *.x3d"
              << std::endl;
    std::cout << std::endl;
    std::cout << opts_desc << std::endl;
    return 1;
  }

  clams::DiscreteDepthDistortionModel model;
  model.load(model_path);

  auto cimg = cv::imread(color_file, -1);
  if (cimg.empty()) {
    printf("Color image (%s) empty\n", color_file.c_str());
    return 1;
  }
  clams::DepthMat dimg = cv::imread(depth_file, -1);
  if (dimg.empty()) {
    printf("Depth image (%s) empty\n", depth_file.c_str());
    return 1;
  }

  Eigen::Matrix3d inv_proj;
  inv_proj << 525, 0, 320,
                    0, 525, 240,
                    0, 0, 1;

  inv_proj = inv_proj.inverse().eval();

  clams::io::x3d::write_pointcloud(cimg, dimg, inv_proj, color_file + "-original.x3d");

  model.undistort(dimg);

  clams::io::x3d::write_pointcloud(cimg, dimg, inv_proj,  color_file + "-undistorted.x3d");

}
