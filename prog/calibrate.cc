#include <fstream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "clams/cam_calib.h"
#include "clams/slam_calibrator.h"
#include "clams/serialization/serialization.h"

int main(int argc, char **argv) {
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  std::string workspace;
  std::vector<float> calib_params; // type, width, height, cell_unit
  opts_desc.add_options()
  ("help,h", "produce help message")
  ("workspace", bpo::value(&workspace)->default_value("."), "CLAMS WORKSPACE.")
  ("increment", bpo::value<int>()->default_value(1),
                          "Use every kth frame for calibration.")
  ("calib_params", bpo::value(&calib_params)->required(),
                  "Calibration target params (type, width, height, cell_unit.\n"
                    "type (0:CHECKER_BOARD, 1:DOT_BOARD, 2:ASYM_DOT_BOARD)");

  p.add("workspace", 1);

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
    std::cout << "Usage: " << bfs::basename(argv[0])
              << " [ OPTS ] workspace " << std::endl;
    std::cout << "  This program will calibrate using all slam results in "
                 "workspace/." << std::endl;
    std::cout << std::endl;
    std::cout << opts_desc << std::endl;
    return 1;
  }

  clams::FrameProjector proj;

  clams::SlamCalibratorPtr calibrator(new clams::SlamCalibrator(proj));

  // -- Get names of sequences that have corresponding results.
  std::vector<std::string> haveclams_folders;
  bfs::directory_iterator it(workspace), eod;
  BOOST_FOREACH (const bfs::path &p, std::make_pair(it, eod)) {
    std::string path = workspace + "/" + p.leaf().string() + "/clams";
    if (bfs::exists(path) && bfs::exists(path + "/" + "clams-slammap.bin"))
      haveclams_folders.push_back(p.leaf().string());
  }
  std::sort(haveclams_folders.begin(), haveclams_folders.end());

  if (haveclams_folders.empty()) {
    printf("NO input directory found from \"%s\"\n", workspace);
    printf("Are you running this program from with correct workspace input?\n");
    return 1;
  }

  // -- Construct slammaps with corresponding trajectories.
  for (size_t i = 0; i < haveclams_folders.size(); ++i) {
    std::string prefix = workspace + "/" + haveclams_folders[i] + "/clams";
    std::string slammap_path =  prefix + "/clams-slammap.bin";

    printf("Sequence %u\n", i);
    printf("Load SlamMap from (%s)\n", slammap_path.c_str());

    clams::SlamMap slammap;
    slammap.Load(slammap_path);
    calibrator->slam_maps().push_back(slammap);

    std::string cloud_path = prefix + "/clams-cloud.pcd";
    if (bfs::exists(cloud_path)) {
      printf("Load map pointcloud from (%s)\n", cloud_path.c_str());
      clams::Cloud::Ptr map(new clams::Cloud);
      pcl::io::loadPCDFile<pcl::PointXYZRGB>(cloud_path, *map);
      calibrator->slam_maps().back().pointcloud(map);
    }
  }

  // -- Run the calibrator.
  calibrator->proj() = calibrator->slam_maps().front().frame_projector();

  if (calib_params.size() == 4) {
    auto t = clams::CalibPattern::get_type(std::to_string(calib_params[0]));
    if (t != clams::CalibPattern::Type::UNDEFINE) {
      calibrator->pattern() = clams::CalibPattern(
          t, cv::Size(calib_params[1], calib_params[2]), calib_params[3]);
    }
  }
  std::cout << std::endl;
  if (opts.count("increment")) {
    calibrator->increment() = opts["increment"].as<int>();
    std::cout << "Using frame increment of " << calibrator->increment()
              << std::endl;
  }
  std::cout << std::endl;

  if (calibrator->Calibrate()) {
    clams::DiscreteDepthDistortionModel model = calibrator->depth_model();
    std::string output_path = workspace + "/clams/distortion_model.bin";
    printf("Save distortion model to %s\n", output_path.c_str());
    model.save(output_path);

    std::array<float, 9> cam{640, 480, 525, 525, 319.5, 239.5, 0, 0, 0};
    auto params = calibrator->camera_model().parameters();
    cam[0] = calibrator->camera_model().ImageSize()[0];
    cam[1] = calibrator->camera_model().ImageSize()[1];
    for (int i = 0; i < 6; ++i)
      cam[i+2] = params[i];

    output_path = workspace + "/clams/camera.txt";
    printf("Save camera model to %s\n", output_path.c_str());
    std::ofstream ofs(output_path);
    ofs << "# camera parameters (width, height, fx, fy, cx, cy, k1, k2, k3)\n";
    for (int i = 0; i < 9; ++i)
      ofs << cam[i] << " ";
    ofs << "\n";

  } else {
    printf("No luck!!! calibration failed!!!\n");
  }
}
