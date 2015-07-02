#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

#include "clams/cam_calib.h"
#include "clams/slam_map.h"
#include "clams/serialization/serialization.h"

namespace bfs = boost::filesystem;

int main(int argc, char **argv) {
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  std::string cam_file;
  std::string rec_file;
  std::string traj_file;

  std::string slammap_file;
  std::string cloud_file;
  float max_range;
  float resolution;
  unsigned skip_poses;
  opts_desc.add_options()("help,h", "produce help message")
  ("cam_file", bpo::value(&cam_file)->required(), "Camera config file")
  ("rec", bpo::value(&rec_file)->required(), "Recording file")
  ("traj_file", bpo::value(&traj_file)->required(), "Freiburg trajectory file")
  ("slammap_file", bpo::value(&slammap_file)->required(), "StreamSequence")
  ("pointcloud", bpo::value(&cloud_file)->required(),
      "Where to save the output pointcloud.")
  ("max_range", bpo::value(&max_range)->default_value(MAX_RANGE_MAP),
      "Maximum range to use when building the map from the given trajectory, "
      "in meters.")
  ("resolution", bpo::value(&resolution)->default_value(0.01),
      "Resolution of the voxel grid used for filtering.")
  ("skip_poses", bpo::value(&skip_poses)->default_value(30),
      "Number of poses to skip");

  p.add("rec", 1);
  p.add("traj_file", 1);
  p.add("slammap_file", 1);
  p.add("pointcloud", 1);
  p.add("skip_poses", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv)
                 .options(opts_desc)
                 .positional(p)
                 .run(),
             opts);
  bool badargs = false;
  try {
    bpo::notify(opts);
  } catch (...) {
    badargs = true;
  }

  if (opts.count("help") || badargs) {
    std::cout << "Usage: " << argv[0] << " [OPTS] REC TRAJ SLAMMAP CLOUD" << std::endl;
    std::cout << std::endl;
    std::cout << opts_desc << std::endl;
    return 1;
  }

  std::array<double, 9> cam{640, 480, 525, 525, 319.5, 239.5, 0, 0, 0};
  if (!cam_file.empty()) {
    clams::ReadCamConfig(cam_file, cam);
  }
  printf("Camera parameters (width, height, fx, fy, cx, cy, k1, k2, k3):\n");
  for (int i = 0; i < 9; ++i)
    printf("%f, ", cam[i]);
  printf("\n");

  traj_file = (traj_file.length() > 3) ? traj_file : "";

  clams::SlamMap slammap;
  if (bfs::exists(slammap_file)) {
    printf("Load slammap from (%s)\n", slammap_file.c_str());
    slammap.Load(slammap_file);
  } else {
    slammap.working_path(bfs::path(slammap_file).parent_path().string());
    slammap.LoadTrajectoryAndRecording(traj_file, rec_file, cam, skip_poses);
    clams::SerializeToFile(slammap_file, slammap);
    std::cout << "Saved slam map to " << slammap_file << std::endl;
  }


  std::cout << "Building map from " << std::endl;
  std::cout << "  " << slammap_file << std::endl;
  
  if (!traj_file.empty()) {
    clams::Cloud::Ptr map = slammap.GeneratePointcloud(max_range, resolution);
    
    std::cout << "Saving to " << cloud_file << std::endl;
    pcl::io::savePCDFileBinary(cloud_file, *map);
  }
}
