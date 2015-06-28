#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

#include "clams/slam_map.h"
#include "clams/serialization/serialization.h"

namespace bfs = boost::filesystem;

int main(int argc, char **argv) {
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  std::string rec_file;
  std::string traj_file;
  std::vector<float> cam_params;

  std::string slammap_file;
  std::string cloud_file;
  float max_range;
  float resolution;
  opts_desc.add_options()("help,h", "produce help message")
  ("rec", bpo::value(&rec_file)->required(), "Recording file")
  ("traj_file", bpo::value(&traj_file)->required(), "Freiburg trajectory file")
  
  // ("cam_params", bpo::value(&cam_params)->default_value(std::vector<float>(0)),
  //     "camera parameters.")

  ("slammap_file", bpo::value(&slammap_file)->required(), "StreamSequence")
  ("pointcloud", bpo::value(&cloud_file)->required(),
      "Where to save the output pointcloud.")
  ("max-range", bpo::value(&max_range)->default_value(MAX_RANGE_MAP),
      "Maximum range to use when building the map from the given trajectory, "
      "in meters.")
  ("resolution", bpo::value(&resolution)->default_value(0.01),
      "Resolution of the voxel grid used for filtering.");

  p.add("rec", 1);
  p.add("traj_file", 1);
  p.add("slammap_file", 1);
  p.add("pointcloud", 1);

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
  if (cam_params.size() != 6) {
    printf("Bad camera parameters input\n");
    badargs = true;
  }
  std::array<float, 6> cam;
  for (int i = 0; i < 6; ++i)
    cam[i] = cam_params[i];

  if (opts.count("help") || badargs) {
    std::cout << "Usage: " << argv[0] << " [OPTS] REC TRAJ SLAMMAP CLOUD" << std::endl;
    std::cout << std::endl;
    std::cout << opts_desc << std::endl;
    return 1;
  }

  clams::SlamMap slammap;
  slammap.working_path(bfs::path(slammap_file).parent_path().string());
  slammap.LoadRecordingAndTrajectory(rec_file, traj_file, cam);

  clams::SerializeToFile(slammap_file, slammap);
  std::cout << "Saved slam map to " << slammap_file << std::endl;


  std::cout << "Building map from " << std::endl;
  std::cout << "  " << slammap_file << std::endl;
  std::cout << "Saving to " << cloud_file << std::endl;
 
  clams::Cloud::Ptr map = slammap.GeneratePointcloud(max_range, resolution);
  pcl::io::savePCDFileBinary(cloud_file, *map);
}
