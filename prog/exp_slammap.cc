#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

#include "clams/io/slam_x3d.h"
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
  opts_desc.add_options()("help,h", "produce help message")
  ("slammap_file", bpo::value(&slammap_file)->required(), "StreamSequence")
  ("pointcloud", bpo::value(&cloud_file)->required(),
      "Where to save the output pointcloud.");

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

  if (opts.count("help") || badargs) {
    std::cout << "Usage: " << argv[0] << " [OPTS] REC TRAJ SLAMMAP CLOUD" << std::endl;
    std::cout << std::endl;
    std::cout << opts_desc << std::endl;
    return 1;
  }

  printf("Deserialize slam map from :%s\n", slammap_file.c_str());
  clams::SlamMap slammap;
  slammap.Load(slammap_file);

  printf("Load cloud from :%s\n", cloud_file.c_str());
  clams::Cloud::Ptr cloud(new clams::Cloud);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>(cloud_file, *cloud);
  slammap.pointcloud(cloud);

  printf("Write slam map to :%s\n", (slammap_file + ".x3d").c_str());
  clams::io::x3d::write_slammap(slammap, slammap_file + ".x3d");
}
