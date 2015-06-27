#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include "stream_sequence/stream_sequence.h"
#include "clams/serialization/serialization.h"
#include "clams/trajectory_visualizer.h"

int main(int argc, char **argv) {
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  std::string sseq_path;
  std::string traj_path;
  std::string map_path;
  double resolution;
  double max_range;
  opts_desc.add_options()("help,h", "produce help message")(
      "sseq", bpo::value(&sseq_path)->required(),
      "StreamSequence, i.e. asus data.")(
      "traj", bpo::value(&traj_path)->required(),
      "Trajectory from slam in CLAMS format.")(
      "map", bpo::value(&map_path)->required(),
      "Where to save the output map.")(
      "resolution", bpo::value(&resolution)->default_value(0.01),
      "Resolution of the voxel grid used for filtering.")(
      "max-range", bpo::value(&max_range)->default_value(MAX_RANGE_MAP),
      "Maximum range to use when building the map from the given trajectory, "
      "in meters.");

  p.add("sseq", 1);
  p.add("traj", 1);
  p.add("map", 1);

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
    std::cout << "Usage: " << bfs::basename(argv[0]) << " [OPTS] SSEQ TRAJ MAP"
         << std::endl;
    std::cout << "  A map will be generated from SSEQ and TRAJ.  It will be saved "
            "to MAP." << std::endl;
    std::cout << std::endl;
    std::cout << opts_desc << std::endl;
    return 1;
  }

  clams::Trajectory traj;
  clams::SerializeFromFile(traj_path, traj);

  clams::StreamSequence::Ptr sseq(new clams::StreamSequence());
  sseq->load(sseq_path);

  std::cout << "Building map from " << std::endl;
  std::cout << "  " << sseq_path << std::endl;
  std::cout << "  " << traj_path << std::endl;
  std::cout << "Saving to " << map_path << std::endl;
 
  clams::Cloud::Ptr map = clams::SlamCalibrator::buildMap(sseq, traj, max_range, resolution);
  pcl::io::savePCDFileBinary(map_path, *map);
}
