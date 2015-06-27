#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "stream_sequence/stream_sequence.h"
#include "clams/trajectory_visualizer.h"
#include "clams/serialization/serialization.h"

int main(int argc, char **argv) {
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  std::string sseq_path;
  std::string traj_path;
  std::string map_path;
  opts_desc.add_options()("help,h", "produce help message")(
      "sseq", bpo::value(&sseq_path)->required(),
      "StreamSequence, i.e. asus data.")(
      "traj", bpo::value(&traj_path)->required(),
      "Trajectory from slam in CLAMS format.")(
      "map", bpo::value(&map_path)->required(), "Cached map.");

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
    std::cout << std::endl;
    std::cout << opts_desc << std::endl;
    return 1;
  }

  clams::Trajectory traj;
  clams::SerializeFromFile(traj_path, traj);

  clams::StreamSequenceBase::Ptr sseq(new clams::StreamSequence());
  sseq->load(sseq_path);

  clams::Cloud::Ptr map(new clams::Cloud);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>(map_path, *map);

  clams::TrajectoryVisualizer tv(sseq, traj, map, sseq_path);
  tv.run();
}
