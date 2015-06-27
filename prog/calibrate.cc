#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "stream_sequence/stream_sequence.h"
#include "clams/slam_calibrator.h"
#include "clams/serialization/serialization.h"

int main(int argc, char **argv) {
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  std::string workspace;
  opts_desc.add_options()
  ("help,h", "produce help message")
  ("workspace", bpo::value(&workspace)->default_value("."), "CLAMS WORKSPACE.")
  ("increment", bpo::value<int>()->default_value(1),
                          "Use every kth frame for calibration.");

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

  clams::SlamCalibratorPtr calibrator(new clams::SlamCalibrator());

  // -- Get names of sequences that have corresponding results.
  std::vector<std::string> haveclams_folders;
  bfs::directory_iterator it(workspace), eod;
  BOOST_FOREACH (const bfs::path &p, std::make_pair(it, eod)) {
    std::string path = workspace + "/" + p.leaf().string() + "/clams";
    if (bfs::exists(path) && bfs::exists(path + "/" + "clams-traj.bin"))
      haveclams_folders.push_back(p.leaf().string());
  }
  std::sort(haveclams_folders.begin(), haveclams_folders.end());

  if (haveclams_folders.empty()) {
    printf("NO input directory found from \"%s\"\n", workspace);
    printf("Are you running this program from with correct workspace input?\n");
    return 1;
  }

  // -- Construct sseqs with corresponding trajectories.
  for (size_t i = 0; i < haveclams_folders.size(); ++i) {
    std::string prefix = workspace + "/" + haveclams_folders[i] + "/clams";
    std::string sseq_path =  prefix + "/clams-sseq.bin";
    std::string traj_path = prefix + "/clams-traj.bin";
    std::string map_path = prefix + "/clams-map.pcd";

    std::cout << "Sequence " << i << std::endl;
    std::cout << "  StreamSequence:" << sseq_path << std::endl;
    std::cout << "  Trajectory: " << traj_path << std::endl;
    std::cout << "  Map: " << map_path << std::endl;

    clams::StreamSequence::Ptr sseq(new clams::StreamSequence());
    sseq->load(sseq_path);
    calibrator->sseqs().push_back(sseq);

    clams::Trajectory traj;
    clams::SerializeFromFile(traj_path, traj);
    calibrator->trajectories().push_back(traj);

    clams::Cloud::Ptr map(new clams::Cloud);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(map_path, *map);
    calibrator->maps().push_back(map);
  }

  // -- Run the calibrator.
  calibrator->proj() = calibrator->sseqs().front()->GetFrameProjector();

  std::cout << std::endl;
  if (opts.count("increment")) {
    calibrator->increment() = opts["increment"].as<int>();
    std::cout << "Using frame increment of " << calibrator->increment()
              << std::endl;
  }
  std::cout << std::endl;

  clams::DiscreteDepthDistortionModel model = calibrator->calibrate();
  std::string output_path = workspace + "/clams/distortion_model.bin";
  model.save(output_path);
  std::cout << "Saved distortion model to " << output_path << std::endl;

  // std::string vis_dir = output_path + "-visualization";
  // model.visualize(vis_dir);
  // std::cout << "Saved visualization of distortion model to " << vis_dir
  //           << std::endl;
}
