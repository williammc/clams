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
  opts_desc.add_options()("help,h", "produce help message")
  ("workspace", bpo::value(&workspace)->default_value("."), "CLAMS workspace.")
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
              << " [ OPTS ] CLAMS_WORKSPACE " << std::endl;
    std::cout << "  This program will calibrate using all slam results in "
                 "CLAMS_WORKSPACE/." << std::endl;
    std::cout << std::endl;
    std::cout << opts_desc << std::endl;
    return 1;
  }

  // -- Check for existence of CLAMS_WORKSPACE/slam_results.
  std::string sequences_path = workspace + "/sequences";
  std::string results_path = workspace + "/slam_results";
  if (!bfs::exists(results_path)) {
    std::cout << "Expected results path \"" << results_path
              << "\" does not exist." << std::endl;
    std::cout << "Are you running this program from within a CLAMS workspace?"
              << std::endl;
    std::cout << "Have you run \"rosrun clams slam\" yet?" << std::endl;
    return 0;
  }

  clams::SlamCalibratorPtr calibrator(new clams::SlamCalibrator());

  // -- Get names of sequences that have corresponding results.
  std::vector<std::string> sseq_names;
  bfs::directory_iterator it(results_path), eod;
  BOOST_FOREACH (const bfs::path &p, std::make_pair(it, eod)) {
    std::string path = results_path + "/" + p.leaf().string() + "/trajectory";
    if (bfs::exists(path))
      sseq_names.push_back(p.leaf().string());
  }
  sort(sseq_names.begin(), sseq_names.end());

  // -- Construct sseqs with corresponding trajectories.
  std::vector<clams::StreamSequenceBase::ConstPtr> sseqs;
  std::vector<clams::Trajectory> trajs;
  std::vector<clams::Cloud::ConstPtr> maps;
  for (size_t i = 0; i < sseq_names.size(); ++i) {
    std::string sseq_path = sequences_path + "/" + sseq_names[i];
    std::string traj_path = results_path + "/" + sseq_names[i] + "/trajectory";
    std::string map_path =
        results_path + "/" + sseq_names[i] + "/clams-calib.pcd";

    std::cout << "Sequence " << i << std::endl;
    std::cout << "  StreamSequence:" << sseq_path << std::endl;
    std::cout << "  Trajectory: " << traj_path << std::endl;
    std::cout << "  Map: " << map_path << std::endl;

    clams::StreamSequence::Ptr sseq(new clams::StreamSequence());
    clams::SerializeFromFile(sseq_path, *sseq);
    calibrator->sseqs().push_back(sseq);

    clams::Trajectory traj;
    clams::SerializeFromFile(traj_path, traj);
    calibrator->trajectories().push_back(traj);

    clams::Cloud::Ptr map(new clams::Cloud);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(map_path, *map);
    calibrator->maps().push_back(map);
  }

  // -- Run the calibrator.
  calibrator->proj() = sseqs.front()->GetFrameProjector();

  std::cout << std::endl;
  if (opts.count("increment")) {
    calibrator->increment() = opts["increment"].as<int>();
    std::cout << "Using frame increment of " << calibrator->increment()
              << std::endl;
  }
  std::cout << std::endl;

  clams::DiscreteDepthDistortionModel model = calibrator->calibrate();
  std::string output_path = workspace + "/distortion_model";
  model.save(output_path);
  std::cout << "Saved distortion model to " << output_path << std::endl;

  std::string vis_dir = output_path + "-visualization";
  model.visualize(vis_dir);
  std::cout << "Saved visualization of distortion model to " << vis_dir
            << std::endl;

  return 0;
}
