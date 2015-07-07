#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <clams/slam_calibrator.h>

int main(int argc, char **argv) {
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  std::string model_path;
  opts_desc.add_options()("help,h", "produce help message")(
      "intrinsics", bpo::value(&model_path)->required(), "Distortion model.");

  p.add("intrinsics", 1);

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
    std::cout << "Usage: " << bfs::basename(argv[0]) << " MODEL" << std::endl;
    std::cout << "  Saves output visualizations to MODEL-visualization/"
              << std::endl;
    std::cout << std::endl;
    std::cout << opts_desc << std::endl;
    return 1;
  }

  clams::DiscreteDepthDistortionModel model;
  model.load(model_path);
  std::string vis_dir = model_path + "-visualization";
  model.visualize(vis_dir);
  printf("Loaded visualization of distortion model from (%s)\n", vis_dir.c_str());
}
