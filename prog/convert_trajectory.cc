#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "stream_sequence/stream_sequence.h"
#include "clams/trajectory.h"
#include "clams/serialization/serialization.h"

namespace bfs = boost::filesystem;

int main(int argc, char **argv) {
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  std::string rec_path;
  std::string sseq_path;
  std::string src;
  std::string dst;
  opts_desc.add_options()("help,h", "produce help message")(
      "rec", bpo::value(&sseq_path)->required(), "Recording file")(
      "sseq", bpo::value(&sseq_path)->required(), "StreamSequence")(
      "src", bpo::value(&src)->required(), "Freiburg trajectory file")(
      "dst", bpo::value(&dst)->required(), "Clams trajectory file");

  p.add("rec", 1);
  p.add("sseq", 1);
  p.add("src", 1);
  p.add("dst", 1);

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
    std::cout << "Usage: " << argv[0] << " [OPTS] SSEQ SRC DST" << std::endl;
    std::cout << std::endl;
    std::cout << opts_desc << std::endl;
    return 1;
  }

  clams::StreamSequenceBase::Ptr sseq =
      clams::StreamSequence::LoadExternalRecording(rec_path);
  clams::SerializeToFile(sseq_path, *sseq);

  clams::Trajectory traj;
  traj.resize(sseq->size());

  std::ifstream frei;
  frei.open(src.c_str());
  while (true) {
    double timestamp, tx, ty, tz, qx, qy, qz, qw;
    frei >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    if (frei.eof())
      break;

    double dt;
    size_t idx = sseq->seek(timestamp, &dt);
    assert(dt < 1e-6);

    Eigen::Quaternion<double> rotation(qw, qx, qy, qz);
    Eigen::Translation<double, 3> translation(tx, ty, tz);
    Eigen::Affine3d transform = translation * rotation;
    traj.set(idx, transform);
  }
  clams::SerializeToFile(dst, traj);
  std::cout << "Saved to " << dst << std::endl;
}
