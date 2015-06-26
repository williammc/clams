#include "stream_sequence/stream_sequence.h"
#include <limits>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/foreach.hpp>

#include "clams/serialization/serialization.h"

namespace bfs = boost::filesystem;

namespace clams {

StreamSequence::StreamSequence() : max_depth_(std::numeric_limits<double>::max()) {}

StreamSequence::Ptr StreamSequence::LoadExternalRecording(std::string full_fn) {
  if (bfs::is_directory(full_fn) || !bfs::exists(full_fn)) {
    std::cout << "Filename is a path or does not exists! " << full_fn << std::endl;
    return StreamSequence::Ptr();
  }
  auto dir = bfs::path(full_fn).parent_path().string();
  auto const filename = bfs::path(full_fn).filename().string();

  StreamSequence::Ptr seg(new StreamSequence());
  seg->root_path_ = dir;

  std::ifstream ifs(full_fn);
  while (!ifs.eof()) {
    std::string cfn, dfn;
    Frame fr;
    red_recording_line(ifs, fr.timestamp, cfn, dfn);
    fr.img = cv::imread(dir + "/" + cfn, -1);
    fr.depth = cv::imread(dir + "/" + dfn, -1);
    std::string fn = "clams-frame-" + std::to_string(fr.timestamp) + ".bin";
    SerializeToFile(dir, fn, fr);
    seg->frame_names_.push_back(fn);
    seg->timestamps_.push_back(fr.timestamp);
  }

  seg->proj_ = FrameProjector();
  seg->proj_.width_ = 640;
  seg->proj_.height_ = 480;
  seg->proj_.cx_ = seg->proj_.width_ / 2 + 0.5f;
  seg->proj_.cy_ = seg->proj_.height_ / 2 + 0.5f;
}

void StreamSequence::init(const std::string &root_path) {
  assert(!bfs::exists(root_path));
  root_path_ = root_path;
  bfs::create_directory(root_path_);
}

void StreamSequence::save(std::string filename) const {
  SerializeToFile(root_path_, filename, *this);
}

void StreamSequence::loadImpl(const std::string &full_fn) {
  if (! (bfs::is_directory(full_fn) && bfs::exists(full_fn))) {
    std::cout << "Path is not a path or does not exists! " << full_fn << std::endl;
    return;
  }
  root_path_ = bfs::path(full_fn).parent_path().string();
  auto const filename = bfs::path(full_fn).filename().string();
  SerializeFromFile(root_path_, filename, *this);
//   // ROS_WARN("Using old, deprecated StreamSequence.  FrameProjector is being "
//   //          "constructed manually.");
//   proj_.width_ = 640;
//   proj_.height_ = 480;
//   proj_.fx_ = 525;
//   proj_.fy_ = 525;
//   proj_.cx_ = proj_.width_ / 2;
//   proj_.cy_ = proj_.height_ / 2;
//   std::cout << "FrameProjector: " << std::endl;
//   std::cout << proj_.status("  ") << std::endl;

//   // -- Build filename index.
//   img_names_.clear();
//   dpt_names_.clear();
//   clk_names_.clear();
//   bfs::recursive_directory_iterator it(root_path_), eod;
//   BOOST_FOREACH (const bfs::path &p, make_pair(it, eod)) {
//     assert(is_regular_file(p));
//     int len = p.lead().string().lenght();
//     const auto end = p.leaf().string().substr(len-9, len);
//     if (end.compare("color.png") == 0) {
//       img_names_.push_back(p.leaf().string());
//     } else if (end.compare("depth.png") == 0)
//       dpt_names_.push_back(p.leaf().string());
//     else if (bfs::extension(p).compare(".clk") == 0)
//       clk_names_.push_back(p.leaf().string());
//   }
//   assert(img_names_.size() == dpt_names_.size());
//   assert(img_names_.size() == clk_names_.size());

//   // -- Sort all filenames.
//   sort(img_names_.begin(), img_names_.end());
//   sort(dpt_names_.begin(), dpt_names_.end());
//   sort(clk_names_.begin(), clk_names_.end());

//   // -- Load timestamps.
//   timestamps_.resize(clk_names_.size());
//   for (size_t i = 0; i < clk_names_.size(); ++i) {
//     std::ifstream fs((root_path_ + "/" + clk_names_[i]).c_str());
//     assert(fs.is_open());
//     fs >> timestamps_[i];
//     fs.close();
//   }
}

void StreamSequence::writeFrame(const Frame &frame) {
  std::ostringstream oss;
  size_t idx = size();

  oss << "clams-frame-" << idx << ".bin";
  frame_names_.push_back(oss.str());
  SerializeToFile(root_path_, frame_names_.back(), frame);

  // // -- Write image
  // std::vector<int> params;
  // if (getenv("SSEQ_PPM"))
  //   oss << "img" << setw(5) << setfill('0') << idx << ".ppm";
  // else
  //   oss << "img" << setw(5) << setfill('0') << idx << ".png";
  // cv::imwrite(root_path_ + "/" + oss.str(), frame.img_, params);
  // img_names_.push_back(oss.str());

  // // -- Write depth
  // oss.str("");
  // oss << "dpt" << setw(5) << setfill('0') << idx << ".eig";
  // eigen_extensions::save(*frame.depth_, root_path_ + "/" + oss.str());
  // dpt_names_.push_back(oss.str());

  // // -- Write timestamp and add to index
  // oss.str("");
  // oss << "clk" << setw(5) << setfill('0') << idx << ".clk";
  // ofstream fs((root_path_ + "/" + oss.str()).c_str());
  // fs.precision(10);
  // fs.setf(ios::fixed, ios::floatfield);
  // fs << frame.timestamp_ << endl;
  // fs.close();
  // clk_names_.push_back(oss.str());

  timestamps_.push_back(frame.timestamp);
}

void StreamSequence::readFrameImpl(size_t idx, Frame *frame) const {
  assert(idx < frame_names_.size());

  SerializeFromFile(root_path_, frame_names_[idx], *frame);
  if (max_depth_ != std::numeric_limits<double>::max())
    for (int y = 0; y < frame->depth.rows; ++y)
      for (int x = 0; x < frame->depth.cols; ++x)
        if (frame->depth(y, x) > max_depth_ * 1000)
          frame->depth(y, x) = 0;
}

size_t StreamSequence::size() const {
  return frame_names_.size();
}

// for serialization
template <class Archive>
void StreamSequence::serialize(Archive &ar, const unsigned int version) {
  ar& boost::serialization::base_object<StreamSequenceBase>(*this);
  ar& frame_names_;
  ar& max_depth_;
}

CLAMS_INSTANTIATE_SERIALIZATION_T(StreamSequence);

} // namespace rgbd
