#include "stream_sequence/stream_sequence.h"
#include <limits>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/foreach.hpp>

#include "clams/serialization/serialization.h"

namespace bfs = boost::filesystem;

namespace clams {

StreamSequence::StreamSequence() : max_depth_(std::numeric_limits<double>::max()) {}

StreamSequence::~StreamSequence() {

}

inline std::string get_frame_name(const Frame& fr) {
  return "clams-frame-" + std::to_string(fr.timestamp) + ".bin";
}

StreamSequence::Ptr
StreamSequence::LoadExternalRecording(std::string full_fn,
                                      std::string out_sseq_full_fn) {
  if (bfs::is_directory(full_fn) || !bfs::exists(full_fn)) {
    std::cout << "Filename is a path or does not exists! " << full_fn << std::endl;
    return StreamSequence::Ptr();
  }
  auto dir = bfs::path(full_fn).parent_path().string();
  auto out_dir = bfs::path(out_sseq_full_fn).parent_path().string();
  if (!bfs::exists(out_dir))
      bfs::create_directory(out_dir);

  auto const filename = bfs::path(full_fn).filename().string();

  StreamSequence::Ptr seg(new StreamSequence());
  seg->root_path_ = out_dir;

  std::ifstream ifs(full_fn);
  while (!ifs.eof()) {
    std::string cfn, dfn;
    Frame fr;
    red_recording_line(ifs, fr.timestamp, cfn, dfn);
    fr.img = cv::imread(dir + "/" + cfn, -1);
    cv::cvtColor(fr.img, fr.img, CV_BGR2RGB);
    fr.depth = cv::imread(dir + "/" + dfn, -1);
    
    if (fr.img.empty() || fr.depth.empty()) 
      continue;

    std::string base_fn = get_frame_name(fr);
    full_fn = out_dir + "/" + base_fn;
    printf("Serialize Frame to %s\n", full_fn.c_str());
    SerializeToFile(full_fn, fr);
    seg->frame_names_.push_back(base_fn);
    seg->timestamps_.push_back(fr.timestamp);
  }

  seg->proj_ = FrameProjector();
  seg->proj_.width_ = 640;
  seg->proj_.height_ = 480;
  seg->proj_.fx_ = 525;
  seg->proj_.fy_ = 525;
  seg->proj_.cx_ = seg->proj_.width_ / 2 + 0.5f;
  seg->proj_.cy_ = seg->proj_.height_ / 2 + 0.5f;

  printf("Serialize StreamSequence to %s\n", out_sseq_full_fn.c_str());
  SerializeToFile(out_sseq_full_fn, *seg);
  return seg;
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
  SerializeFromFile(full_fn, *this);
  root_path_ = bfs::path(full_fn).parent_path().string();
}

void StreamSequence::writeFrame(const Frame &frame) {
  size_t idx = size();

  frame_names_.push_back(get_frame_name(frame));
  SerializeToFile(root_path_, frame_names_.back(), frame);

  timestamps_.push_back(frame.timestamp);
}

void StreamSequence::readFrameImpl(size_t idx, Frame *frame) const {
  assert(idx < frame_names_.size());

  printf("StreamSequence>> read frame from :%s\n", frame_names_[idx].c_str());
  SerializeFromFile(root_path_ + "/" + frame_names_[idx], *frame);
  if (max_depth_ != std::numeric_limits<double>::max())
    for (int y = 0; y < frame->depth.rows; ++y)
      for (int x = 0; x < frame->depth.cols; ++x)
        if (frame->depth(y, x) > max_depth_ * 1000)
          frame->depth(y, x) = 0;
  printf("StreamSequence>> check frame img(%d, %d) depth(%d, %d)\n", frame->img.cols,
         frame->img.rows, frame->depth.cols, frame->depth.rows);
}

size_t StreamSequence::size() const {
  return frame_names_.size();
}

// for serialization
template <class Archive>
void StreamSequence::serialize(Archive &ar, const unsigned int version) {
  printf("StreamSequence::serialize >> 000\n");
  ar& boost::serialization::base_object<StreamSequenceBase>(*this);
  printf("StreamSequence::serialize >> 111\n");
  ar& frame_names_;
  printf("StreamSequence::serialize >> 222\n");
  ar& max_depth_;
}

CLAMS_INSTANTIATE_SERIALIZATION_T(StreamSequence);

} // namespace rgbd
