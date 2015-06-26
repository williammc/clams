#pragma once

#include "stream_sequence/stream_sequence_base.h"

namespace clams {

class StreamSequence : public StreamSequenceBase {
public:
  using Ptr = std::shared_ptr<StreamSequence>;
  using ConstPtr = std::shared_ptr<const StreamSequence>;

  using StreamSequenceBase::timestamps_;
  using StreamSequenceBase::proj_;
  using StreamSequenceBase::root_path_;
  using StreamSequenceBase::seek;

  std::vector<std::string> frame_names_;

  //! The maximum depth in meters, used when reading data.
  //! Anything beyond this is set to 0.
  double max_depth_;

  //! Does not initialize anything.
  StreamSequence();
  //! Creates a new directory at root_path for streaming to.
  void init(const std::string &root_path);
  //! Saves PrimeSenseModel and timestamps to root_path_.
  //! Must have an initialized model_.
  void save(std::string fn = "clams-sseq.bin") const;
  size_t size() const;
  void writeFrame(const Frame &frame);

protected:
  //! Loads existing model and timestamps at root_path_, prepares for streaming
  //from here.
  void loadImpl(const std::string &root_path);
  // Assignment op & copy constructor would deep copy if they were implemented.
  //! Loads from disk and fills frame.
  void readFrameImpl(size_t idx, Frame *frame) const;

  StreamSequence(const StreamSequence &seq);
  StreamSequence &operator=(const StreamSequence &seq);


public:
  // for serialization
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version);
};
}
