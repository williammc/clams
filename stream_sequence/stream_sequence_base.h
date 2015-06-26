#pragma once

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>
// #include <timer/timer.h>
#include "stream_sequence/frame_projector.h"

namespace clams {

class StreamSequenceBase {
public:
  using Ptr = std::shared_ptr<StreamSequenceBase>;
  using ConstPtr = std::shared_ptr<const StreamSequenceBase>;

  StreamSequenceBase() : cache_size_(0) {}

  virtual ~StreamSequenceBase() {}

  static StreamSequenceBase::Ptr
  initializeFromDirectory(const std::string &dir, std::string filename = "clams-sseq.bin");

  void load(const std::string &root_path);

  std::string getRootPath() const { return root_path_; };

  virtual size_t size() const = 0;

  void readFrame(size_t idx, Frame *frame) const;
  //! Returns the nearest frame, no matter how far away it is in time.  Check dt
  //to find out.
  size_t readFrame(double timestamp, double *dt, Frame *frame) const;

  //! dt is signed.
  size_t seek(double timestamp, double *dt) const;

  clams::Cloud::ConstPtr operator[](size_t idx) const;

  clams::Cloud::ConstPtr at(size_t idx) const;

  void setCacheSize(size_t cache_size) const {
    cache_size_ = cache_size;
  };

  //! Adds dt to all timestamps.  Does not save.
  void applyTimeOffset(double dt);
  //! Inefficient accessors that conceal how the projection is done.
  //! These shouldn't be used if it can be avoided.
  clams::Cloud::Ptr getCloud(size_t idx) const;
  clams::Cloud::Ptr getCloud(double timestamp, double *dt) const;
  cv::Mat3b getImage(size_t idx) const;
  cv::Mat3b getImage(double timestamp, double *dt) const;

  FrameProjector& GetFrameProjector() {
    return proj_;
  }
  
  const FrameProjector& GetFrameProjector() const {
    return proj_;
  }

protected:
  friend class StreamSequenceAccessor;

  virtual void readFrameImpl(size_t idx, Frame *frame) const = 0;
  virtual void loadImpl(const std::string &root_path) = 0;

  void addCloudToCache(size_t idx, clams::Cloud::ConstPtr cloud) const;

  std::vector<double> timestamps_;
  FrameProjector proj_;

  mutable std::deque<size_t> frames_cache_;
  mutable std::map<size_t, clams::Cloud::ConstPtr> pcds_cache_;
  mutable size_t cache_size_;
  std::string root_path_;
  bool undistort_;

public:
  // for serialization
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar& timestamps_;
    ar& proj_;
    
    ar& frames_cache_;
    //ar& pcds_cache_;
    ar& cache_size_;
    ar& root_path_;
    ar& undistort_;
  }
};

//! Accessor which has "push back" functionality, remapping indices
//appropriately...don't ask
class StreamSequenceAccessor {
public:
  using Ptr = std::shared_ptr<StreamSequenceAccessor>;
  using ConstPtr = std::shared_ptr<const StreamSequenceAccessor>;

  StreamSequenceAccessor(StreamSequenceBase::ConstPtr sseq) : sseq_(sseq) {}

  void turnOnFrameNumber(size_t idx) { frames_.push_back(idx); }

  clams::Cloud::ConstPtr operator[](size_t idx) const {
    return ((*sseq_)[frames_[idx]]);
  }

  clams::Cloud::ConstPtr at(size_t idx) const {
    assert(idx < frames_.size());
    return (operator[](idx));
  }

  void setCacheSize(size_t cache_size) const {
    sseq_->setCacheSize(cache_size);
  }

  size_t size() const { return frames_.size(); }

protected:
  StreamSequenceBase::ConstPtr sseq_;
  std::vector<size_t> frames_;

public:
  // for serialization
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar& sseq_;
    ar& frames_;
  }
};

inline bool isFinite(const Point &pt) {
  return (pcl_isfinite(pt.x) && pcl_isfinite(pt.y) && pcl_isfinite(pt.z));
}
} // namespace clams