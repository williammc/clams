#include "stream_sequence/stream_sequence_base.h"
#include "stream_sequence/stream_sequence.h"
#include <limits>

namespace clams {
StreamSequenceBase::Ptr
StreamSequenceBase::initializeFromDirectory(const std::string &dir, std::string filename) {
  // Check if it's the old or new format
  StreamSequenceBase::Ptr out;
  std::string full_fn = dir + "/" + filename;
  out.reset(new StreamSequence);
  out->load(full_fn);
  return out;
}

void StreamSequenceBase::load(const std::string &full_fn) {
  loadImpl(full_fn);
}

size_t StreamSequenceBase::seek(double timestamp, double *dt) const {
  assert(!timestamps_.empty());

  // TODO: This could be much faster than linear search.
  size_t nearest = 0;
  *dt = std::numeric_limits<double>::max();
  for (size_t i = 0; i < timestamps_.size(); ++i) {
    double d = timestamp - timestamps_[i];
    if (fabs(d) < *dt) {
      *dt = d;
      nearest = i;
    }
  }

  return nearest;
}

void StreamSequenceBase::applyTimeOffset(double dt) {
  for (size_t i = 0; i < timestamps_.size(); ++i)
    timestamps_[i] += dt;
}

Cloud::Ptr StreamSequenceBase::getCloud(size_t idx) const {
  Cloud::Ptr pcd(new Cloud);
  Frame frame;
  readFrame(idx, &frame);
  proj_.frameToCloud(frame, pcd.get());
  return pcd;
}

Cloud::Ptr StreamSequenceBase::getCloud(double timestamp, double *dt) const {
  Cloud::Ptr pcd(new Cloud);
  Frame frame;
  readFrame(timestamp, dt, &frame);
  proj_.frameToCloud(frame, pcd.get());
  return pcd;
}

cv::Mat3b StreamSequenceBase::getImage(size_t idx) const {
  Frame frame;
  readFrame(idx, &frame);
  return frame.img_;
}

cv::Mat3b StreamSequenceBase::getImage(double timestamp, double *dt) const {
  Frame frame;
  readFrame(timestamp, dt, &frame);
  return frame.img_;
}

Cloud::ConstPtr StreamSequenceBase::operator[](size_t idx) const {
  // With no cache, we just return the cloud
  if (cache_size_ == 0)
    return getCloud(idx);
  // Otherwise we update cache -- not threadsafe
  if (!pcds_cache_[idx]) {
#pragma omp critical
    {
      if (!pcds_cache_[idx]) {
        Cloud::ConstPtr cloud = getCloud(idx);
        addCloudToCache(idx, cloud);
      }
    }
  }
  return pcds_cache_[idx];
}

Cloud::ConstPtr StreamSequenceBase::at(size_t idx) const {
  assert(idx < size());
  return operator[](idx);
}

void StreamSequenceBase::addCloudToCache(size_t idx,
                                         Cloud::ConstPtr cloud) const {
  if (frames_cache_.size() >= cache_size_) {
    size_t frame_to_remove = frames_cache_.back();
    Cloud::ConstPtr &cloud_to_remove = pcds_cache_[frame_to_remove];
    cloud_to_remove.reset();
    frames_cache_.pop_back();
  }
  // Add to the cache
  frames_cache_.push_front(idx);
  pcds_cache_[idx] = cloud;
}

size_t StreamSequenceBase::readFrame(double timestamp, double *dt,
                                     Frame *frame) const {
  size_t idx = seek(timestamp, dt);
  readFrame(idx, frame);
  return idx;
}

void StreamSequenceBase::readFrame(size_t idx, Frame *frame) const {
  readFrameImpl(idx, frame);
}
} // namespace clams