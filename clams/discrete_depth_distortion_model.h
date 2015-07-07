#pragma once

#include <assert.h>
#include <vector>
#include <mutex>
#include "clams/common/typedefs.h"
#include "clams/common/clams_macros.h"
#include "clams/common/clams_defer.h"
#include "clams/serialization/serialization.h"

namespace clams {

/* SharedLockable is based on the uncopyable boost::shared_mutex.
   This presents a dilemma when assigning or copy constructing.
   Right now, the state of the mutex in the other SharedLockable
   does not get copied to the target SharedLockable.
   I'm not sure yet if this is the desired behavior.
*/
class SharedLockable {
public:
  SharedLockable() {}
  //! Copy constructor will make a new shared_mutex that is unlocked.
  SharedLockable(const SharedLockable &other) {}
  //! Assignment operator will *not* copy the mutex_ or the state of
  //mutex_ from other.
  SharedLockable &operator=(const SharedLockable &other) { return *this; }

  void lockWrite() { mutex_.lock(); }
  void unlockWrite() { mutex_.unlock(); }
  bool trylockWrite() { return mutex_.try_lock(); }

  void lockRead() { mutex_.lock(); }
  void unlockRead() { mutex_.unlock(); }
  bool trylockRead() { return mutex_.try_lock(); }

protected:
  //! For the first time ever, I'm tempted to make this mutable.
  //! It'd make user methods still be able to be const even if they are locking.
  std::mutex mutex_;
};

#define scopeLockWrite                                                         \
  lockWrite();                                                                 \
  auto tdefer = defer([&] (){ unlockWrite(); });

#define scopeLockRead                                                          \
  lockRead();                                                                  \
  auto tdefer =defer([&] (){ unlockRead(); });

// DiscreteFrustum =============================================================
class DiscreteFrustum : public SharedLockable {
public:
  DiscreteFrustum(int smoothing = 1, double bin_depth = 1.0);
  //! z value, not distance to origin.
  //! thread-safe.
  void addExample(double ground_truth, double measurement);
  void CalcMultipliers();
  void SmoothMultipliers();
  void CalcDepthOffsets();

  int index(double z) const {
    return std::min(num_bins_ - 1, (int)std::floor(z / bin_depth_));
  }

  void undistort(double *z) const {
    *z *= multipliers_.coeffRef(index(*z));
    // *z = *z + depth_offsets_(0);
  }

  void interpolatedUndistort(double *z) const {
    int idx = index(*z);
    double start = bin_depth_ * idx;
    int idx1;
    if (*z - start < bin_depth_ / 2)
      idx1 = idx;
    else
      idx1 = idx + 1;
    int idx0 = idx1 - 1;
    if (idx0 < 0 || idx1 >= num_bins_ 
        // || counts_(idx0) < 50 || counts_(idx1) < 50
       ) {
      undistort(z);
      return;
    }

    double z0 = (idx0 + 1) * bin_depth_ - bin_depth_ * 0.5;
    double coeff1 = (*z - z0) / bin_depth_;
    double coeff0 = 1.0 - coeff1;
    double mult = coeff0 * multipliers_.coeffRef(idx0) +
                  coeff1 * multipliers_.coeffRef(idx1);
    *z *= mult;
  }

  int num_bins() const { return num_bins_; }

  Eigen::VectorXf& multipliers() { return multipliers_; }
  Eigen::VectorXf& depth_offsets() { return depth_offsets_; }


protected:
  double max_dist_;
  int num_bins_;
  double bin_depth_;
  Eigen::VectorXf counts_;
  Eigen::VectorXf total_numerators_;
  Eigen::VectorXf total_denominators_;
  Eigen::VectorXf multipliers_;
  Eigen::VectorXf depth_offsets_;

  std::vector<std::vector<float>> gt_depths_, meas_depths_;

  friend class DiscreteDepthDistortionModel;

public:
  // for serialization
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version ) {
    ar& max_dist_;
    ar& num_bins_;
    ar& bin_depth_;
    ar& counts_;
    ar& total_numerators_;
    ar& total_denominators_;
    ar& multipliers_;
    ar& depth_offsets_;
  }
};

using DiscreteFrustumPtr = std::shared_ptr<DiscreteFrustum>;

// DiscreteDepthDistortionModel ================================================
class DiscreteDepthDistortionModel {
public:
  DiscreteDepthDistortionModel() {}
  ~DiscreteDepthDistortionModel();
  DiscreteDepthDistortionModel(int width, int height, int bin_width = 8,
                               int bin_height = 6, double bin_depth = 2,
                               int smoothing = 1)
   : width_(width), height_(height), bin_width_(bin_width),
      bin_height_(bin_height), bin_depth_(bin_depth) {
  assert(width_ % bin_width_ == 0);
  assert(height_ % bin_height_ == 0);

  num_bins_x_ = width_ / bin_width_;
  num_bins_y_ = height_ / bin_height_;

  frustums_.resize(num_bins_y_);
  for (size_t i = 0; i < frustums_.size(); ++i) {
    frustums_[i].resize(num_bins_x_, NULL);
    for (size_t j = 0; j < frustums_[i].size(); ++j)
      frustums_[i][j].reset(new DiscreteFrustum(smoothing, bin_depth));
  }
}
  DiscreteDepthDistortionModel(const DiscreteDepthDistortionModel &other) = delete;
  DiscreteDepthDistortionModel &
  operator=(const DiscreteDepthDistortionModel &other) = delete;

  void undistort(DepthMat& depth) const {
    assert(width_ == depth.cols);
    assert(height_ == depth.rows);

  #pragma omp parallel for
    for (int v = 0; v < height_; ++v) {
      for (int u = 0; u < width_; ++u) {
        if (depth.at<uint16_t>(v, u) == 0)
          continue;

        double z = depth.at<uint16_t>(v, u) * 0.001;
        frustum(v, u).interpolatedUndistort(&z);
        depth.at<uint16_t>(v, u) = z * 1000;
      }
    }
  }

  //! Returns the number of training examples it used from this pair.
  //! Thread-safe.
  size_t accumulate(const DepthMat &ground_truth, const DepthMat &measurement);

  void addExample(int v, int u, double ground_truth, double measurement);
  void CalcMultipliers();
  void FillMissingMultipliers();
  void CalcDepthOffsets();
  void FillMissingDepthOffsets();

  void save(const std::string &path) const {
    SerializeToFile(path.c_str(), *this);
  }

  void load(const std::string &path) {
    SerializeFromFile(path.c_str(), *this);
  }

  //! Saves images to the directory found at path.
  //! If path doesn't exist, it will be created.
  void visualize(const std::string &path) const;
  std::string status(const std::string &prefix = "") const;

  int num_bins_x() const { return num_bins_x_; }
  int num_bins_y() const { return num_bins_y_; }

protected:
  //! Image width.
  int width_;
  //! Image height.
  int height_;
  //! Width of each bin in pixels.
  int bin_width_;
  //! Height of each bin in pixels.
  int bin_height_;
  //! Depth of each bin in meters.
  double bin_depth_;
  int num_bins_x_;
  int num_bins_y_;
  //! frustums_[y][x]
  std::vector<std::vector<DiscreteFrustumPtr>> frustums_;

  DiscreteFrustum &frustum(int y, int x) {
    assert(x >= 0 && x < width_);
    assert(y >= 0 && y < height_);
    int xidx = x / bin_width_;
    int yidx = y / bin_height_;
    return (*frustums_[yidx][xidx]);
  }

  const DiscreteFrustum &frustum(int y,
                                                               int x) const {
    assert(x >= 0 && x < width_);
    assert(y >= 0 && y < height_);
    int xidx = x / bin_width_;
    int yidx = y / bin_height_;
    return (*frustums_[yidx][xidx]);
  }

  
public:
  // for serialization
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar& width_;
    ar& height_;
    ar& bin_width_;
    ar& bin_height_;
    ar& bin_depth_;
    ar& num_bins_x_;
    ar& num_bins_y_;
    ar& frustums_;
  }
};

using DiscreteDepthDistortionModelPtr = std::shared_ptr<DiscreteDepthDistortionModel>;
} // namespace clams
