#pragma once

#include <assert.h>
#include <vector>
#include <mutex>
#include "clams/common/typedefs.h"
#include "clams/common/clams_macros.h"
#include "clams/common/clams_defer.h"

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
  int index(double z) const;
  void undistort(double *z) const;
  void interpolatedUndistort(double *z) const;

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
  void serialize(Archive &ar, const unsigned int version);
};

using DiscreteFrustumPtr = std::shared_ptr<DiscreteFrustum>;

// DiscreteDepthDistortionModel ================================================
class DiscreteDepthDistortionModel {
public:
  DiscreteDepthDistortionModel() {}
  ~DiscreteDepthDistortionModel();
  DiscreteDepthDistortionModel(int width, int height, int bin_width = 8,
                               int bin_height = 6, double bin_depth = 2,
                               int smoothing = 1);
  DiscreteDepthDistortionModel(const DiscreteDepthDistortionModel &other) = delete;
  DiscreteDepthDistortionModel &
  operator=(const DiscreteDepthDistortionModel &other) = delete;

  void undistort(DepthMat& depth) const;
  //! Returns the number of training examples it used from this pair.
  //! Thread-safe.
  size_t accumulate(const DepthMat &ground_truth, const DepthMat &measurement);

  void addExample(int v, int u, double ground_truth, double measurement);
  void CalcMultipliers();
  void FillMissingMultipliers();
  void CalcDepthOffsets();
  void FillMissingDepthOffsets();
  void save(const std::string &path) const;
  void load(const std::string &path);
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

  DiscreteFrustum &frustum(int y, int x);
  const DiscreteFrustum &frustum(int y, int x) const;
  
public:
  // for serialization
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version);
};

using DiscreteDepthDistortionModelPtr = std::shared_ptr<DiscreteDepthDistortionModel>;
} // namespace clams
