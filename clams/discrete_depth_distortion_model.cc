#include "clams/discrete_depth_distortion_model.h"
#include <iostream>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include "clams/serialization/serialization.h"

namespace bfs = boost::filesystem;

namespace clams {

DiscreteFrustum::DiscreteFrustum(int smoothing, double bin_depth)
    : max_dist_(10), bin_depth_(bin_depth) {
  num_bins_ = ceil(max_dist_ / bin_depth_);
  counts_ = Eigen::VectorXf::Ones(num_bins_) * smoothing;
  total_numerators_ = Eigen::VectorXf::Ones(num_bins_) * smoothing;
  total_denominators_ = Eigen::VectorXf::Ones(num_bins_) * smoothing;
  multipliers_ = Eigen::VectorXf::Ones(num_bins_);
}

void DiscreteFrustum::addExample(double ground_truth, double measurement) {
  std::unique_lock<std::mutex> ul(mutex_);

  double mult = ground_truth / measurement;
  if (mult > MAX_MULT || mult < MIN_MULT)
    return;

  int idx = std::min(num_bins_ - 1, (int)std::floor(measurement / bin_depth_));
  assert(idx >= 0);

  total_numerators_(idx) += ground_truth * ground_truth;
  total_denominators_(idx) += ground_truth * measurement;
  ++counts_(idx);
  multipliers_(idx) = total_numerators_(idx) / total_denominators_(idx);
}

inline int DiscreteFrustum::index(double z) const {
  return std::min(num_bins_ - 1, (int)std::floor(z / bin_depth_));
}

inline void DiscreteFrustum::undistort(double *z) const {
  *z *= multipliers_.coeffRef(index(*z));
}

void DiscreteFrustum::interpolatedUndistort(double *z) const {
  int idx = index(*z);
  double start = bin_depth_ * idx;
  int idx1;
  if (*z - start < bin_depth_ / 2)
    idx1 = idx;
  else
    idx1 = idx + 1;
  int idx0 = idx1 - 1;
  if (idx0 < 0 || idx1 >= num_bins_ || counts_(idx0) < 50 ||
      counts_(idx1) < 50) {
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

template <class Archive>
void DiscreteFrustum::serialize(Archive &ar, const unsigned int version ) {
  ar& max_dist_;
  ar& num_bins_;
  ar& bin_depth_;
  ar& counts_;
  ar& total_numerators_;
  ar& total_denominators_;
  ar& multipliers_;
}

DiscreteDepthDistortionModel::DiscreteDepthDistortionModel(
    const DiscreteDepthDistortionModel &other) {
  *this = other;
}

DiscreteDepthDistortionModel &DiscreteDepthDistortionModel::
operator=(const DiscreteDepthDistortionModel &other) {
  width_ = other.width_;
  height_ = other.height_;
  bin_width_ = other.bin_width_;
  bin_height_ = other.bin_height_;
  bin_depth_ = other.bin_depth_;
  num_bins_x_ = other.num_bins_x_;
  num_bins_y_ = other.num_bins_y_;

  frustums_ = other.frustums_;
  for (size_t i = 0; i < frustums_.size(); ++i)
    for (size_t j = 0; j < frustums_[i].size(); ++j)
      frustums_[i][j] = new DiscreteFrustum(*other.frustums_[i][j]);

  return *this;
}

DiscreteDepthDistortionModel::DiscreteDepthDistortionModel(
    int width, int height, int bin_width, int bin_height, double bin_depth,
    int smoothing)
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
      frustums_[i][j] = new DiscreteFrustum(smoothing, bin_depth);
  }
}

void DiscreteDepthDistortionModel::deleteFrustums() {
  for (size_t y = 0; y < frustums_.size(); ++y)
    for (size_t x = 0; x < frustums_[y].size(); ++x)
      if (frustums_[y][x])
        delete frustums_[y][x];
}

DiscreteDepthDistortionModel::~DiscreteDepthDistortionModel() {
  deleteFrustums();
}

void DiscreteDepthDistortionModel::undistort(DepthMat& depth) const {
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

void DiscreteDepthDistortionModel::addExample(int v, int u, double ground_truth,
                                              double measurement) {
  frustum(v, u).addExample(ground_truth, measurement);
}

size_t DiscreteDepthDistortionModel::accumulate(const DepthMat &ground_truth,
                                                const DepthMat &measurement) {
  assert(width_ == ground_truth.cols);
  assert(height_ == ground_truth.rows);
  assert(width_ == measurement.cols);
  assert(height_ == measurement.rows);

  size_t num_training_examples = 0;
  for (int v = 0; v < height_; ++v) {
    for (int u = 0; u < width_; ++u) {
      if (ground_truth.at<uint16_t>(v, u) == 0)
        continue;
      if (measurement.at<uint16_t>(v, u) == 0)
        continue;

      double gt = ground_truth.at<uint16_t>(v, u) * 0.001;
      double meas = measurement.at<uint16_t>(v, u) * 0.001;
      frustum(v, u).addExample(gt, meas);
      ++num_training_examples;
    }
  }

  return num_training_examples;
}

void DiscreteDepthDistortionModel::load(const std::string &path) {
  SerializeFromFile(path.c_str(), *this);
}

void DiscreteDepthDistortionModel::save(const std::string &path) const {
  SerializeToFile(path.c_str(), *this);
}

template <class Archive>
void DiscreteDepthDistortionModel::serialize(Archive &ar, const unsigned int version) {
  ar& width_;
  ar& height_;
  ar& bin_width_;
  ar& bin_height_;
  ar& bin_depth_;
  ar& num_bins_x_;
  ar& num_bins_y_;
  ar& frustums_;
}

DiscreteFrustum &DiscreteDepthDistortionModel::frustum(int y, int x) {
  assert(x >= 0 && x < width_);
  assert(y >= 0 && y < height_);
  int xidx = x / bin_width_;
  int yidx = y / bin_height_;
  return (*frustums_[yidx][xidx]);
}

const DiscreteFrustum &DiscreteDepthDistortionModel::frustum(int y,
                                                             int x) const {
  assert(x >= 0 && x < width_);
  assert(y >= 0 && y < height_);
  int xidx = x / bin_width_;
  int yidx = y / bin_height_;
  return (*frustums_[yidx][xidx]);
}

std::string
DiscreteDepthDistortionModel::status(const std::string &prefix) const {
  std::ostringstream oss;
  oss << prefix << "Image width (pixels): " << width_ << "\n";
  oss << prefix << "Image height (pixels): " << height_ << "\n";
  oss << prefix << "Bin width (pixels): " << bin_width_ << "\n";
  oss << prefix << "Bin height (pixels): " << bin_height_ << "\n";
  oss << prefix << "Bin depth (m): " << bin_depth_ << "\n";
  return oss.str();
}

} // namespace clams
