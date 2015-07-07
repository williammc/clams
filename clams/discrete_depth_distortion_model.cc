#include "clams/discrete_depth_distortion_model.h"
#include <iostream>
#include <numeric>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "slick/util/mestimator.h"
#include "clams/common/clams_macros.h"
#include "clams/serialization/serialization.h"

namespace bfs = boost::filesystem;

namespace clams {

std::vector<int> get_inliers(std::vector<float> meas) {
  slick::Tukey<float> est;
  auto avg = std::accumulate(meas.begin(), meas.end(), 0.0f) / meas.size();
  std::vector<float> sqerrors(meas.size());
  for (size_t i = 0; i < meas.size(); ++i) {
    sqerrors[i] = (meas[i] - avg) * (meas[i] - avg);
  }
  est.compute_sigma_squared(sqerrors);

  std::vector<int> inliers;
  inliers.reserve(meas.size());
  for (size_t i = 0; i < sqerrors.size(); ++i) {
    if (est.weight(sqerrors[i]) > 0.85)
      inliers.push_back(i);
  }
  return inliers;
}

DiscreteFrustum::DiscreteFrustum(int smoothing, double bin_depth)
    : max_dist_(10), bin_depth_(bin_depth) {
  num_bins_ = ceil(max_dist_ / bin_depth_);
  counts_ = Eigen::VectorXf::Ones(num_bins_) * smoothing;
  total_numerators_ = Eigen::VectorXf::Ones(num_bins_) * smoothing;
  total_denominators_ = Eigen::VectorXf::Ones(num_bins_) * smoothing;
  multipliers_ = Eigen::VectorXf::Ones(num_bins_);
  depth_offsets_ = Eigen::VectorXf::Ones(num_bins_);
  gt_depths_.resize(num_bins_);
  meas_depths_.resize(num_bins_);
}

void DiscreteFrustum::addExample(double ground_truth, double measurement) {
  std::unique_lock<std::mutex> ul(mutex_);

  double mult = ground_truth / measurement;
  if (mult > MAX_MULT || mult < MIN_MULT)
    return;

  int idx = std::min(num_bins_ - 1, (int)std::floor(measurement / bin_depth_));
  assert(idx >= 0);

  gt_depths_.at(idx).push_back(ground_truth);
  meas_depths_.at(idx).push_back(measurement);
  ++counts_(idx);

  // const auto inliers = get_inliers(meas_depths_.at(idx));
  // if (inliers.empty()) return;

  // auto& gt = gt_depths_.at(idx);
  // auto& meas = meas_depths_.at(idx);
  // auto& num = total_numerators_(idx);
  // num = 0.0f;
  // auto& den = total_denominators_(idx);
  // den = 0.0f;
  // for (size_t i = 0; i < inliers.size(); ++i) {
  //   num += gt.at(inliers.at(i))*gt.at(inliers.at(i));
  //   den += gt.at(inliers.at(i)) * meas.at(inliers.at(i));
  // }

  // // total_numerators_(idx) += ground_truth * ground_truth;
  // // total_denominators_(idx) += ground_truth * measurement;
  // multipliers_(idx) = total_numerators_(idx) / total_denominators_(idx);
}

void DiscreteFrustum::CalcMultipliers() {
  std::unique_lock<std::mutex> ul(mutex_);

  for (size_t idx = 0; idx < gt_depths_.size(); ++idx) {
    const auto inliers = get_inliers(meas_depths_.at(idx));
    if (inliers.empty()) continue;

    auto& gt = gt_depths_.at(idx);
    auto& meas = meas_depths_.at(idx);
    auto& num = total_numerators_(idx);
    num = 0.0f;
    auto& den = total_denominators_(idx);
    den = 0.0f;
    for (auto idx : inliers) {
      num += gt.at(idx)*gt.at(idx);
      den += gt.at(idx) * meas.at(idx);
    }

    multipliers_(idx) = total_numerators_(idx) / total_denominators_(idx);
  }
}

void smooth_it(const std::vector<float>&data,
               std::vector<float> &smth_data, const unsigned smooth_size = 7) {
  assert(smooth_size % 2 == 1);

  cv::Mat kernel = cv::getGaussianKernel(smooth_size, -1, CV_32F);

  /// extending two ends of the normals list
  std::vector<float> data1(data.size());
  for (size_t i = 0; i < data.size(); ++i) {
    data1[i] = data[i];
  }
  for (size_t i = 1; i < data1.size(); ++i) {
    if (data1[i] == 1.0f)
    data1[i] = data1[i-1];
  }

  int halfsize = smooth_size / 2;
  for (size_t i = 0; i < halfsize; ++i) {
    data1.push_back(0);
    data1.insert(data1.begin(), 0);
  }

  // calc smoothed normals;
  smth_data.resize(data.size());
  for (size_t i = 0; i < data.size(); ++i) {
    float val = 0.0f;
    for (size_t j = 0; j < smooth_size; ++j) {
      val += kernel.at<float>(j, 0) * data1[i + j];
    }
    smth_data[i] = val;
  }
  printf("data:");
  for (auto d : data)
    printf("%f, ", d);
  printf("\nsm_data:");
  for (auto d : smth_data)
    printf("%f, ", d);
  printf("\n");
}

void DiscreteFrustum::SmoothMultipliers() {
  return;
  std::unique_lock<std::mutex> ul(mutex_);

  std::vector<float> mults(multipliers_.rows());
  for (size_t idx = 0; idx < gt_depths_.size(); ++idx) {
    mults[idx] = multipliers_(idx);
  }
  std::vector<float> sm_mults;
  smooth_it(mults, sm_mults, 3);
  for (size_t idx = 0; idx < gt_depths_.size(); ++idx) {
    multipliers_(idx) = sm_mults[idx];
  }
}

void DiscreteFrustum::CalcDepthOffsets() {
  std::unique_lock<std::mutex> ul(mutex_);

  std::vector<float> all_offsets;
  for (size_t idx = 0; idx < gt_depths_.size(); ++idx) {
    auto& gtdeps = gt_depths_.at(idx);
    auto& meas_deps = meas_depths_.at(idx);
    for (size_t idx1 = 0; idx1 < meas_deps.size(); ++idx1) {
      float offset = gtdeps.at(idx1) - meas_deps.at(idx1);
      all_offsets.push_back(offset);
    }
  }

  auto inliers = get_inliers(all_offsets);
  float avg = 0.0f;
  for (auto id : inliers)
    avg += all_offsets[id];
  avg /= inliers.size();

  for (int i = 0; i < depth_offsets_.rows(); ++i) 
    depth_offsets_(i) = avg;

}

inline int DiscreteFrustum::index(double z) const {
  return std::min(num_bins_ - 1, (int)std::floor(z / bin_depth_));
}

inline void DiscreteFrustum::undistort(double *z) const {
  *z *= multipliers_.coeffRef(index(*z));
  // *z = *z + depth_offsets_(0);
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

template <class Archive>
void DiscreteFrustum::serialize(Archive &ar, const unsigned int version ) {
  ar& max_dist_;
  ar& num_bins_;
  ar& bin_depth_;
  ar& counts_;
  ar& total_numerators_;
  ar& total_denominators_;
  ar& multipliers_;
  ar& depth_offsets_;
}

// =============================================================================
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
      frustums_[i][j].reset(new DiscreteFrustum(smoothing, bin_depth));
  }
}

DiscreteDepthDistortionModel::~DiscreteDepthDistortionModel() {
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

void DiscreteDepthDistortionModel::CalcMultipliers() {
  for (auto t : frustums_) {
    for (auto frustum : t) {
      frustum->CalcMultipliers();
    }
  }
}

void DiscreteDepthDistortionModel::CalcDepthOffsets() {
  for (auto t : frustums_) {
    for (auto frustum : t) {
      frustum->CalcDepthOffsets();
    }
  }
}

void DiscreteDepthDistortionModel::FillMissingMultipliers() {
  int depth_bins = frustums_.front().front()->num_bins();
  for (int i = 0; i < depth_bins; ++i) {
    printf("Fill depth bin%d\n", i);
    cv::Mat_<float> multis(num_bins_y_, num_bins_x_);
    multis = 0;
    for (size_t y = 0; y < frustums_.size(); ++y) {
      for (size_t x = 0; x < frustums_[y].size(); ++x) {
        multis(y, x) = frustums_[y][x]->multipliers()(i);
        multis(y, x) = (multis(y, x) == 1.0f) ? 0.0f : multis(y, x);
      }
    }

    auto exists_zero = [&]() -> bool {
      for (int y = 0; y < multis.rows; ++y) {
        for (int x = 0; x < multis.cols; ++x) {
          if (multis(y, x) == 0.0f) return true;
        }
      }
      return false;
    };

    int count = 0;
    while (exists_zero() && count < 10) {
      cv::dilate(multis, multis, cv::Mat(), cv::Point(-1, -1), 8);
      count += 1;
    }

    auto multisclone = multis.clone();
    cv::GaussianBlur(multisclone, multis, cv::Size(3, 3), 0);
    if (count >= 10) continue;
    for (size_t y = 0; y < frustums_.size(); ++y) {
      for (size_t x = 0; x < frustums_[y].size(); ++x) {
        frustums_[y][x]->multipliers()(i) = multis(y, x);
      }
    }
  }

  // smooth multipliers
  printf("Smooth multipliers\n");
  for (size_t y = 0; y < frustums_.size(); ++y) {
    for (size_t x = 0; x < frustums_[y].size(); ++x) {
      frustums_[y][x]->SmoothMultipliers();
    }
  }

}

void DiscreteDepthDistortionModel::FillMissingDepthOffsets() {
  return;
  int depth_bins = frustums_.front().front()->num_bins();
  for (int i = 0; i < 1; ++i) {
    printf("Fill depth bin%d\n", i);
    cv::Mat_<float> offsets(num_bins_y_, num_bins_x_);
    offsets = 0;
    for (size_t y = 0; y < frustums_.size(); ++y) {
      for (size_t x = 0; x < frustums_[y].size(); ++x) {
        offsets(y, x) = frustums_[y][x]->depth_offsets()(i);
        offsets(y, x) = (offsets(y, x) == 1.0f) ? 0.0f : offsets(y, x);
      }
    }

    auto exists_zero = [&]() -> bool {
      for (int y = 0; y < offsets.rows; ++y) {
        for (int x = 0; x < offsets.cols; ++x) {
          if (offsets(y, x) == 0.0f) return true;
        }
      }
      return false;
    };

    int count = 0;
    while (exists_zero() && count < 10) {
      cv::dilate(offsets, offsets, cv::Mat(), cv::Point(-1, -1), 8);
      count += 1;
    }
    if (count >= 10) continue;
    for (size_t y = 0; y < frustums_.size(); ++y) {
      for (size_t x = 0; x < frustums_[y].size(); ++x) {
        auto& perfrus_offsets = frustums_[y][x]->depth_offsets();
        for (int i = 0; i < perfrus_offsets.rows(); ++i)
          perfrus_offsets(i) = offsets(y, x);
      }
    }
  }
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

CLAMS_INSTANTIATE_SERIALIZATION_T(DiscreteDepthDistortionModel)
} // namespace clams
