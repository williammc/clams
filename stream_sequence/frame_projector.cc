#include "stream_sequence/frame_projector.h"
#include "clams/common/clams_macros.h"
#include "clams/serialization/serialization.h"

namespace clams {

template <class Archive>
void ProjectivePoint::serialize(Archive &ar, const unsigned int version) {
  ar& u_;
  ar& v_;
  ar& z_;
  ar& r_;
  ar& g_;
  ar& b_;
}

CLAMS_INSTANTIATE_SERIALIZATION_T(ProjectivePoint);

// Frame =======================================================================
cv::Vec3b Frame::colorize(double depth, double min_range,
                          double max_range) const {
  if (depth == 0)
    return cv::Vec3b(0, 0, 0);

  double increment = (max_range - min_range) / 3;
  double thresh0 = min_range;
  double thresh1 = thresh0 + increment;
  double thresh2 = thresh1 + increment;
  double thresh3 = thresh2 + increment;

  if (depth < thresh0) {
    return cv::Vec3b(0, 0, 255);
  }
  if (depth >= thresh0 && depth < thresh1) {
    int val = (depth - thresh0) / (thresh1 - thresh0) * 255.;
    return cv::Vec3b(val, val, 255 - val);
  } else if (depth >= thresh1 && depth < thresh2) {
    int val = (depth - thresh1) / (thresh2 - thresh1) * 255.;
    return cv::Vec3b(255, 255 - val, 0);
  } else if (depth >= thresh2 && depth < thresh3) {
    int val = (depth - thresh2) / (thresh3 - thresh2) * 255.;
    return cv::Vec3b(255 - val, val, 0);
  }

  return cv::Vec3b(0, 255, 0);
}

cv::Mat3b Frame::depthImage() const {
  cv::Mat3b dimg(depth.rows, depth.cols);
  dimg = cv::Vec3b(0, 0, 0);
  for (int y = 0; y < depth.rows; ++y)
    for (int x = 0; x < depth.cols; ++x)
      dimg(y, x) = colorize(depth.at<uint16_t>(y, x) * 0.001, 0, 10);
  return depth;
}

template <class Archive>
void Frame::serialize(Archive &ar, const unsigned int version) {
  printf("Frame::serialize 000\n");
  ar& prefix;
  ar& timestamp;
  std::string tfn = prefix + "-" + std::to_string(timestamp);
  std::string fn = tfn + "-color.png";
  std::string dfn = tfn + "-depth.png";
  ar& fn;
  ar& dfn;
  printf("Frame::serialize 111 fn:%s, dfn:%s\n", fn.c_str(), dfn.c_str());
  if (Archive::is_saving::value) {
    cv::cvtColor(img, img, CV_RGB2BGR);
    cv::imwrite(fn, img);
    cv::imwrite(dfn, depth);
  } else {
    cv::cvtColor(img, img, CV_BGR2RGB);
    img = cv::imread(fn, -1);
    depth = cv::imread(dfn, -1);
    printf("Check frame img(%d, %d) depth(%d, %d)\n", img.cols,
         img.rows, depth.cols, depth.rows);
  }
}

CLAMS_INSTANTIATE_SERIALIZATION_T(Frame);

// FrameProjector ==============================================================
FrameProjector::FrameProjector()
    : width_(-1), height_(-1), cx_(-1), cy_(-1), fx_(-1), fy_(-1) {}

void FrameProjector::cloudToRangeIndex(const Cloud &pcd,
                                       RangeIndex *rindex) const {
  RangeIndex &ind = *rindex;
  if ((int)ind.size() != height_)
    ind.resize(height_);
  for (size_t y = 0; y < ind.size(); ++y)
    if ((int)ind[y].size() != width_)
      ind[y].resize(width_);
  for (size_t y = 0; y < ind.size(); ++y) {
    for (size_t x = 0; x < ind[y].size(); ++x) {
      ind[y][x].clear();
      ind[y][x].reserve(10);
    }
  }

  ProjectivePoint ppt;
  for (size_t i = 0; i < pcd.size(); ++i) {
    if (!isFinite(pcd[i]))
      continue;
    project(pcd[i], &ppt);
    if (ppt.z_ == 0 ||
        !(ppt.u_ >= 0 && ppt.v_ >= 0 && ppt.u_ < width_ && ppt.v_ < height_))
      continue;
    ind[ppt.v_][ppt.u_].push_back(pcd[i].z);
  }
}

void FrameProjector::cloudToFrame(const Cloud &pcd, Frame *frame,
                                  IndexMap *indexmap) const {
  assert(frame);
  assert(width_ != -1 && height_ != -1 && cx_ != -1 && cy_ != -1 &&
             fx_ != -1 && fy_ != -1);

  frame->timestamp = pcd.header.stamp * 1e-9;
  frame->depth = cv::Mat(height_, width_, CV_16UC1, cv::Scalar(0));
  frame->img = cv::Mat3b(height_, width_);

  if (indexmap) {
    *indexmap = IndexMap(height_, width_);
  }

  ProjectivePoint ppt;
  for (size_t i = 0; i < pcd.size(); ++i) {
    if (!isFinite(pcd[i]))
      continue;

    // Ignore points outside the depth image or behind the sensor.
    project(pcd[i], &ppt);
    if (ppt.z_ <= 0 ||
        !(ppt.u_ >= 0 && ppt.v_ >= 0 && ppt.u_ < width_ && ppt.v_ < height_))
      continue;

    // Eigen is column-major by default:
    // http://eigen.tuxfamily.org/dox/TopicStorageOrders.html
    // opencv is row-major
    // pcl is row-major:
    // cout << "u, v: " << ppt.u_ << " " << ppt.v_ << endl;

    // Take the closest point in pcd.
    unsigned short curr_depth = frame->depth(ppt.v_, ppt.u_);
    if (curr_depth == 0 || ppt.z_ < curr_depth) {
      frame->depth(ppt.v_, ppt.u_) = ppt.z_;
      frame->img(ppt.v_, ppt.u_)[0] = ppt.b_;
      frame->img(ppt.v_, ppt.u_)[1] = ppt.g_;
      frame->img(ppt.v_, ppt.u_)[2] = ppt.r_;
      if (indexmap) {
        (*indexmap)(ppt.v_, ppt.u_) = i;
      }
    }
  }
}

void FrameProjector::frameToCloud(const Frame &frame, Cloud *pcd,
                                  double max_range) const {
  const auto &dm = frame.depth;
  cv::Mat3b img = frame.img;

  printf("width_:%d height_:%d img(%d, %d)\n",
    width_, height_, img.cols, img.rows);

  assert(fx_ > 0 && fy_ > 0 && cx_ > 0 && cy_ > 0);
  assert(dm.rows == img.rows);
  assert(dm.cols == img.cols);
  assert(img.rows == height_);
  assert(img.cols == width_);

  pcd->clear();
  pcd->height = dm.rows;
  pcd->width = dm.cols;
  pcd->is_dense = false;
  pcd->resize(dm.rows * dm.cols);
  pcd->header.stamp = (frame.timestamp) * 1e9;

  int idx = 0;
  ProjectivePoint ppt;
  for (ppt.v_ = 0; ppt.v_ < dm.rows; ++ppt.v_) {
    for (ppt.u_ = 0; ppt.u_ < dm.cols; ++ppt.u_, ++idx) {
      ppt.z_ = dm(ppt.v_, ppt.u_);
      if (ppt.z_ > max_range * 1000)
        ppt.z_ = 0; // bad point.

      ppt.r_ = img(ppt.v_, ppt.u_)[2];
      ppt.g_ = img(ppt.v_, ppt.u_)[1];
      ppt.b_ = img(ppt.v_, ppt.u_)[0];
      project(ppt, &pcd->at(idx));
    }
  }
}

void FrameProjector::project(const ProjectivePoint &ppt, Point *pt) const {
  assert(ppt.u_ >= 0 && ppt.v_ >= 0 && ppt.u_ < width_ && ppt.v_ < height_);

  pt->r = ppt.r_;
  pt->g = ppt.g_;
  pt->b = ppt.b_;

  if (ppt.z_ == 0) {
    pt->x = std::numeric_limits<float>::quiet_NaN();
    pt->y = std::numeric_limits<float>::quiet_NaN();
    pt->z = std::numeric_limits<float>::quiet_NaN();
  } else {
    pt->z = ppt.z_ * 0.001;
    pt->x = pt->z * (ppt.u_ - cx_) / fx_;
    pt->y = pt->z * (ppt.v_ - cy_) / fy_;
  }
}

void FrameProjector::project(const Point &pt, ProjectivePoint *ppt) const {
  assert(isFinite(pt));

  ppt->u_ = pt.x * fx_ / pt.z + cx_;
  ppt->v_ = pt.y * fy_ / pt.z + cy_;
  ppt->z_ = pt.z * 1000;
  ppt->r_ = pt.r;
  ppt->g_ = pt.g;
  ppt->b_ = pt.b;

  if (pt.z < 0)
    ppt->z_ = 0;
}

bool FrameProjector::initialized() const {
  if (cx_ == -1 || cy_ == -1)
    return false;
  if (fx_ == -1 || fy_ == -1)
    return false;
  if (width_ == -1 || height_ == -1)
    return false;

  return true;
}

std::string FrameProjector::status(const std::string &prefix) const {
  std::ostringstream oss;
  oss << prefix << "size: " << width_ << " x " << height_ << "\n";
  oss << prefix << "fx: " << fx_ << "\n";
  oss << prefix << "fy: " << fy_ << "\n";
  oss << prefix << "cx: " << cx_ << "\n";
  oss << prefix << "cy: " << cy_ << "\n";

  return oss.str();
}

void FrameProjector::estimateMapDepth(const Cloud &map,
                                      const Eigen::Affine3f &transform,
                                      const Frame &measurement,
                                      DepthMat *estimate) const {
  // -- Reallocate estimate if necessary.
  if (estimate->rows != measurement.depth.rows ||
      estimate->cols != measurement.depth.cols) {
    *estimate = DepthMat(measurement.depth.rows, measurement.depth.cols);
  }
  *estimate = 0;

  // -- Get the depth index.
  Cloud transformed;
  transformPointCloud(map, transformed, transform);
  RangeIndex rindex;
  cloudToRangeIndex(transformed, &rindex);

  // -- Compute the edge-of-map mask.
  Frame naive_mapframe;
  cloudToFrame(transformed, &naive_mapframe);
  const DepthMat &measurement_depth = measurement.depth;
  const DepthMat &naive_mapdepth = naive_mapframe.depth;
  cv::Mat1b mask(measurement_depth.rows, measurement_depth.cols);
  mask = 0;
  for (int y = 0; y < mask.rows; ++y)
    for (int x = 0; x < mask.cols; ++x)
      if (naive_mapdepth(y, x) != 0)
        mask(y, x) = 255;
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 4);
  cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 15);

  // -- Main loop: for all points in the image...
  ProjectivePoint ppt;
  Point pt;
  for (ppt.v_ = 0; ppt.v_ < measurement_depth.rows; ++ppt.v_) {
    for (ppt.u_ = 0; ppt.u_ < measurement_depth.cols; ++ppt.u_) {
      // -- Reject points with no data.
      if (measurement_depth(ppt.v_, ppt.u_) == 0)
        continue;
      if (naive_mapdepth(ppt.v_, ppt.u_) == 0)
        continue;

      // -- Reject points on the edge of the map.
      if (mask(ppt.v_, ppt.u_) == 0)
        continue;

      // -- Find nearby points in the cone to get a good estimate of the map
      // depth.
      double radius = 0.02;
      double mean = 0;
      double stdev = 0;
      // double stdev_thresh = numeric_limits<double>::max();
      double stdev_thresh = 0.03;
      bool valid =
          coneFit(naive_mapdepth, rindex, ppt.u_, ppt.v_, radius,
                  measurement_depth(ppt.v_, ppt.u_) * 0.001, &mean, &stdev);
      if (!valid)
        continue;
      if (stdev > stdev_thresh)
        continue;

      (*estimate)(ppt.v_, ppt.u_) = mean * 1000;
    }
  }
}

bool FrameProjector::coneFit(const DepthMat &naive_mapdepth,
                             const RangeIndex &rindex, int uc, int vc,
                             double radius, double measurement_depth,
                             double *mean, double *stdev) const {
  Point pt_center, pt_ul, pt_lr;
  ProjectivePoint ppt, ppt_ul, ppt_lr;
  ppt.u_ = uc;
  ppt.v_ = vc;
  ppt.z_ = (ushort)(measurement_depth * 1000);
  project(ppt, &pt_center);

  pt_ul = pt_center;
  pt_lr = pt_center;
  pt_ul.x -= radius;
  pt_ul.y -= radius;
  pt_lr.x += radius;
  pt_lr.y += radius;

  project(pt_ul, &ppt_ul);
  project(pt_lr, &ppt_lr);
  if (ppt_ul.z_ == 0 ||
      !(ppt_ul.u_ >= 0 && ppt_ul.v_ >= 0 && ppt_ul.u_ < naive_mapdepth.cols &&
        ppt_ul.v_ < naive_mapdepth.rows))
    return false;
  if (ppt_lr.z_ == 0 ||
      !(ppt_lr.u_ >= 0 && ppt_lr.v_ >= 0 && ppt_lr.u_ < naive_mapdepth.cols &&
        ppt_lr.v_ < naive_mapdepth.rows))
    return false;

  int min_u = ppt_ul.u_;
  int max_u = ppt_lr.u_;
  int min_v = ppt_ul.v_;
  int max_v = ppt_lr.v_;

  *mean = 0;
  double num = 0;
  for (ppt.u_ = min_u; ppt.u_ <= max_u; ++ppt.u_) {
    for (ppt.v_ = min_v; ppt.v_ <= max_v; ++ppt.v_) {
      const std::vector<double> &vals = rindex[ppt.v_][ppt.u_];
      for (size_t i = 0; i < vals.size(); ++i) {
        double mult = vals[i] / measurement_depth;
        if (mult > MIN_MULT && mult < MAX_MULT) {
          *mean += vals[i];
          ++num;
        }
      }
    }
  }
  if (num == 0)
    return false;
  *mean /= num;

  double var = 0;
  for (ppt.u_ = min_u; ppt.u_ <= max_u; ++ppt.u_) {
    for (ppt.v_ = min_v; ppt.v_ <= max_v; ++ppt.v_) {
      const std::vector<double> &vals = rindex[ppt.v_][ppt.u_];
      for (size_t i = 0; i < vals.size(); ++i) {
        double mult = vals[i] / measurement_depth;
        if (mult > MIN_MULT && mult < MAX_MULT)
          var += (vals[i] - *mean) * (vals[i] - *mean);
      }
    }
  }
  var /= num;

  *stdev = std::sqrt(var);
  return true;
}

template <class Archive>
void FrameProjector::serialize(Archive &ar, const unsigned int version) {
  ar& width_;
  ar& height_;
  ar& fx_;
  ar& fy_;
  ar& cx_;
  ar& cy_;
}

CLAMS_INSTANTIATE_SERIALIZATION_T(FrameProjector);

} // namespace clams
