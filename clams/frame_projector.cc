#include "clams/frame_projector.h"
#include "clams/common/clams_macros.h"
#include "clams/serialization/serialization.h"
#include "clams/pose_optimization.h"
#include "clams/plane_util.h"

namespace clams {

inline bool is_finite(const Point &pt) {
  return (pcl_isfinite(pt.x) && pcl_isfinite(pt.y) && pcl_isfinite(pt.z));
}

template <class Archive>
void ProjectivePoint::serialize(Archive &ar, const unsigned int version) {
  ar& u;
  ar& v;
  ar& z;
  ar& r;
  ar& g;
  ar& b;
}

CLAMS_INSTANTIATE_SERIALIZATION_T(ProjectivePoint);

// Frame =======================================================================
void Frame::FilterFringe() {
  cv::Mat1b mask(depth.rows, depth.cols); // points to be removed.
  mask = 0;
  uint16_t threshold = 100;// 5000; // millimeters
  for (int y = 1; y < depth.rows-1; ++y) {
    for (int x = 1; x < depth.cols-1; ++x) {
      float d = depth(y, x);
      float d0_1 = depth(y-1, x);
      float d_10 = depth(y, x-1);
      float d10 = depth(y, x+1);
      float d01 = depth(y+1, x);

      if (d == 0 ||d10 == 0 || d01 == 0 || d0_1 == 0 || d_10 == 0 ||
          std::fabs(d - d01) > threshold ||
          std::fabs(d - d10) > threshold ||
          std::fabs(d - d0_1) > threshold ||
          std::fabs(d - d_10) > threshold) {
        mask(y, x) = 255;
      }
    }
  }

#if 0 // enalble to visualize debugging info
  cv::imshow("mask", mask);
  cv::imshow("depth", DepthImage());
  cv::waitKey();

  // cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 8);
  // cv::imshow("mask", mask);
  // cv::waitKey();
#endif

  for (int y = 1; y < depth.rows; ++y)
    for (int x = 1; x < depth.cols; ++x)
      if (mask(y, x))
        depth(y, x) = 0;

  // cv::imshow("depth", DepthImage());
  // cv::waitKey();
}

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

cv::Mat3b Frame::DepthImage() const {
  cv::Mat3b dimg(depth.rows, depth.cols);
  dimg = cv::Vec3b(0, 0, 0);
  for (int y = 0; y < depth.rows; ++y)
    for (int x = 0; x < depth.cols; ++x)
      dimg(y, x) = colorize(depth(y, x) * 0.001, 0, 3);
  return dimg;
}

template <class Archive>
void Frame::serialize(Archive &ar, const unsigned int version) {
  ar& prefix;
  ar& timestamp;
  std::string tfn = prefix + "-" + std::to_string(timestamp);
  std::string fn = tfn + "-color.png";
  std::string dfn = tfn + "-depth.png";
  ar& fn;
  ar& dfn;
  if (Archive::is_saving::value) {
    cv::imwrite(fn, img);
    cv::imwrite(dfn, depth);
  } else {
    img = cv::imread(fn, -1);
    depth = cv::imread(dfn, -1);
    // cv::imshow("depth", DepthImage());
    // cv::waitKey();
  }
}

CLAMS_INSTANTIATE_SERIALIZATION_T(Frame);

// FrameProjector ==============================================================
FrameProjector::FrameProjector() {}

void FrameProjector::CloudToRangeIndex(const Cloud &pcd,
                                       RangeIndex *rindex) const {
  RangeIndex &ind = *rindex;
  if ((int)ind.size() != height())
    ind.resize(height());
  for (size_t y = 0; y < ind.size(); ++y)
    if ((int)ind[y].size() != width())
      ind[y].resize(width());
  for (size_t y = 0; y < ind.size(); ++y) {
    for (size_t x = 0; x < ind[y].size(); ++x) {
      ind[y][x].clear();
      ind[y][x].reserve(10);
    }
  }

  ProjectivePoint ppt;
  for (size_t i = 0; i < pcd.size(); ++i) {
    if (!clams::is_finite(pcd[i]))
      continue;
    ppt = Project(pcd[i]);
    if (ppt.z == 0 ||
        !(ppt.u >= 0 && ppt.v >= 0 && ppt.u < width() && ppt.v < height()))
      continue;
    ind[ppt.v][ppt.u].push_back(pcd[i].z);
  }
}

void
FrameProjector::ReestimatePoseAndPlane(Frame &frame,
                                       const Eigen::Vector4d &target_plane)  const {
  CenterLocPairVec observations;
  for (size_t i = 0; i < frame.measurements.size(); i++) {
    // unproject 2D(u,v) image points to camera plane z=1
    Eigen::Vector2d v2_unprojected =
        poli_cam_.UnProject(frame.measurements[i].second);
    observations.push_back(
        std::make_pair(frame.measurements[i].first, v2_unprojected));
  }
  auto const pair_re = FindBestPose2Steps(observations);
  frame.target2cam = pair_re.first;
  frame.target_plane = transform(target_plane, to_affine(frame.target2cam));
}

void FrameProjector::CloudToFrame(const Cloud &pcd, Frame *frame,
                                  IndexMap *indexmap) const {
  assert(frame);
  // assert(width() != -1 && height() != -1 && cx_ != -1 && cy_ != -1 &&
  //            fx_ != -1 && fy_ != -1);

  frame->timestamp = pcd.header.stamp * 1e-9;
  frame->depth = cv::Mat(height(), width(), CV_16UC1, cv::Scalar(0));
  frame->img = cv::Mat3b(height(), width());

  if (indexmap) {
    *indexmap = IndexMap(height(), width());
  }

  for (size_t i = 0; i < pcd.size(); ++i) {
    if (!clams::is_finite(pcd[i]))
      continue;

    // Ignore points outside the depth image or behind the sensor.
    const ProjectivePoint ppt = Project(pcd[i]);
    if (ppt.z <= 0 ||
        !(ppt.u >= 0 && ppt.v >= 0 && ppt.u < width() && ppt.v < height()))
      continue;

    // Eigen is column-major by default:
    // http://eigen.tuxfamily.org/dox/TopicStorageOrders.html
    // opencv is row-major
    // pcl is row-major:
    // cout << "u, v: " << ppt.u << " " << ppt.v << endl;

    // Take the closest point in pcd.
    unsigned short curr_depth = frame->depth(ppt.v, ppt.u);
    if (curr_depth == 0 || ppt.z < curr_depth) {
      frame->depth(ppt.v, ppt.u) = ppt.z;
      frame->img(ppt.v, ppt.u)[0] = ppt.b;
      frame->img(ppt.v, ppt.u)[1] = ppt.g;
      frame->img(ppt.v, ppt.u)[2] = ppt.r;
      if (indexmap) {
        (*indexmap)(ppt.v, ppt.u) = i;
      }
    }
  }
}

void FrameProjector::FrameToCloud(const Frame &frame, Cloud *pcd,
                                  double max_range) const {
  const auto &dm = frame.depth;
  cv::Mat3b img = frame.img;

  // printf("width():%d height():%d img(%d, %d)\n",
  //   width(), height(), img.cols, img.rows);

  // assert(fx_ > 0 && fy_ > 0 && cx_ > 0 && cy_ > 0);
  assert(dm.rows == img.rows);
  assert(dm.cols == img.cols);
  assert(img.rows == height());
  assert(img.cols == width());

  pcd->clear();
  pcd->height = dm.rows;
  pcd->width = dm.cols;
  pcd->is_dense = false;
  pcd->resize(dm.rows * dm.cols);
  pcd->header.stamp = (frame.timestamp) * 1e9;

  int idx = 0;
  ProjectivePoint ppt;
  for (ppt.v = 0; ppt.v < dm.rows; ++ppt.v) {
    for (ppt.u = 0; ppt.u < dm.cols; ++ppt.u, ++idx) {
      ppt.z = dm(ppt.v, ppt.u);
      if (ppt.z > max_range * 1000)
        ppt.z = 0; // bad point.

      ppt.r = img(ppt.v, ppt.u)[2];
      ppt.g = img(ppt.v, ppt.u)[1];
      ppt.b = img(ppt.v, ppt.u)[0];
      pcd->at(idx) = UnProject(ppt);
    }
  }
}


ProjectivePoint FrameProjector::Project(const Point& pt) const {
  assert(clams::is_finite(pt));
  ProjectivePoint ppt;
  auto loc = poli_cam_.Project(Eigen::Vector3d(pt.x, pt.y, pt.z));
  ppt.u = loc[0];
  ppt.v = loc[1];
  ppt.z = pt.z * 1000;
  ppt.r = pt.r;
  ppt.g = pt.g;
  ppt.b = pt.b;

  if (pt.z < 0)
    ppt.z = 0;
  return ppt;
}

Point FrameProjector::UnProject(const ProjectivePoint &ppt) const {
  auto imgsize = poli_cam_.ImageSize();
  assert(ppt.u >= 0 && ppt.v >= 0 && ppt.u < imgsize[0] && ppt.v < imgsize[1]);
  Point pt;
  pt.r = ppt.r;
  pt.g = ppt.g;
  pt.b = ppt.b;

  if (ppt.z == 0) {
    pt.x = std::numeric_limits<float>::quiet_NaN();
    pt.y = std::numeric_limits<float>::quiet_NaN();
    pt.z = std::numeric_limits<float>::quiet_NaN();
  } else {
    pt.z = ppt.z * 0.001;
    auto cam = poli_cam_.UnProject(Eigen::Vector2d(ppt.u, ppt.v));
    pt.x = cam[0];
    pt.y = cam[1];
  }
  return pt;
}

bool FrameProjector::Initialized() const {
  return true;
}

std::string FrameProjector::Status(const std::string &prefix) const {
  std::ostringstream oss;
  // oss << prefix << "size: " << width() << " x " << height() << "\n";
  // oss << prefix << "fx: " << fx_ << "\n";
  // oss << prefix << "fy: " << fy_ << "\n";
  // oss << prefix << "cx: " << cx_ << "\n";
  // oss << prefix << "cy: " << cy_ << "\n";

  return oss.str();
}

void FrameProjector::EstimateMapDepth(const Cloud &map, const Eigen::Affine3f &transform,
                        Frame &measurement, DepthMat& estimate) const {
  // -- Reallocate estimate if necessary.
  if (estimate.rows != measurement.depth.rows ||
      estimate.cols != measurement.depth.cols) {
    estimate = DepthMat(measurement.depth.rows, measurement.depth.cols);
  }
  estimate = 0;

  // -- Get the depth index.
  Cloud transformed;
  transformPointCloud(map, transformed, transform);
  RangeIndex rindex;
  CloudToRangeIndex(transformed, &rindex);

  // -- Compute the edge-of-map mask.
  Frame naive_mapframe;
  CloudToFrame(transformed, &naive_mapframe);
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
  for (ppt.v = 0; ppt.v < measurement_depth.rows; ++ppt.v) {
    for (ppt.u = 0; ppt.u < measurement_depth.cols; ++ppt.u) {
      // -- Reject points with no data.
      if (measurement_depth(ppt.v, ppt.u) == 0)
        continue;
      if (naive_mapdepth(ppt.v, ppt.u) == 0)
        continue;

      // -- Reject points on the edge of the map.
      if (mask(ppt.v, ppt.u) == 0)
        continue;

      // -- Find nearby points in the cone to get a good estimate of the map
      // depth.
      double radius = 0.02;
      double mean = 0;
      double stdev = 0;
      // double stdev_thresh = numeric_limits<double>::max();
      double stdev_thresh = 0.03;
      bool valid =
          ConeFit(naive_mapdepth, rindex, ppt.u, ppt.v, radius,
                  measurement_depth(ppt.v, ppt.u) * 0.001, &mean, &stdev);
      if (!valid)
        continue;
      if (stdev > stdev_thresh)
        continue;

      estimate(ppt.v, ppt.u) = mean * 1000;
    }
  }
}

struct Neighbor {
  Neighbor(int x = 0, int y = 0, int id = 0) : d_x(x), d_y(y), d_index(id) {}
  int d_x;
  int d_y;
  int d_index;
};

inline void find_connected_component(cv::Point pt, cv::Mat1b in_mask,
                                     cv::Mat1b &out_mask) {
  out_mask = in_mask.clone();
  out_mask = 0;
  
  // find upmost point
  cv::Point upmost = pt;
  while (true) {
    if (upmost.y <= 0 || in_mask(upmost.y-1, upmost.x) == 0)
      break;
    upmost.y -= 1;
  }

  // scrolling along the outer border to find the bounding polygon of the connected commponent
  std::vector<cv::Point> bder_pixs;
  bder_pixs.reserve(300*300);

  bder_pixs.push_back(upmost);

  // fill lookup table for next points to visit in clockwise order
  std::array<Neighbor, 8> directions = {
      Neighbor(-1, 0, -1), Neighbor(-1, -1, -in_mask.cols - 1),
      Neighbor(0, -1, -in_mask.cols), Neighbor(1, -1, -in_mask.cols + 1),
      Neighbor(1, 0, 1), Neighbor(1, 1, in_mask.cols + 1),
      Neighbor(0, 1, in_mask.cols), Neighbor(-1, 1, in_mask.cols - 1)};
  int upmost_idx = upmost.y * in_mask.cols + upmost.x;
  int curr_idx = upmost_idx;
  int curr_x = upmost.x;
  int curr_y = upmost.y;
  int direction = -1;
  int x, y;
  do {
    unsigned n_idx;
    for (unsigned d_idx = 1; d_idx <= 8; ++d_idx) {
      n_idx = (direction + d_idx) & 7;

      x = curr_x + directions[n_idx].d_x;
      y = curr_y + directions[n_idx].d_y;
      if (x >= 0 && x < int(in_mask.cols) && y >= 0 && y < int(in_mask.rows) &&
          in_mask(y, x))
        break;
    }

    // update the direction
    direction = (n_idx + 4) & 7;
    curr_idx += directions[n_idx].d_index;
    curr_x += directions[n_idx].d_x;
    curr_y += directions[n_idx].d_y;
    bder_pixs.push_back(cv::Point(curr_idx % in_mask.cols, curr_idx/in_mask.cols));
  } while (curr_idx != upmost_idx);

  bder_pixs.pop_back(); // last item is also the first one

  cv::fillPoly(out_mask, bder_pixs, cv::Scalar(255));

  cv::imshow("in_mask", in_mask);
  cv::imshow("out_mask", out_mask);
  cv::waitKey();
}

void FrameProjector::EstimateDepthFromPlanarPattern(const Eigen::Vector4d& target_plane,
                        Frame &measurement, DepthMat& estimate) const {
  // -- Reallocate estimate if necessary.
  if (estimate.rows != measurement.depth.rows ||
      estimate.cols != measurement.depth.cols) {
    estimate = DepthMat(measurement.depth.rows, measurement.depth.cols);
  }
  estimate = 0;

  double dist_threshold = 0.02; // 2 cm

  Frame& meas = measurement;
  ReestimatePoseAndPlane(meas, target_plane);
  DepthMat est_depth(meas.depth.rows, meas.depth.cols);
  est_depth = 0;
  cv::Mat1b est_mask(meas.depth.rows, meas.depth.cols);
  est_mask = 0;
  for (int v = 0; v < est_mask.rows; ++v) {
    for (int u = 0; u < est_mask.cols; ++u) {
      if (meas.depth(v, u) == 0) continue;
      const auto cam = slick::unproject(poli_cam_.UnProject(Eigen::Vector2d(u, v)));
      const auto pt = projection(cam, meas.target_plane);
      if (std::fabs(pt[2] - meas.depth(v, u)*0.001) < dist_threshold) {
        estimate(v, u) = pt[2]*1000;
        est_mask(v, u) = 255;
      }
    }
  }

  cv::Point pt(-1, -1);
  for (auto t : measurement.measurements)
    if (measurement.depth(t.second[1], t.second[0])) {
      pt.x = t.second[0];
      pt.y = t.second[1];
      break;
    }
  if (pt.x == -1) return;  // cannot find valid depth on the pattern

  cv::Mat1b center_mask;
  find_connected_component(pt, est_mask, center_mask);
  for (int v = 0; v < est_mask.rows; ++v) {
    for (int u = 0; u < est_mask.cols; ++u) {
      if (center_mask(v, u) == 0)
        estimate(v, u) = 0;
    }
  }
}

bool FrameProjector::ConeFit(const DepthMat &naive_mapdepth,
                             const RangeIndex &rindex, int uc, int vc,
                             double radius, double measurement_depth,
                             double *mean, double *stdev) const {
  ProjectivePoint ppt;
  ppt.u = uc;
  ppt.v = vc;
  ppt.z = (ushort)(measurement_depth * 1000);
  Point pt_center = UnProject(ppt);

  Point pt_ul = pt_center;
  Point pt_lr = pt_center;
  pt_ul.x -= radius;
  pt_ul.y -= radius;
  pt_lr.x += radius;
  pt_lr.y += radius;

  ProjectivePoint ppt_ul = Project(pt_ul);
  ProjectivePoint ppt_lr = Project(pt_lr);
  if (ppt_ul.z == 0 ||
      !(ppt_ul.u >= 0 && ppt_ul.v >= 0 && ppt_ul.u < naive_mapdepth.cols &&
        ppt_ul.v < naive_mapdepth.rows))
    return false;
  if (ppt_lr.z == 0 ||
      !(ppt_lr.u >= 0 && ppt_lr.v >= 0 && ppt_lr.u < naive_mapdepth.cols &&
        ppt_lr.v < naive_mapdepth.rows))
    return false;

  int min_u = ppt_ul.u;
  int max_u = ppt_lr.u;
  int min_v = ppt_ul.v;
  int max_v = ppt_lr.v;

  *mean = 0;
  double num = 0;
  for (ppt.u = min_u; ppt.u <= max_u; ++ppt.u) {
    for (ppt.v = min_v; ppt.v <= max_v; ++ppt.v) {
      const std::vector<double> &vals = rindex[ppt.v][ppt.u];
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
  for (ppt.u = min_u; ppt.u <= max_u; ++ppt.u) {
    for (ppt.v = min_v; ppt.v <= max_v; ++ppt.v) {
      const std::vector<double> &vals = rindex[ppt.v][ppt.u];
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
  auto imgsize = poli_cam_.ImageSize();
  auto params = poli_cam_.parameters();
  ar& imgsize;
  ar& params;
  if (Archive::is_loading::value) {
    poli_cam_.set_resolution_and_parameters(imgsize[0], imgsize[1], params);
  }
}

CLAMS_INSTANTIATE_SERIALIZATION_T(FrameProjector);

} // namespace clams
