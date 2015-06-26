#pragma once

#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "stream_sequence/pcl_typedefs.h"
#include "stream_sequence/typedefs.h"

namespace clams {

//! "Projective" point comes from the OpenNI terminology, and refers to (u, v,
//z), i.e.
//! pixel id and depth value.  Here I've added color, too, so that this
//represents everything
//! that is known about a pixel in an RBGD camera.
class ProjectivePoint {
public:
  int u_;
  int v_;
  //! In millimeters, same as the raw depth image from the primesense device.
  unsigned short z_;
  unsigned char r_;
  unsigned char g_;
  unsigned char b_;

public:
  // for serialization
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version);
};

class Frame {
public:
  Frame() {
    prefix_ = "clams-frame";
    timestamp_ = 0.0;
  }

  std::string& prefix() { return prefix_; }
  double& timestamp() { return timestamp_; }
  cv::Mat3b& img() {return img_; }
  cv::Mat_<uint16_t>& depth() { return depth_; }

  std::string prefix_;
  double timestamp_;
  cv::Mat3b img_;
  cv::Mat_<uint16_t> depth_;

  cv::Mat3b depthImage() const;

protected:
  cv::Vec3b colorize(double depth, double min_range, double max_range) const;

public:
  // for serialization
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version);
};

//! This is essentially a pinhole camera model for an RGBD sensor, with
//! some extra functions added on for use during calibration.
class FrameProjector {
public:
  // For storing z values in meters.  This is not Euclidean distance.
  using RangeIndex = std::vector<std::vector<std::vector<double>>>;
  using IndexMap = Eigen::Matrix<size_t, Eigen::Dynamic, Eigen::Dynamic>;

  int width_;
  int height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;

  FrameProjector();

  //! max_range in meters
  void
  frameToCloud(const Frame &frame, Cloud *pcd,
               double max_range = std::numeric_limits<double>::max()) const;
  void cloudToFrame(const Cloud &pcd, Frame *frame,
                    IndexMap *indexmap = NULL) const;
  void cloudToRangeIndex(const Cloud &pcd, RangeIndex *rindex) const;
  //! transform is applied to the map, then projected into a depth index.
  //! The best depth estimate from the map corresponding to the measurement
  //depth frame
  //! will be returned.
  void estimateMapDepth(const Cloud &map, const Eigen::Affine3f &transform,
                        const Frame &measurement, DepthMat *estimate) const;

  void project(const ProjectivePoint &ppt, Point *pt) const;
  void project(const Point &pt, ProjectivePoint *ppt) const;

  bool initialized() const;
  std::string status(const std::string &prefix = "") const;

protected:
  bool coneFit(const DepthMat &naive_mapdepth, const RangeIndex &rindex, int uc,
               int vc, double radius, double measurement_depth, double *mean,
               double *stdev) const;

public:
  // for serialization
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version);
};

} // namespace clams