#pragma once

#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "slick/datatypes.h"
#include "slick/scene/poli_camera.h"
#include "clams/common/typedefs.h"
#include "clams/cam_calib.h"

namespace clams {

//! "Projective" point comes from the OpenNI terminology, and refers to (u, v,
//z), i.e.
//! pixel id and depth value.  Here I've added color, too, so that this
//represents everything
//! that is known about a pixel in an RBGD camera.
struct ProjectivePoint {
  int u;
  int v;
  //! In millimeters, same as the raw depth image from the primesense device.
  unsigned short z;
  unsigned char r;
  unsigned char g;
  unsigned char b;

  // for serialization
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version);
};

// Frame =======================================================================
struct Frame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Frame() {
    prefix = "clams-frame";
    timestamp = 0.0;
  }

  void FilterFringe();
  cv::Mat3b DepthImage() const;

protected:
  cv::Vec3b colorize(double depth, double min_range, double max_range) const;

public:
  std::string prefix;
  double timestamp;
  cv::Mat3b img;
  cv::Mat_<uint16_t> depth;
  CenterLocPairVec measurements; // world_point-pixel_location pairs

  // conventional data
  slick::SE3d target2cam; // calibration target to camera pose
  Eigen::Vector4d target_plane; // in camera coordinate

  // for serialization
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version);
};


// FrameProjector ==============================================================
//! This is essentially a pinhole camera model for an RGBD sensor, with
//! some extra functions added on for use during calibration.
class FrameProjector {
public:
  // For storing z values in meters.  This is not Euclidean distance.
  using RangeIndex = std::vector<std::vector<std::vector<double>>>;
  using IndexMap = Eigen::Matrix<size_t, Eigen::Dynamic, Eigen::Dynamic>;

  FrameProjector();

  void ReestimatePoseAndPlane(Frame &frame,
                              const Eigen::Vector4d &target_plane) const;

  //! max_range in meters
  void
  FrameToCloud(const Frame &frame, Cloud *pcd,
               double max_range = std::numeric_limits<double>::max()) const;
  void CloudToFrame(const Cloud &pcd, Frame *frame,
                    IndexMap *indexmap = NULL) const;
  void CloudToRangeIndex(const Cloud &pcd, RangeIndex *rindex) const;
  //! transform is applied to the map, then projected into a depth index.
  //! The best depth estimate from the map corresponding to the measurement
  //depth frame
  //! will be returned.
  void EstimateMapDepth(const Cloud &map, const Eigen::Affine3f &transform,
                        Frame &measurement, DepthMat& estimate) const;

  //! transform is applied to the map, then projected into a depth index.
  //! The best depth estimate from the map corresponding to the measurement
  //depth frame
  //! will be returned.
  void EstimateDepthFromPlanarPattern(const Eigen::Vector4d& target_plane,
                         Frame &measurement, DepthMat& estimate) const;

  ProjectivePoint Project(const Point& pt) const;
  Point UnProject(const ProjectivePoint &ppt) const;

  bool Initialized() const;
  std::string Status(const std::string &prefix = "") const;

  slick::PoliCamera<double>& poli_cam() { return poli_cam_; }
  const slick::PoliCamera<double>& poli_cam() const { return poli_cam_; }

  int width() const { return poli_cam_.ImageSize()[0]; }
  int height() const { return poli_cam_.ImageSize()[1]; }

protected:
  bool ConeFit(const DepthMat &naive_mapdepth, const RangeIndex &rindex, int uc,
               int vc, double radius, double measurement_depth, double *mean,
               double *stdev) const;


  slick::PoliCamera<double> poli_cam_;
public:
  // for serialization
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version);
};

} // namespace clams