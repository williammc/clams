#pragma once

#include "clams/slam_map.h"
#include "clams/cam_calib.h"
#include "clams/discrete_depth_distortion_model.h"

#define CALIB_CAM_INTRINSICS 0

namespace clams {
class SlamCalibrationVisualizer;

class SlamCalibrator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SlamCalibrator() = delete;

  SlamCalibrator(FrameProjector &proj);
  SlamCalibrator(const CalibPattern& pt, FrameProjector &proj);
  Cloud::Ptr BuildMap(unsigned int traj_idx);

  size_t size() const;
  bool Calibrate();
  void Validate();

  CalibPattern& pattern() { return pattern_; }
  FrameProjector& proj() { return proj_; }
  std::vector<SlamMap>& slam_maps() { return slam_maps_; };
  int& increment() { return increment_; }

  const slick::PoliCamera<double>& camera_model() const { return proj_.poli_cam(); }
  const DiscreteDepthDistortionModel& depth_model() const { return depth_model_; }

protected:
  bool PreparePlanarTargetData();  ///< track target and store data
  bool CalibrateIntrinsicsNoDepth(); ///< camera parameters: fx, fy, cx, cy, k1, k2
  bool CalibrateIntrinsicsUseDepth();
  bool CalibrateDepthDistortion(); ///< discrete depth distortion model

  //! Updates DiscreteDepthDistortionModel using a sequence, trajectory,
  //! and pre-built map.
  //! Returns the number of training examples harvested from this map.
  size_t ProcessMap(SlamMap &map);
  size_t ProcessMapPlanarTarget(SlamMap &map);

  static bool ValidDepthFromPlanarPattern(
                       cv::Mat3b img, cv::Mat1s undist_depth,
                       const slick::PoliCamera<>& cam,
                       const CalibPattern& pattern,
                        cv::Mat1f& errors, 
                        cv::Mat3b* drawn_pattern = nullptr);
  void ValidatePlanarTarget(SlamMap &slammap) const;

  friend class SlamCalibrationVisualizer;
  CalibPattern pattern_;
  FrameProjector& proj_;
  clams::DiscreteDepthDistortionModel depth_model_;
  std::vector<SlamMap> slam_maps_;
  int increment_;
};

using SlamCalibratorPtr = std::shared_ptr<SlamCalibrator>;
} // namespace clams
