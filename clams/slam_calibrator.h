#pragma once

#include "clams/slam_map.h"
#include "clams/discrete_depth_distortion_model.h"

namespace clams {
class SlamCalibrationVisualizer;

class SlamCalibrator {
public:
  SlamCalibrator() = default;

  SlamCalibrator(const FrameProjector &proj);
  Cloud::Ptr BuildMap(unsigned int traj_idx);

  size_t size() const;
  DiscreteDepthDistortionModel Calibrate() const;

  //! Updates DiscreteDepthDistortionModel using a sequence, trajectory,
  //! and pre-built map.
  //! Returns the number of training examples harvested from this map.
  size_t ProcessMap(const SlamMap &map,
                    DiscreteDepthDistortionModel *model) const;

  FrameProjector& proj() { return proj_; }
  std::vector<SlamMap>& slam_maps() { return slam_maps_; };
  int& increment() { return increment_; }

protected:
  friend class SlamCalibrationVisualizer;
  FrameProjector proj_;
  std::vector<SlamMap> slam_maps_;
  int increment_;
};

using SlamCalibratorPtr = std::shared_ptr<SlamCalibrator>;
} // namespace clams
