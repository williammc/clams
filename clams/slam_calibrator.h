#pragma once

#include <pcl/filters/voxel_grid.h>
#include "stream_sequence/stream_sequence_base.h"
#include "clams/trajectory.h"
#include "clams/discrete_depth_distortion_model.h"

#define DEFAULT_VGSIZE 0.01
#define MAX_RANGE_MAP 2

namespace clams {
class SlamCalibrationVisualizer;

class SlamCalibrator {
public:
  SlamCalibrator() = default;

  SlamCalibrator(const FrameProjector &proj);
  Cloud::Ptr buildMap(unsigned int traj_idx);
  static Cloud::Ptr buildMap(StreamSequenceBase::ConstPtr sseq,
                             const Trajectory &traj, double max_range,
                             double vgsize = DEFAULT_VGSIZE);
  size_t size() const;
  DiscreteDepthDistortionModel calibrate() const;

  //! Updates DiscreteDepthDistortionModel using a sequence, trajectory,
  //! and pre-built map.
  //! Returns the number of training examples harvested from this map.
  size_t processMap(const StreamSequenceBase &sseq, const Trajectory &traj,
                    const Cloud &map,
                    DiscreteDepthDistortionModel *model) const;

  FrameProjector& proj() { return proj_; }

  std::vector<Trajectory>& trajectories() { return trajectories_; };
  std::vector<StreamSequenceBase::ConstPtr>& sseqs() { return sseqs_; };
  std::vector<Cloud::ConstPtr>& maps() { return maps_;};

  int& increment() { return increment_; }

protected:
  friend class SlamCalibrationVisualizer;
  FrameProjector proj_;
  std::vector<Trajectory> trajectories_;
  std::vector<StreamSequenceBase::ConstPtr> sseqs_;
  std::vector<Cloud::ConstPtr> maps_;
  int increment_;
};

using SlamCalibratorPtr = std::shared_ptr<SlamCalibrator>;
} // namespace clams
