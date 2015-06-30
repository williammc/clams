#pragma once
#include <array>
#include <Eigen/Core>
#include "clams/common/typedefs.h"
#include "clams/frame_projector.h"

#define DEFAULT_VGSIZE 0.01
#define MAX_RANGE_MAP 2

namespace clams {

using TimeFramePair = std::pair<double, std::string>;
using TimeFramePairVec = std::vector<TimeFramePair>;

using TimePosePair = std::pair<double, Eigen::Affine3d>;
using TimePosePairVec =
    std::vector<TimePosePair, Eigen::aligned_allocator<TimePosePair>>;

struct SlamMap {

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SlamMap();

  bool Load(const std::string &slammap_file);
  bool Save(const std::string &slammap_file) const;

  /// Load from external data format 
  /// @rec_file in recording format of me
  /// @traj_file in TUM trajectory file format
  /// @cam_params : width, height, fx, fy, cx, cy
  bool LoadTrajectoryAndRecording(std::string traj_file, std::string rec_file,
                                  std::array<float, 9> cam_params,
                                  unsigned skip_poses);
  Cloud::Ptr GeneratePointcloud(float max_range = MAX_RANGE_MAP, 
    float resolution = DEFAULT_VGSIZE) const;

  std::string working_path() const { return working_path_; }
  void working_path(std::string rp) { working_path_ = rp; }

  FrameProjector& frame_projector() {
    return proj_;
  }
  
  const FrameProjector& frame_projector() const {
    return proj_;
  }

  Cloud::Ptr pointcloud() const { return pointcloud_; }
  void pointcloud(Cloud::Ptr cl) { pointcloud_ = cl; }

  TimePosePairVec traj_timeposes() const { return traj_timeposes_; }

  void ReadFrameInTrajectory(size_t idx, Frame& frame) const;
  Eigen::Affine3d GetPoseInTrajectory(size_t idx) const {
    return traj_timeposes_[idx].second;
  }
  //! Returns the nearest frame, no matter how far away it is in time.  Check dt
  //to find out.
  size_t ReadFrameInTrajectory(double timestamp, double *dt, Frame &frame) const;

  //! dt is signed.
  size_t SeekInFrames(double timestamp, double *dt) const;
  size_t SeekInTrajectory(double timestamp, double *dt) const;

protected:
  std::string working_path_; ///< path of this SlamMap object
  std::string rec_file_, traj_file_;
  double max_depth_;
  bool undistorted_;
  FrameProjector proj_;
  Cloud::Ptr pointcloud_;  ///< integrated pointcloud of this map

  TimeFramePairVec rec_timeframes_;
  
  TimePosePairVec traj_timeposes_;
  
  ///< associate frame index of trajectory poses, based on timestamp
  std::vector<int> traj_associate_frames_;

public:
  // for serialization
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version);
};

} // namespace clams