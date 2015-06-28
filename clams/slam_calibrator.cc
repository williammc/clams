#include "clams/slam_calibrator.h"
#include "clams/common/timer.h"

namespace clams {

SlamCalibrator::SlamCalibrator(const FrameProjector &proj)
    : proj_(proj), increment_(1) {}

Cloud::Ptr SlamCalibrator::BuildMap(unsigned int idx) {
  assert(idx < slam_maps_.size());
  return slam_maps_[idx].GeneratePointcloud(MAX_RANGE_MAP);
}

size_t SlamCalibrator::size() const {
  return slam_maps_.size();
}

DiscreteDepthDistortionModel SlamCalibrator::Calibrate() const {
  // assert(!sseqs_.empty());
  clams::DiscreteDepthDistortionModel model(slam_maps_[0].frame_projector().width_,
                                     slam_maps_[0].frame_projector().height_);

  size_t total_num_training = 0;
  for (size_t i = 0; i < size(); ++i) {
    std::cout << "Accumulating training data for sequence " << i << std::flush;
    total_num_training +=
        ProcessMap(slam_maps_[i], &model);
  }

  std::cout << "Trained new DiscreteDepthDistortionModel using "
       << total_num_training << " training examples." << std::endl;

  return model;
}

size_t SlamCalibrator::ProcessMap(const SlamMap& slammap,
                                  DiscreteDepthDistortionModel *model) const {
  // -- Select which frame indices from the sequence to use.
  //    Consider only those with a pose in the Trajectory,
  //    and apply downsampling based on increment_.
  const Cloud& pointcloud = *(slammap.pointcloud());

  // -- For all selected frames, accumulate training examples
  //    in the distortion model.
  Eigen::VectorXi counts = Eigen::VectorXi::Zero(slammap.traj_timeposes().size());
#pragma omp parallel for
  for (size_t i = 0; i < slammap.traj_timeposes().size(); ++i) {
    std::cout << "." << std::flush;

    Frame measurement;
    slammap.ReadFrameInTrajectory(i, measurement);

    Frame mapframe;
    slammap.frame_projector().EstimateMapDepth(pointcloud, slammap.GetPoseInTrajectory(i).inverse().cast<float>(),
                                measurement, &mapframe.depth);
    counts[i] = model->accumulate(mapframe.depth, measurement.depth);

    // cv::imshow("map", mapframe.depthImage());
    // cv::imshow("measurement", measurement.depthImage());
    // cv::waitKey();

    // -- Quick and dirty option for data inspection.
    if (getenv("U") && getenv("V")) {
      int u_center = atoi(getenv("U"));
      int v_center = atoi(getenv("V"));
      int radius = 1;
      for (int u = std::max(0, u_center - radius);
           u < std::min(640, u_center + radius + 1); ++u) {
        for (int v = std::max(0, v_center - radius);
             v < std::min(480, v_center + radius + 1); ++v) {
          if (mapframe.depth(v, u) == 0)
            continue;
          if (measurement.depth(v, u) == 0)
            continue;
          std::cerr << mapframe.depth(v, u) * 0.001 << " "
               << measurement.depth(v, u) * 0.001 << std::endl;
        }
      }
    }
  }
  std::cout << std::endl;

  return counts.sum();
}

} // namespace clams
