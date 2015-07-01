#include "clams/slam_calibrator.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "clams/common/timer.h"

namespace clams {

SlamCalibrator::SlamCalibrator(FrameProjector &proj)
    : proj_(proj), increment_(1) {

  depth_model_ = clams::DiscreteDepthDistortionModel(proj.width(), proj.height());
}

SlamCalibrator::SlamCalibrator(const CalibPattern& pt, FrameProjector &proj)
    : pattern_(pt), proj_(proj), increment_(1) {
  depth_model_ = clams::DiscreteDepthDistortionModel(proj.width(), proj.height());
}

Cloud::Ptr SlamCalibrator::BuildMap(unsigned int idx) {
  assert(idx < slam_maps_.size());
  return slam_maps_[idx].GeneratePointcloud(MAX_RANGE_MAP);
}

size_t SlamCalibrator::size() const {
  return slam_maps_.size();
}

bool SlamCalibrator::Calibrate() {
  CalibrateIntrinsicsNoDepth();
  for (int i = 0; i < 1; ++i) {
    printf("Interation %d ....\n", i);
    printf("Calibrate depth model:\n");
    CalibrateDepthDistortion();
    printf("Calibrate camera model:\n");
    CalibrateIntrinsicsUseDepth();
  }
  return true;
}

bool SlamCalibrator::CalibrateIntrinsicsNoDepth() {
  if (pattern_.empty()) return false;
  printf("Calibrate camera intrinsics with pattern(%s)...\n",
    CalibPattern::to_string(pattern_.type).c_str());
  // collect measurements
  std::vector<CenterLocPairVec> measurements_vec;
  std::vector<slick::SE3d> poses;

  size_t img_count = 0, meas_count = 0;
  for (size_t k = 0; k < size(); ++k) {
    printf("Accumulating training data for sequence %u\n", k);
    auto& slammap = slam_maps_[k];
    bool tracked_calib = slammap.ExistsTrackedCalibPattern();

    for (size_t i = 0; i < slammap.rec_timeframes().size(); ++i) {
      if ( i % increment_ != 0)
        continue;
      std::cout << "." << std::flush;

      Frame measurement;
      slammap.ReadFrameInTrajectory(i, measurement);

      if (!tracked_calib) {
        cv::Mat gray;
        cv::cvtColor(measurement.img, gray, CV_BGR2GRAY);
        bool found = TrackCalibPattern(slammap.frame_projector().poli_cam(), gray, measurement.img,
                          measurement.measurements, measurement.target2cam,
                          pattern_);
        if (!found) continue;
        slammap.WriteFrameInTrajectory(i, measurement);
      }

      if (!measurement.measurements.empty()) {
        measurements_vec.push_back(measurement.measurements);
        poses.push_back(measurement.target2cam);
        img_count += 1;
        meas_count += measurement.measurements.size();
        printf("target2cam pose of frame-%u:\n", i);
        std::cout << measurement.target2cam << std::endl;
      }
    }
  }

  if (measurements_vec.empty()) return false;
  printf("Trained new intrinsics calibration using %d training images with %d "
         "measurements\n",
         img_count, meas_count);

  double reproj_error;
  return RunCalibration(measurements_vec, poses, proj_.poli_cam(), reproj_error, 10);
}

bool SlamCalibrator::CalibrateIntrinsicsUseDepth() {
  return true;
}

bool SlamCalibrator::CalibrateDepthDistortion() {
  size_t total_num_training = 0;
  for (size_t i = 0; i < size(); ++i) {
    std::cout << "Accumulating training data for sequence " << i << std::flush;
    if (pattern_.empty())
    total_num_training +=
        ProcessMap(slam_maps_[i], depth_model_);
    else {
      total_num_training +=
        ProcessMapPlanarTarget(slam_maps_[i], depth_model_);
    }
  }

  printf(
      "Trained new DiscreteDepthDistortionModel using %d training examples.\n",
      total_num_training);
  return true;

}

size_t SlamCalibrator::ProcessMap(SlamMap& slammap,
                                  DiscreteDepthDistortionModel& model) const {
  // -- Select which frame indices from the sequence to use.
  //    Consider only those with a pose in the Trajectory,
  //    and apply downsampling based on increment_.
  const Cloud& pointcloud = *(slammap.pointcloud());

  // -- For all selected frames, accumulate training examples
  //    in the distortion model.
  Eigen::VectorXi counts = Eigen::VectorXi::Zero(slammap.traj_timeposes().size());
#pragma omp parallel for
  for (size_t i = 0; i < slammap.traj_timeposes().size(); ++i) {
    if ( i % increment_ != 0)
      continue;
    std::cout << "." << std::flush;

    Frame measurement;
    slammap.ReadFrameInTrajectory(i, measurement);

    Frame mapframe;
    slammap.frame_projector().EstimateMapDepth(
        pointcloud, slammap.GetPoseInTrajectory(i).inverse().cast<float>(),
        measurement, mapframe.depth);
    counts[i] = model.accumulate(mapframe.depth, measurement.depth);

    
    cv::imshow("measurement", measurement.DepthImage());
    cv::imshow("est depth", mapframe.DepthImage());
    cv::waitKey();

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

size_t SlamCalibrator::ProcessMapPlanarTarget(
  SlamMap &slammap, DiscreteDepthDistortionModel& model) const {
  slammap.frame_projector().poli_cam() = proj_.poli_cam();
  auto& worldpoints = pattern_.worldpoints;
  auto& target_plane = pattern_.plane;

  // -- For all selected frames, accumulate training examples
  //    in the distortion model.
  Eigen::VectorXi counts = Eigen::VectorXi::Zero(slammap.rec_timeframes().size());
  bool tracked_pattern = slammap.ExistsTrackedCalibPattern();
#pragma omp parallel for
  for (size_t i = 0; i < slammap.rec_timeframes().size(); ++i) {
    if ( i % increment_ != 0)
      continue;
    std::cout << "." << std::flush;

    Frame measurement;
    slammap.ReadFrameInTrajectory(i, measurement);

    if (measurement.measurements.empty())
      continue;

    Frame est_frame;
    slammap.frame_projector().EstimateDepthFromPlanarPattern(target_plane,
                        measurement, est_frame.depth);
    counts[i] = model.accumulate(est_frame.depth, measurement.depth);
#if 0
    cv::imshow("measurement", measurement.DepthImage());
    cv::imshow("est depth", est_frame.DepthImage());
    cv::waitKey();
#endif

    // -- Quick and dirty option for data inspection.
    if (getenv("U") && getenv("V")) {
      int u_center = atoi(getenv("U"));
      int v_center = atoi(getenv("V"));
      int radius = 1;
      for (int u = std::max(0, u_center - radius);
           u < std::min(640, u_center + radius + 1); ++u) {
        for (int v = std::max(0, v_center - radius);
             v < std::min(480, v_center + radius + 1); ++v) {
          if (est_frame.depth(v, u) == 0)
            continue;
          if (measurement.depth(v, u) == 0)
            continue;
          std::cerr << est_frame.depth(v, u) * 0.001 << " "
               << measurement.depth(v, u) * 0.001 << std::endl;
        }
      }
    }
  }
  printf("\n");

  return counts.sum();
}

} // namespace clams
