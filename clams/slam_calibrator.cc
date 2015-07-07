#include "clams/slam_calibrator.h"
#include <numeric>
#include <opencv2/imgproc/imgproc.hpp>
#include "slick/geometry/plane3d.h"
#include "clams/common/timer.h"
#include "clams/draw_helpers.h"
#include "clams/plane_util.h"

namespace clams {

SlamCalibrator::SlamCalibrator(FrameProjector &proj)
    : proj_(proj), increment_(1), depth_model_(proj.width(), proj.height()) {
}

SlamCalibrator::SlamCalibrator(const CalibPattern& pt, FrameProjector &proj)
    : pattern_(pt), proj_(proj), increment_(1), depth_model_(proj.width(), proj.height()) {
}

Cloud::Ptr SlamCalibrator::BuildMap(unsigned int idx) {
  assert(idx < slam_maps_.size());
  return slam_maps_[idx].GeneratePointcloud(MAX_RANGE_MAP);
}

size_t SlamCalibrator::size() const {
  return slam_maps_.size();
}

bool SlamCalibrator::Calibrate() {
  PreparePlanarTargetData();
#if CALIB_CAM_INTRINSICS
  CalibrateIntrinsicsNoDepth();
#endif
  for (int i = 0; i < 1; ++i) {
    printf("Interation %d ....\n", i);
    CalibrateDepthDistortion();
#if CALIB_CAM_INTRINSICS
    CalibrateIntrinsicsUseDepth();
#endif
  }
  printf("FillMissingMultipliers \n");
  depth_model_.FillMissingMultipliers();
  printf("FillMissingDepthOffsets \n");
  depth_model_.FillMissingDepthOffsets();
  return true;
}

void SlamCalibrator::Validate() {
  for (size_t k = 0; k < size(); ++k) {
    printf("Validating training data for sequence %u\n", k);
    auto& slammap = slam_maps_[k];
    if (!pattern_.empty())
      ValidatePlanarTarget(slammap);
  }
}

bool SlamCalibrator::PreparePlanarTargetData() {
  if (pattern_.empty()) return false;
  printf("PreparePlanarTargetData for calib pattern (%s)...\n",
    CalibPattern::to_string(pattern_.type).c_str());
  // collect measurements
  std::vector<CenterLocPairVec> measurements_vec;
  std::vector<slick::SE3d> poses;

  size_t img_count = 0, meas_count = 0;
  for (size_t k = 0; k < size(); ++k) {
    printf("Preparing training data for sequence %u\n", k);
    auto& slammap = slam_maps_[k];
    bool tracked_calib = slammap.ExistsTrackedCalibPattern();
    if (tracked_calib) continue;

    for (size_t i = 0; i < slammap.rec_timeframes().size(); ++i) {
      if ( i % increment_ != 0)
        continue;
      std::cout << "." << std::flush;

      Frame measurement;
      slammap.ReadFrameInTrajectory(i, measurement);

      cv::Mat gray;
      cv::cvtColor(measurement.img, gray, CV_BGR2GRAY);
      bool found = TrackCalibPattern(slammap.frame_projector().poli_cam(), gray, measurement.img,
                        measurement.measurements, measurement.target2cam,
                        pattern_);
      if (!found) continue;

      if (!measurement.measurements.empty()) {
        measurements_vec.push_back(measurement.measurements);
        poses.push_back(measurement.target2cam);
        img_count += 1;
        meas_count += measurement.measurements.size();
        printf("target2cam pose of frame-%u:\n", i);
        std::cout << measurement.target2cam << std::endl;
      }

      slammap.WriteFrameInTrajectory(i, measurement);
    }
  }

  for (size_t k = 0; k < size(); ++k) {
    printf("Preparing training depth offsets for sequence %u\n", k);
    auto& slammap = slam_maps_[k];
    for (size_t i = 0; i < slammap.rec_timeframes().size(); ++i) {
      if ( i % increment_ != 0)
        continue;
      std::cout << "." << std::flush;

      Frame measurement;
      slammap.ReadFrameInTrajectory(i, measurement);

      auto offsets = slammap.frame_projector().EstimateDepthOffsets(measurement);
      if (!offsets.empty()) {
        measurement.depth_offset =
          std::accumulate(offsets.begin(), offsets.end(), 0.0) / offsets.size();
        printf("measurement depth_offset:%f\n", measurement.depth_offset);
      }

      // depth_model_.accumulate(measurement.depth_offset);
    }
  }

  if (measurements_vec.empty()) return false;
  printf("Prepared planar target calibration using %d training images with %d "
         "measurements\n",
         img_count, meas_count);
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

    for (size_t i = 0; i < slammap.rec_timeframes().size(); ++i) {
      if ( i % increment_ != 0)
        continue;
      std::cout << "." << std::flush;

      Frame measurement;
      slammap.ReadFrameInTrajectory(i, measurement);

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

bool SlamCalibrator::CalibrateDepthDistortion() {
  printf("CalibrateDepthDistortion()\n");
  size_t total_num_training = 0;
  for (size_t i = 0; i < size(); ++i) {
    std::cout << "Accumulating training data for sequence " << i << std::flush;
    if (pattern_.empty())
    total_num_training +=
        ProcessMap(slam_maps_[i]);
    else {
      total_num_training +=
        ProcessMapPlanarTarget(slam_maps_[i]);
    }
  }

  depth_model_.CalcMultipliers();
  depth_model_.CalcDepthOffsets();

  printf(
      "Trained new DiscreteDepthDistortionModel using %d training examples.\n",
      total_num_training);
  return true;

}

bool SlamCalibrator::CalibrateIntrinsicsUseDepth() {
  printf("CalibrateIntrinsicsUseDepth()\n");
  return true;
}

size_t SlamCalibrator::ProcessMap(SlamMap& slammap) {
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
    counts[i] = depth_model_.accumulate(mapframe.depth, measurement.depth);

    
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

size_t SlamCalibrator::ProcessMapPlanarTarget(SlamMap &slammap) {
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

    printf("processing frame%f\n", measurement.timestamp);

    if (measurement.measurements.empty())
      continue;

    Frame est_frame;
    slammap.frame_projector().EstimateDepthFromPlanarPattern(target_plane,
                        measurement, est_frame.depth);
    counts[i] = depth_model_.accumulate(est_frame.depth, measurement.depth);
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

bool SlamCalibrator::ValidDepthFromPlanarPattern(
                       cv::Mat3b img, cv::Mat1s undist_depth,
                       const slick::PoliCamera<double>& poli_cam,
                       const CalibPattern& pattern,
                       cv::Mat1f& errors,
                       cv::Mat3b* drawn_pattern) {
  errors.create(undist_depth.rows, undist_depth.cols);
  errors = -1.0f;


  Frame frame;
  frame.img = img;
  frame.depth = undist_depth;

  cv::Mat gray;
  cv::cvtColor(img, gray, CV_BGR2GRAY);
  bool found = TrackCalibPattern(poli_cam, gray, frame.img,
                    frame.measurements, frame.target2cam,
                    pattern);
  if (!found) return false;

  if (drawn_pattern) {
    *drawn_pattern = img.clone();
    std::vector<cv::Point2f> corners;
    for (auto t : frame.measurements)
      corners.push_back(cv::Point2f(t.second[0], t.second[1]));
    cv::Mat m_corners(1, corners.size(), CV_32FC2, &corners[0]);
    cv::drawChessboardCorners(*drawn_pattern, pattern.size, m_corners, true);
  }

  FrameProjector::ReestimatePoseAndPlane(frame, poli_cam, pattern.plane);
  std::vector<cv::Point> target_pts;
  for (auto t : frame.measurements)
    if (frame.depth(t.second[1], t.second[0])) {
      target_pts.push_back(cv::Point(t.second[0], t.second[1]));
    }
  if (target_pts.empty()) {
    printf("No valid depths in the calibration pattern region\n");
    return false;  // cannot find valid depth on the pattern
  }
  
  cv::Mat1b target_mask(frame.depth.rows, frame.depth.cols);
  target_mask = 0;
  cv::convexHull(target_pts, target_pts);
  cv::fillConvexPoly(target_mask, target_pts, cv::Scalar(255));
  
#if 0
  cv::imshow("img", frame.img);
  cv::imshow("target_mask", target_mask);
  cv::waitKey();
#endif

  for (int v = 0; v < target_mask.rows; ++v) {
    for (int u = 0; u < target_mask.cols; ++u) {
      if (frame.depth(v, u) == 0) continue;
      if (target_mask(v, u)) {
        Eigen::Vector3d cam = slick::unproject(poli_cam.UnProject(Eigen::Vector2d(u, v)));
        cam = cam.normalized() * frame.depth(v, u) * 0.001;
        slick::Plane3d pln(frame.target_plane);
        Eigen::Vector3d pt = pln.project(cam);
        errors(v, u) = (pt - cam).norm();
      }
    }
  }
  return true;
}

void SlamCalibrator::ValidatePlanarTarget(SlamMap &slammap) const {

  // -- For all selected frames, accumulate training examples
  //    in the distortion model.
#pragma omp parallel for
  for (size_t i = 0; i < slammap.rec_timeframes().size(); ++i) {
    if ( i % increment_ != 0)
      continue;
    std::cout << "." << std::flush;

    Frame meas;
    slammap.ReadFrameInTrajectory(i, meas);

    printf("validating frame%f\n", meas.timestamp);

    if (meas.measurements.empty())
      continue;

    cv::Mat1f errors, undist_errors;
    cv::Mat3b out_error, undist_out_error;
    cv::Mat3b drawn_pattern;
    auto depth = meas.depth.clone();
    bool ok = ValidDepthFromPlanarPattern(meas.img, depth, proj_.poli_cam(), pattern_,
                                    errors, &drawn_pattern);

    depth_model_.undistort(depth);
    ok = ok && ValidDepthFromPlanarPattern(meas.img, depth, proj_.poli_cam(), pattern_,
                                    undist_errors);
    if (!ok) continue;
    Eigen::Vector4d errors_mask(0, 0, 0, 0);
    draw_error_map(undist_errors, undist_out_error, errors_mask);
    draw_error_map(errors, out_error, errors_mask);
    cv::Mat big = CombineMatrixes(drawn_pattern, out_error);
    std::string fn = slammap.working_path() + "/" + meas.FilenamePrefix() +
                       "-errors.png";
    printf("write error heat map to (%s)\n", fn.c_str());
    cv::imwrite(fn, CombineMatrixes(big, undist_out_error));
  }
  printf("\n");
}
} // namespace clams
