#include "clams/slam_calibrator.h"
#include "clams/common/timer.h"

namespace clams {

SlamCalibrator::SlamCalibrator(const FrameProjector &proj)
    : proj_(proj), increment_(1) {}

void filterFringe(Frame *frame) {
  DepthMat &depth = frame->depth;

  cv::Mat1b mask(depth.rows, depth.cols); // points to be removed.
  mask = 0;
  uint16_t threshold = 5000; // millimeters
  for (int y = 1; y < depth.rows; ++y) {
    for (int x = 1; x < depth.cols; ++x) {
      float d = depth(y, x);
      float d1 = depth(y-1, x);
      float d2 = depth(y, x-1);
      if (d == 0 ||d1 == 0 || d2 == 0 ||
          std::fabs(d - d1) > threshold ||
          std::fabs(d - d2) > threshold) {
        mask(y, x) = 255;
      }
    }
  }

  // cv::imshow("mask", mask);
  // cv::imshow("depth", frame->depthImage());
  // cv::waitKey();

  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 8);
  // cv::imshow("mask", mask);
  // cv::waitKey();

  for (int y = 1; y < depth.rows; ++y)
    for (int x = 1; x < depth.cols; ++x)
      if (mask(y, x))
        depth(y, x) = 0;

  // cv::imshow("depth", frame->depthImage());
  // cv::waitKey();
}

Cloud::Ptr SlamCalibrator::buildMap(unsigned int traj_idx) {
  assert(traj_idx < trajectories_.size() && sseqs_.size() == trajectories_.size());
  return buildMap(sseqs_[traj_idx], trajectories_[traj_idx],
    MAX_RANGE_MAP);
}

Cloud::Ptr SlamCalibrator::buildMap(StreamSequenceBase::ConstPtr sseq,
                                    const Trajectory &traj, double max_range,
                                    double vgsize) {
//   ROS_DEBUG_STREAM("Building slam calibration map using max range of "
                   // << max_range);

  Cloud::Ptr map(new Cloud);
  int num_used_frames = 0;
  for (size_t i = 0; i < traj.size(); ++i) {
    if (!traj.exists(i))
      continue;

    // if(i % traj.size() / 10 == 0)
    //   cout << "." << flush;
    Frame frame;

    sseq->readFrame(i, &frame);
    filterFringe(&frame);

    Cloud::Ptr tmp(new Cloud);
    sseq->GetFrameProjector().frameToCloud(frame, tmp.get(), max_range);
    Cloud::Ptr nonans(new Cloud);
    nonans->reserve(tmp->size());
    for (size_t j = 0; j < tmp->size(); ++j)
      if (isFinite(tmp->at(j)))
        nonans->push_back(tmp->at(j));

    pcl::transformPointCloud(*nonans, *nonans, traj.get(i).cast<float>());

    *map += *nonans;
    ++num_used_frames;
    // Added intermediate filtering to handle memory overload on huge maps
    if (num_used_frames % 50 == 0) {
      // cout << "Filtering..." << endl;
      clams::ScopedTimer hrt("filtering");
      pcl::VoxelGrid<Point> vg;
      vg.setLeafSize(vgsize, vgsize, vgsize);
      Cloud::Ptr tmp(new Cloud);
      vg.setInputCloud(map);
      vg.filter(*tmp);
      *map = *tmp;
      hrt.StopAndPrint();
    }
  }
  // cout << endl;

  // cout << "Filtering..." << endl;
  clams::ScopedTimer hrt("filtering");
  pcl::VoxelGrid<Point> vg;
  vg.setLeafSize(vgsize, vgsize, vgsize);
  Cloud::Ptr tmp(new Cloud);
  vg.setInputCloud(map);
  vg.filter(*tmp);
  *map = *tmp;
  hrt.StopAndPrint();
  // cout << hrt.reportMilliseconds() << endl;
  std::cout << "Filtered map has " << map->size() << " points." << std::endl;

  return map;
}

size_t SlamCalibrator::size() const {
  // assert(trajectories_.size() == sseqs_.size());
  return trajectories_.size();
}

DiscreteDepthDistortionModel SlamCalibrator::calibrate() const {
  // assert(!sseqs_.empty());
  clams::DiscreteDepthDistortionModel model(sseqs_[0]->GetFrameProjector().width_,
                                     sseqs_[0]->GetFrameProjector().height_);

  size_t total_num_training = 0;
  for (size_t i = 0; i < size(); ++i) {
    std::cout << "Accumulating training data for sequence " << i << std::flush;
    total_num_training +=
        processMap(*sseqs_[i], trajectories_[i], *maps_[i], &model);
  }

  std::cout << "Trained new DiscreteDepthDistortionModel using "
       << total_num_training << " training examples." << std::endl;

  return model;
}

size_t SlamCalibrator::processMap(const StreamSequenceBase &sseq,
                                  const Trajectory &traj, const Cloud &map,
                                  DiscreteDepthDistortionModel *model) const {
  // -- Select which frame indices from the sequence to use.
  //    Consider only those with a pose in the Trajectory,
  //    and apply downsampling based on increment_.
  std::vector<size_t> indices;
  indices.reserve(traj.numValid());
  int num = 0;
  for (size_t i = 0; i < traj.size(); ++i) {
    if (traj.exists(i)) {
      ++num;
      if (num % increment_ == 0)
        indices.push_back(i);
    }
  }

  // -- For all selected frames, accumulate training examples
  //    in the distortion model.
  Eigen::VectorXi counts = Eigen::VectorXi::Zero(indices.size());
#pragma omp parallel for
  for (size_t i = 0; i < indices.size(); ++i) {
    size_t idx = indices[i];
    // assert(traj.exists(idx));
    std::cout << "." << std::flush;

    Frame measurement;
    sseq.readFrame(idx, &measurement);

    Frame mapframe;
    sseq.GetFrameProjector().estimateMapDepth(map, traj.get(idx).inverse().cast<float>(),
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
