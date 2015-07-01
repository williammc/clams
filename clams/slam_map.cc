#include "clams/slam_map.h"
#include <opencv2/highgui/highgui.hpp>
#include <pcl/filters/voxel_grid.h>
#include "clams/common/timer.h"
#include "clams/common/clams_macros.h"
#include "clams/serialization/serialization.h"

namespace clams {

SlamMap::SlamMap()
    : working_path_("."), undistorted_(false),
      max_depth_(std::numeric_limits<double>::max()) {}

bool SlamMap::Load(const std::string &slammap_file) {
  bool ok = SerializeFromFile(slammap_file, *this);
  working_path_ = bfs::path(slammap_file).parent_path().string();
  return ok;
}

bool SlamMap::Save(const std::string &slammap_file) const {
  return SerializeToFile(slammap_file, *this);
}

inline void red_recording_line(std::istream &is, double &timestamp,
                               std::string &color_fn, std::string &depth_fn) {
  std::string tag;
  bool aligned;
  is >> tag >> timestamp >> color_fn >> depth_fn >> aligned;
}

inline std::string get_frame_name(const Frame& fr) {
  return "clams-frame-" + std::to_string(fr.timestamp) + ".bin";
}

bool SlamMap::LoadTrajectoryAndRecording(std::string traj_file,
                                         std::string rec_file,
                                         std::array<float, 9> cam_params,
                                         unsigned skip_poses) {
  
  traj_file_ = traj_file;
  rec_file_ = rec_file;

  if (!traj_file_.empty()) {
    printf("SlamMap::LoadExternalRecording>> load trajectory file:%s\n",
      traj_file.c_str());
    if (bfs::is_directory(traj_file) || !bfs::exists(traj_file)) {
      std::cout << "Filename is a path or does not exists! " << traj_file
                << std::endl;
      return false;
    }
    std::ifstream frei(traj_file);
    int line_count = 0;
    while (true) {
      double timestamp, tx, ty, tz, qx, qy, qz, qw;
      frei >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
      line_count += 1;
      if (traj_timeposes_.size() % 30 == 0)
        printf("Read line %f, %f, %f, %f, %f, %f, %f, %f\n", timestamp, tx, ty, tz,
               qx, qy, qz, qw);
      if (frei.eof())
        break;

      if (line_count % skip_poses != 0) 
        continue;

      Eigen::Quaternion<double> rotation(qw, qx, qy, qz);
      Eigen::Translation<double, 3> translation(tx, ty, tz);
      Eigen::Affine3d transform = translation * rotation;

      traj_timeposes_.push_back(std::make_pair(timestamp, transform));

    }
  }

  printf("SlamMap::LoadExternalRecording>> load recording file:%s\n",
    rec_file.c_str());
  if (bfs::is_directory(rec_file) || !bfs::exists(rec_file)) {
    std::cout << "Filename is a path or does not exists! " << rec_file
              << std::endl;
    return false;
  }
  auto rec_dir = bfs::path(rec_file).parent_path().string();

  std::ifstream ifs(rec_file);
  int count = 0;
  while (!ifs.eof()) {
    count += 1;
    std::string cfn, dfn;
    Frame fr;
    red_recording_line(ifs, fr.timestamp, cfn, dfn);

    if (count % skip_poses != 1)
      continue;

    if (rec_timeframes_.size() % 30 == 0)
      printf("read recording line timestamp:%f cfn:%s dfn:%s\n",
        fr.timestamp, cfn.c_str(), dfn.c_str());
    fr.img = cv::imread(rec_dir + "/" + cfn, -1);
    fr.depth = cv::imread(rec_dir + "/" + dfn, -1);

    if (fr.img.empty() || fr.depth.empty())
      continue;

    // cv::imshow("depth", fr.DepthImage());
    // cv::waitKey();

    std::string base_fn = get_frame_name(fr);
    std::string full_fn = working_path_ + "/" + base_fn;
    if (rec_timeframes_.size() % 30 == 0)
      printf("Serialize Frame to %s\n", full_fn.c_str());
    if (!bfs::exists(full_fn))
      SerializeToFile(full_fn, fr);
    rec_timeframes_.push_back(std::make_pair(fr.timestamp, base_fn));

    if (traj_file_.empty()) {
      traj_timeposes_.push_back(std::make_pair(fr.timestamp, Eigen::Affine3d()));
    }
  }

  Eigen::VectorXf x(9);
  for (int i = 0; i < 9; ++i)
    x[i] = cam_params[i];

  proj_.poli_cam() = slick::PoliCamera<double>(x);

  printf("SlamMap::LoadExternalRecording>> find associate frames for trajectory\n");
  traj_associate_frames_.resize(traj_timeposes_.size(), -1);
  for (size_t i = 0; i < traj_timeposes_.size(); ++i) {
    double timestamp = traj_timeposes_[i].first;
    double dt;
    size_t idx = SeekInFrames(timestamp, &dt);
    // assert(dt < 1e-6);
    printf("associate idx:%u timestamp:%f dt:%f\n", idx, timestamp, dt);
    // if (dt < 1.e-6)
      traj_associate_frames_[i] = idx;
  }
}

Cloud::Ptr SlamMap::GeneratePointcloud(
  float max_range, float vgsize) const {

  Cloud::Ptr map(new Cloud);
  int num_used_frames = 0;
  for (size_t i = 0; i < traj_timeposes_.size(); ++i) {
    if(i % traj_timeposes_.size() / 10 == 0)
      std::cout << "." << std::flush;
    Frame frame;
    ReadFrameInTrajectory(i, frame);

    frame.FilterFringe();

    Cloud::Ptr tmp(new Cloud);
    frame_projector().FrameToCloud(frame, tmp.get(), max_range);
    Cloud::Ptr nonans(new Cloud);
    nonans->reserve(tmp->size());
    for (size_t j = 0; j < tmp->size(); ++j)
      if (isFinite(tmp->at(j)))
        nonans->push_back(tmp->at(j));

    pcl::transformPointCloud(*nonans, *nonans, traj_timeposes_[i].second.cast<float>());

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

size_t SlamMap::ReadFrameInTrajectory(double timestamp, double *dt,
                                     Frame& frame) const {
  size_t idx = SeekInTrajectory(timestamp, dt);
  ReadFrameInTrajectory(idx, frame);
  return idx;
}

void SlamMap::ReadFrameInTrajectory(size_t idx, Frame& frame) const {
  assert(idx < traj_associate_frames_.size());
  idx = traj_associate_frames_[idx];
  std::string fn = working_path_ + "/" + rec_timeframes_[idx].second;
  SerializeFromFile(fn, frame);
  if (max_depth_ != std::numeric_limits<double>::max())
    for (int y = 0; y < frame.depth.rows; ++y)
      for (int x = 0; x < frame.depth.cols; ++x)
        if (frame.depth(y, x) > max_depth_ * 1000)
          frame.depth(y, x) = 0;
}

void SlamMap::WriteFrameInTrajectory(size_t idx, const Frame& frame) const {
  assert(idx < traj_associate_frames_.size());
  idx = traj_associate_frames_[idx];
  std::string fn = working_path_ + "/" + rec_timeframes_[idx].second;
  SerializeToFile(fn, frame);
}

size_t SlamMap::SeekInFrames(double timestamp, double *dt) const {
  assert(!rec_timeframes_.empty());

  // TODO: This could be much faster than linear search.
  size_t nearest = 0;
  *dt = std::numeric_limits<double>::max();
  for (size_t i = 0; i < rec_timeframes_.size(); ++i) {
    double d = timestamp - rec_timeframes_[i].first;
    if (fabs(d) < *dt) {
      *dt = d;
      nearest = i;
    }
  }

  return nearest;
}

size_t SlamMap::SeekInTrajectory(double timestamp, double *dt) const {
  assert(!traj_timeposes_.empty());

  // TODO: This could be much faster than linear search.
  size_t nearest = 0;
  *dt = std::numeric_limits<double>::max();
  for (size_t i = 0; i < traj_timeposes_.size(); ++i) {
    double d = timestamp - traj_timeposes_[i].first;
    if (fabs(d) < *dt) {
      *dt = d;
      nearest = i;
    }
  }

  return nearest;
}

bool SlamMap::ExistsTrackedCalibPattern() const {
  for (int i = 0; i < traj_timeposes_.size(); ++i) {
    Frame fr;
    ReadFrameInTrajectory(0, fr);
    if (!fr.measurements.empty())
      return true;
  }
  return false;
}

template <class Archive>
void SlamMap::serialize(Archive &ar, const unsigned int version) {
  ar& rec_file_;
  ar& traj_file_;
  ar& max_depth_;
  ar& undistorted_;
  ar& proj_;
  ar& rec_timeframes_;
#if 0
  ar& traj_timeposes_;
#else
  int len = traj_timeposes_.size();
  ar& len;
  if (Archive::is_loading::value) {
    traj_timeposes_.resize(len);
  }
  for (int i = 0; i < traj_timeposes_.size(); ++i) {
    ar& traj_timeposes_[i].first;
    ar& traj_timeposes_[i].second;
  }
#endif
  ar& traj_associate_frames_;
}

CLAMS_INSTANTIATE_SERIALIZATION_T(SlamMap);
} // namespace clams