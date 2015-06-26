#include "clams/trajectory_visualizer.h"
#include <thread>
#include "clams/common/clams_defer.h"
#include "clams/common/timer.h"

namespace clams {

TrajectoryVisualizer::TrajectoryVisualizer(StreamSequenceBase::ConstPtr sseq,
                                           Trajectory traj, Cloud::Ptr map,
                                           std::string title)
    :

      dddm_(NULL),
      sseq_(sseq), traj_(traj), map_(map), quitting_(false),
      needs_update_(true), frame_idx_(0), show_frame_(false),
      use_distortion_model_(false), color_frame_(false), title_(title) {
  vis_.registerKeyboardCallback(&TrajectoryVisualizer::keyboardCallback, *this);
  vis_.registerPointPickingCallback(&TrajectoryVisualizer::pointPickingCallback,
                                    *this);
  vis_.setBackgroundColor(1, 1, 1);
  if (title != "")
    vis_.addText(title_, 10, 10, 16, 0, 0, 0, "title");

  Cloud::Ptr pcd(new Cloud);
  Point tmp(0, 0, 0);
  tmp.r = 0;
  tmp.g = 0;
  tmp.b = 0;
  pcd->push_back(tmp); // PCLVis / VTK can spit out horrible garbage if you give
                       // it an empty pcd.
  vis_.addPointCloud(pcd, "map");
  vis_.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "map");
}

void TrajectoryVisualizer::run() {
  // -- Run the main visualization loop.
  incrementFrameIdx(1);
  while (true) {
    {
      lockRead();
      if (quitting_)
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    lockWrite();
    if (needs_update_) {
      Cloud::Ptr pcd(new Cloud);
      *pcd = *map_;

      // -- Add the raw sensor data from the current frame.
      if (show_frame_) {
        if (traj_.exists(frame_idx_)) {
          Frame pose_frame;
          sseq_->readFrame(frame_idx_, &pose_frame);
          if (dddm_ && use_distortion_model_) {
            ScopedTimer st("Undistorting");
            dddm_->undistort(pose_frame.depth);
            st.StopAndPrint();
          }
          Cloud pose_pcd;
          sseq_->GetFrameProjector().frameToCloud(pose_frame, &pose_pcd);
          if (!color_frame_) {
            for (size_t i = 0; i < pose_pcd.size(); ++i) {
              pose_pcd[i].r = 255;
              pose_pcd[i].g = 0;
              pose_pcd[i].b = 0;
            }
          }

          Eigen::Affine3f transform = traj_.get(frame_idx_).cast<float>();
          pcl::transformPointCloud(pose_pcd, pose_pcd, transform);
          *pcd += pose_pcd;
        }
      }

      vis_.removeShape("title");
      vis_.addText(title_, 10, 10, 16, 0, 0, 0, "title");

      vis_.updatePointCloud(pcd, "map");
      needs_update_ = false;
    }

    vis_.removeCoordinateSystem(frame_idx_);
    vis_.addCoordinateSystem(0.3, traj_.get(frame_idx_).cast<float>(), frame_idx_);

    unlockWrite();
    vis_.spinOnce(3);
  }

  // This PCL call doesn't appear to do anything.
  // The result is that the visualizer persists when it shouldn't.
  // This is a known bug. See
  // https://github.com/PointCloudLibrary/pcl/pull/85
  vis_.close();
}

void TrajectoryVisualizer::pointPickingCallback(
    const pcl::visualization::PointPickingEvent &event, void *cookie) {
  scopeLockRead;

  if (event.getPointIndex() == -1)
    return;
  if (!traj_.exists(frame_idx_))
    return;

  Point pt;
  event.getPoint(pt.x, pt.y, pt.z);
  cout << "Selected point: " << pt.x << ", " << pt.y << ", " << pt.z << endl;
  vis_.removeAllShapes();
  vis_.addText(title_, 10, 10, 16, 0, 0, 0, "title");

  Point origin;
  Eigen::Affine3f transform = traj_.get(frame_idx_).cast<float>();
  origin.getVector3fMap() = transform.translation();
  vis_.addArrow<Point, Point>(origin, pt, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, "line");
}

void TrajectoryVisualizer::keyboardCallback(
    const pcl::visualization::KeyboardEvent &event, void *cookie) {
  if (event.keyDown()) {
    char key = event.getKeyCode();
    if (key == 27) {
      lockWrite();
      quitting_ = true;
      unlockWrite();
    } else if (key == '>')
      incrementFrameIdx(30);
    else if (key == '<')
      incrementFrameIdx(-30);
    else if (key == '.')
      incrementFrameIdx(1);
    else if (key == ',')
      incrementFrameIdx(-1);
    else if (key == 's') {
      scopeLockWrite;
      show_frame_ = !show_frame_;
      needs_update_ = true;
      std::cout << "show_frame_: " << show_frame_ << std::endl;
    } else if (key == 'm') {
      scopeLockWrite;
      use_distortion_model_ = !use_distortion_model_;
      std::cout << "use_distortion_model_: " << use_distortion_model_ << std::endl;
      if (use_distortion_model_ && !dddm_)
        std::cout << "No distortion model provided." << std::endl;
      needs_update_ = true;
    } else if (key == 'c') {
      scopeLockWrite;
      color_frame_ = !color_frame_;
    }
  }
}

void TrajectoryVisualizer::incrementFrameIdx(int num) {
  int idx = (int)frame_idx_;
  int incr = 1;
  if (num < 0)
    incr = -1;

  while (num != 0) {
    // Find the next valid transform.
    while (true) {
      idx += incr;
      if (idx < 0)
        idx = traj_.size() - 1;
      if (idx >= (int)traj_.size())
        idx = 0;

      if (traj_.exists(idx)) {
        break;
      }
    }
    num -= incr;
  }

  scopeLockWrite;
  vis_.removeAllShapes();
  vis_.addText(title_, 10, 10, 16, 0, 0, 0, "title");
  frame_idx_ = (size_t)idx;
  needs_update_ = true;
}

} // namespace clams
