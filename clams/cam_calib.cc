#include "clams/cam_calib.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "slick/scene/poli_camera.h"
#include "slick/scene/three_point_pose.h"
#include "slick/util/mestimator.h"
#include "slick/util/common.h"
#include "slick/util/wls.h"

#include "clams/pose_optimization.h"

namespace clams {


void CalibPattern::GenWorldpoints() {
  if (type == Type::ASYM_DOT_BOARD) {
    double offset = 0;
    for (size_t y = 0; y < size.height; y++) {
      if (y % 2 == 1)
        offset = cell_unit * 0.5;
      else
        offset = 0;

      for (size_t x = 0; x < size.width; x++) {
        Eigen::Vector3d v3World(x * cell_unit - offset,
                                y * cell_unit, 0.0f);
        worldpoints.push_back(v3World);
      }
    }
  } else {
    for (size_t y = 0; y < size.height; y++) {
      for (size_t x = 0; x < size.width; x++) {
        Eigen::Vector3d center(x * cell_unit, y * cell_unit, 0.0f);
        worldpoints.push_back(center);
      }
    }
  }
  plane << 0, 0, 1, 0; // z=0 plane
}

/// Get reprojection errors (store in a list)
/// @param  ms  IN  list of CenterLocPairVec-s
/// @param  poses IN  camera pose (estimated)
/// @param  cm  IN  camera model
/// @param  v IN/OUT  list of errors
/// @return a pair of sum square error & Max square error
template <class CameraModel>
inline std::pair<double, double> get_reprojection_error(
    const std::vector<CenterLocPairVec> &ms,
    const std::vector<slick::SE3d> &poses, CameraModel &cm,
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &v) {
  double error = 0, maxError = 0;
  for (size_t i = 0; i < ms.size(); ++i) {
    for (size_t j = 0; j < ms[i].size(); ++j) {
      Eigen::Vector3d camFrame = slick::project(poses[i] * ms[i][j].first);
      Eigen::Vector2d im = cm.Project(slick::project(camFrame));
      Eigen::Vector2d err = ms[i][j].second - im;
      v.push_back(std::make_pair(im, err));
      double thisError = err.squaredNorm();
      maxError = std::max(maxError, thisError);
      error += thisError;
    }
  }
  return std::make_pair(error, maxError);
}

/// Get reprojection error
/// @param  measurementss  IN  list of CenterLocPairVec
/// @param  vPoses  IN list of camera poses
/// @param  cm  IN  camera model
template <class CameraModel>
inline double
get_reprojection_error(const std::vector<CenterLocPairVec> &measurementss,
                       const std::vector<slick::SE3d> &poses,
                       CameraModel &cm) {
  double error = 0;
  for (size_t i = 0; i < measurementss.size(); ++i) {
    for (size_t j = 0; j < measurementss[i].size(); ++j) {
      Eigen::Vector3d vcam =
          slick::project(poses[i] * measurementss[i][j].first);
      Eigen::Vector2d loc = cm.Project(slick::project(vcam));
      error += (measurementss[i][j].second - loc).squaredNorm();
    }
  }
  return error;
}

/// Calculate uncertainty of measurements
/// @param  measurements IN list of measurements
/// @param  poses  IN  list of camera pose
/// @param  cm
/// @param  mCov  IN/OUT covariant matrix
template <class CameraModel>
inline void get_uncertainty(
    const std::vector<CenterLocPairVec> &measurements,
    const std::vector<slick::SE3d> &poses, CameraModel &cm,
    Eigen::Matrix<double, CameraModel::param_n_, CameraModel::param_n_> &mCov) {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> JTJ(
      CameraModel::param_n_ + measurements.size() * 6,
      CameraModel::param_n_ + measurements.size() * 6);
  JTJ.setZero();
  for (size_t i = 0; i < measurements.size(); ++i) {
    Eigen::Matrix<double, 6, 6> poseBlock = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, CameraModel::param_n_, CameraModel::param_n_>
        paramBlock = Eigen::Matrix<double, CameraModel::param_n_,
                                   CameraModel::param_n_>::Zero();
    Eigen::Matrix<double, CameraModel::param_n_, 6> offDiag =
        Eigen::Matrix<double, CameraModel::param_n_, 6>::Zero();
    for (size_t j = 0; j < measurements[i].size(); ++j) {
      Eigen::Vector3d camFrame =
          slick::project(poses[i] * measurements[i][j].first);
      Eigen::Matrix<double, 2, 6> J_pose;
      Eigen::Vector2d v = measurements[i][j].second -
                          cm.Project(transform_and_project(
                              poses[i], measurements[i][j].first, J_pose));

      J_pose = cm.GetProjectionDerivatives(slick::project(camFrame)) * J_pose;
      Eigen::Matrix<double, 2, CameraModel::param_n_> J_param =
          cm.GetParameterDerivs(slick::project(camFrame));
      poseBlock += J_pose.transpose() * J_pose;
      paramBlock += J_param.transpose() * J_param;
      offDiag += J_param.transpose() * J_pose;
    }
    JTJ.block(CameraModel::param_n_ + i * 6, CameraModel::param_n_ + i * 6, 6,
              6) = poseBlock;
    JTJ.block(0, 0, CameraModel::param_n_, CameraModel::param_n_) =
        JTJ.block(0, 0, CameraModel::param_n_, CameraModel::param_n_) +
        paramBlock;
    JTJ.block(0, CameraModel::param_n_ + i * 6, CameraModel::param_n_, 6) =
        offDiag;
    JTJ.block(CameraModel::param_n_ + i * 6, 0, 6, CameraModel::param_n_) =
        offDiag.transpose();
  }
  Eigen::LLT<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> chol(JTJ);
  Eigen::Matrix<double, Eigen::Dynamic, 1> v(JTJ.cols());
  v.setZero();
  for (int i = 0; i < CameraModel::param_n_; ++i) {
    v[i] = 1;
    Eigen::Matrix<double, Eigen::Dynamic, 1> Cv = chol.solve(v);
    v[i] = 0;
    mCov.col(i) = Cv.template segment<CameraModel::param_n_>(0);
  }
}


/// Can track checkerboard, symmetric/asymmetric dots pattern
template <typename CameraModel>
bool TrackCalibPattern(const CameraModel &cam, const cv::Mat &gray,
                              cv::Mat &color_img,
                              CenterLocPairVec &out_measurments,
                              slick::SE3d &pose, const CalibPattern& pattern) {
  cv::SimpleBlobDetector::Params params;
  params.thresholdStep = 10;
  params.minThreshold = 50;
  params.maxThreshold = 220;
  params.minRepeatability = 2;
  params.minDistBetweenBlobs = 10; // adjusted; original:10

  params.filterByColor = true;
  params.blobColor = 0;

  params.filterByArea = true;
  params.minArea = 30; // adjusted; orignal: 25
  params.maxArea = 5000;

  params.filterByCircularity = false;
  params.minCircularity = 0.8f;
  params.maxCircularity = std::numeric_limits<float>::max();

  params.filterByInertia = true;
  // minInertiaRatio = 0.6;
  params.minInertiaRatio = 0.1f;
  params.maxInertiaRatio = std::numeric_limits<float>::max();

  params.filterByConvexity = true;
  // minConvexity = 0.8;
  params.minConvexity =
      0.5f; // adjusted to have wider view angle offset; original:0.95
  params.maxConvexity = std::numeric_limits<float>::max();

  cv::Ptr<cv::FeatureDetector> ptr_featuredetector(
      new cv::SimpleBlobDetector(params));

  const cv::Size board_size = pattern.size;
  bool found;
  std::vector<cv::Point2f> corners;
  switch (pattern.type) { // Find feature points on the input format
  case CalibPattern::Type::CHECKER_BOARD:
    // break 80 columns rule to make it readable!
    found = cv::findChessboardCorners(gray, pattern.size, corners,
                                      CV_CALIB_CB_ADAPTIVE_THRESH |
                                          CV_CALIB_CB_FAST_CHECK |
                                          CV_CALIB_CB_NORMALIZE_IMAGE);
    if (found)
      cv::cornerSubPix(
          gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
          cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    break;
  case CalibPattern::Type::DOT_BOARD:
    found =
        cv::findCirclesGrid(gray, pattern.size, corners,
                            cv::CALIB_CB_SYMMETRIC_GRID, ptr_featuredetector);
    break;
  case CalibPattern::Type::ASYM_DOT_BOARD:
    found =
        cv::findCirclesGrid(gray, pattern.size, corners,
                            cv::CALIB_CB_ASYMMETRIC_GRID, ptr_featuredetector);
    break;
  }

  if (found) {
    cv::Mat m_corners(1, corners.size(), CV_32FC2, &corners[0]);

#if 1
    cv::Mat cimg = color_img.clone();
    cv::drawChessboardCorners(cimg, pattern.size, m_corners, found);
    cv::imshow("Tracked pattern", cimg);
    cv::waitKey();
#endif

    CenterLocPairVec observations;
    for (size_t i = 0; i < pattern.worldpoints.size(); i++) {
      const Eigen::Vector2d loc(corners[i].x, corners[i].y);
      // unproject 2D(u,v) image points to camera plane z=1
      Eigen::Vector2d v2_unprojected =
          cam.UnProject(loc);
      observations.push_back(
          std::make_pair(slick::unproject(pattern.worldpoints[i]), v2_unprojected));
      out_measurments.push_back(
        std::make_pair(slick::unproject(pattern.worldpoints[i]), loc));
    }
    auto const pair_re = FindBestPose2Steps(observations);
    pose = pair_re.first;

    if (0) {
      printf("==================================================\n");
      printf("FindBestPose2Steps error:%.10f\n", pair_re.second);
      printf("==================================================\n");
    }
  }
  return found;
}

/// Do Levenberg Marquatt step to 'improve' measurement
/// @param  measurements IN/OUT list of measurment (3D point with 2D projected
/// (u,v))
/// @param  poses IN/OUT list of camera poses (estimated)
/// @param  cm
/// @param  dlamda  IN/OUT lamda in LM algorithm
template <class CameraModel>
inline void
do_step_lm_camera_pose(std::vector<CenterLocPairVec> &measurements,
                       std::vector<slick::SE3d> &poses,
                       CameraModel &cm, double dLambda) {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> JTJ(
      CameraModel::param_n_ + measurements.size() * 6,
      CameraModel::param_n_ + measurements.size() * 6);
  Eigen::Matrix<double, Eigen::Dynamic, 1> JTe(JTJ.rows());

  JTJ.setZero();
  Eigen::Matrix<double, CameraModel::param_n_, 1> JTep =
      Eigen::Matrix<double, CameraModel::param_n_, 1>::Zero();

  for (size_t i = 0; i < measurements.size(); ++i) {
    Eigen::Matrix<double, 6, 6> poseBlock = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, CameraModel::param_n_, CameraModel::param_n_>
        paramBlock = Eigen::Matrix<double, CameraModel::param_n_,
                                   CameraModel::param_n_>::Zero();
    Eigen::Matrix<double, CameraModel::param_n_, 6> offDiag =
        Eigen::Matrix<double, CameraModel::param_n_, 6>::Zero();

    Eigen::Matrix<double, 6, 1> JTei = Eigen::Matrix<double, 6, 1>::Zero();

    for (size_t j = 0; j < measurements[i].size(); j++) {
      Eigen::Vector3d camFrame = slick::project(
          poses[i] * slick::unproject(measurements[i][j].first));
      Eigen::Matrix<double, 2, 6> J_pose;
      Eigen::Vector2d v =
          measurements[i][j].second -
          cm.Project(transform_and_project(
              poses[i], slick::unproject(measurements[i][j].first), J_pose));
      J_pose = cm.GetProjectionDerivatives(slick::project(camFrame)) * J_pose;
      Eigen::Matrix<double, 2, CameraModel::param_n_> J_param =
          cm.GetParameterDerivs(slick::project(camFrame));
      poseBlock += J_pose.transpose() * J_pose;
      paramBlock += J_param.transpose() * J_param;
      offDiag += J_param.transpose() * J_pose;
      JTei += J_pose.transpose() * v;
      JTep += J_param.transpose() * v;
    }
    JTe.template segment<6>(CameraModel::param_n_ + i * 6) = JTei;
    JTJ.block(CameraModel::param_n_ + i * 6, CameraModel::param_n_ + i * 6, 6,
              6) = poseBlock;
    JTJ.block(0, 0, CameraModel::param_n_, CameraModel::param_n_) += paramBlock;
    JTJ.block(0, CameraModel::param_n_ + i * 6, CameraModel::param_n_, 6) =
        offDiag;
    JTJ.block(CameraModel::param_n_ + i * 6, 0, 6, CameraModel::param_n_) =
        offDiag.transpose();
  }
  JTe.template segment<CameraModel::param_n_>(0) = JTep;

  for (int i = 0; i < JTJ.rows(); ++i)
    JTJ(i, i) += dLambda;
  Eigen::LLT<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> chol(JTJ);
  Eigen::Matrix<double, Eigen::Dynamic, 1> delta = chol.solve(JTe);
  Eigen::Matrix<double, CameraModel::param_n_, 1> v6 =
      cm.parameters() + delta.template segment<CameraModel::param_n_>(0);
  cm.SetParameters(v6);
  for (size_t i = 0; i < poses.size(); ++i) {
    poses[i] = slick::SE3d::exp(
                   delta.segment(CameraModel::param_n_ + i * 6, 6)) *
               poses[i];
  }
}

/// Calculate camera parameter based on captured measurements(3D world position
/// & 2D image point)
/// @param  measurementss[in] list of CenterLocPairVec-s
/// @param  vPoses[in]  list of camera poses
/// @param  cameraModel[in]  camera model
template <class CameraModel>
int RunCalibration(std::vector<CenterLocPairVec> &measurements,
                          std::vector<slick::SE3d> poses, CameraModel &cam,
                          double &reprojection_error) {
  size_t nmeas = 0;
  for (size_t i = 0; i < measurements.size(); i++)
    nmeas += measurements[i].size();
  size_t npoints = 0;
  for (size_t i = 0; i < measurements.size(); i++)
    npoints += measurements[i].size();

  std::cerr << measurements.size() << " sets of measurements, " << npoints
            << " total points" << std::endl;

  double min_error = get_reprojection_error(measurements, poses, cam);
  double factor = 1.0 / std::max(640, 480);
  std::cerr << sqrt(min_error / npoints) << " initial reprojection error"
            << std::endl;

  std::cerr << "Optimizing with Levenberg-Marquardt..." << std::endl;
  double lambda = 1;
  size_t i = 0;
  while (lambda < 1e12 && i < 50) {
    Eigen::Matrix<double, CameraModel::param_n_, 1> params = cam.parameters();
    std::vector<slick::SE3d> vOldPoses = poses;
    do_step_lm_camera_pose(measurements, poses, cam, lambda);
    double error = get_reprojection_error(measurements, poses, cam);
    if (fabs(min_error - error) > 1e-19) {
      min_error = error;
      lambda *= 0.5;
      std::cerr << "rms reprojection error = " << sqrt(min_error / npoints)
                << " pixels" << std::endl;
    } else {
      poses = vOldPoses;
      cam.SetParameters(params);
      lambda *= 10;
    }
    i++;
  }

  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> vErrors;
  std::pair<double, double> pError =
      get_reprojection_error(measurements, poses, cam, vErrors);

#ifndef WIN32
  std::cerr << "Estimating uncertainty..." << std::endl;
  Eigen::Matrix<double, CameraModel::param_n_, CameraModel::param_n_> Cov;
  get_uncertainty(measurements, poses, cam, Cov);
  static const int NumRadialParams = CameraModel::param_n_ - 4;
  Cov.block(4, 4, NumRadialParams, NumRadialParams) *= factor * factor;
  Cov.block(0, 4, 4, NumRadialParams) *= factor;
  Cov.block(4, 0, NumRadialParams, 4) *= factor;

  Eigen::Matrix<double, CameraModel::param_n_, 1> uncertainty;
  for (int i = 0; i < CameraModel::param_n_; i++)
    uncertainty[i] = 3 * std::sqrt(Cov(i, i));

  std::cout << "Covariance:" << std::endl;
  std::cout << std::endl << Cov << std::endl << std::endl;
  std::cout << "Three sigma uncertainties: " << std::endl;
  std::cout << uncertainty << std::endl << std::endl;

#endif

  size_t numPoints = 0;
  for (size_t i = 0; i < measurements.size(); i++)
    numPoints += measurements[i].size();

  reprojection_error = sqrt(pError.first / npoints);

  std::cout.precision(14);
  std::cout << sqrt(pError.first / npoints)
            << " pixels average reprojection error" << std::endl;
  std::cout << "Parameters:" << std::endl;
  std::cout << cam.parameters() << std::endl << std::endl;
  return 1;
}

// instantiate =================================================================
template bool TrackCalibPattern(const slick::PoliCamera<double> &cam,
                                const cv::Mat &gray, cv::Mat &color_img,
                                CenterLocPairVec &out_meas, slick::SE3d &pose,
                                const CalibPattern &pattern);
} // namespace clams