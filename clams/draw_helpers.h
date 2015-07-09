#pragma once
#include <array>
#include <fstream>
#include <Eigen/Core>
#include <opencv2/highgui/highgui.hpp>

namespace clams {
// C = [A | B]
inline cv::Mat CombineMatrixes(const cv::Mat &mat1, const cv::Mat &mat2) {

  int cols = mat1.cols + mat2.cols;
  cv::Mat mResult(std::max<int>(mat1.rows, mat2.rows), cols, mat1.type());
  printf("cols:%d, mResult(%d, %d)\n", cols,  mResult.cols, mResult.rows);
  cv::Mat mat1ROI(mResult, cv::Rect(0, 0, mat1.cols, mat1.rows));
  mat1.copyTo(mat1ROI);
  cv::Mat mat2ROI(mResult, cv::Rect(mat1.cols, 0, mat2.cols, mat2.rows));
  mat2.copyTo(mat2ROI);
  
  return mResult;
}

/// draw error heat map
inline void draw_error_map(const cv::Mat1f& errors, cv::Mat3b& out_img,
                           Eigen::Vector4d& error_marks_in,
                           float opacity = 0.3) {
  if (out_img.empty()) {
    out_img.create(errors.rows, errors.cols);
  }

  out_img = 0;

  // worl out min error & max error, and mask
  float min_error = std::numeric_limits<float>::max();
  float max_error = std::numeric_limits<float>::min();
  for (int v = 0; v < errors.rows; ++v) {
    for (int u = 0; u < errors.cols; ++u) {
      const float error = errors(v, u);
      min_error = std::min(min_error, std::fabs(error));
      max_error = std::max(max_error, std::fabs(error));
    }
  }

  // figuring out error anchors
  Eigen::Vector4d& error_marks = error_marks_in;
  if (error_marks[3] == 0.0) {
    float error_gap = max_error - min_error;
    error_marks[0] = min_error;
    error_marks[1] = min_error + error_gap / 3.0;
    error_marks[2] = min_error + 2.0 * error_gap / 3.0;
    error_marks[3] = max_error;
  }

  // draw heatmap
  const float min_color = 70, max_color = 255;
  for (int v = 0; v < errors.rows; ++v) {
    for (int u = 0; u < errors.cols; ++u) {
      const float error = errors(v, u);
      if (error < 0.0f) continue;
      cv::Vec3b c2(0, 0, 0);
      if (error < error_marks[1]) {
        const float c = min_color +
                        max_color * (error - error_marks[0]) /
                            (error_marks[1] - error_marks[0]);
        c2[0] = int(c);
      } else if (error < error_marks[2]) {
        const float c = min_color +
                        max_color * (error - error_marks[1]) /
                            (error_marks[2] - error_marks[1]);
        c2[1] = int(c);
      } else {
        const float c = min_color +
                        max_color * (error - error_marks[2]) /
                            (error_marks[3] - error_marks[2]);
        c2[2] = int(c);
      }

      out_img(v, u) = c2;
    }
  }

  char ca[200];
  sprintf(ca, "BLUE(%.3f, %.3f), GREEN(%.3f, %.3f), RED(%.3f, %.3f) in meter",
          error_marks[0], error_marks[1], error_marks[1], error_marks[2],
          error_marks[2], error_marks[3]);
  cv::putText(out_img, ca, cv::Point(10, 10), cv::FONT_HERSHEY_PLAIN, 1,
              cv::Scalar(255, 255, 255));

  sprintf(ca, "Min error:%.4f, Max error:%.4f in meter", min_error, max_error);
  cv::putText(out_img, ca, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1,
              cv::Scalar(255, 255, 255));
}

} // namespace clams