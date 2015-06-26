// Copyright 2014 The OSlam Authors. All rights reserved.
#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "clams/serialization/serialization.h"

namespace boost {
namespace serialization {
// cv::Point2f serialization ==================================================
template <class Archive>
inline void serialize(
    Archive& ar, cv::Point2f& pt,const unsigned int file_version) {
  ar& pt.x;
  ar& pt.y;
}

// cv::Keypoint serialization =================================================
template <class Archive>
inline void serialize(
  Archive& ar, cv::KeyPoint& kp, const unsigned int file_version) {
  ar& kp.pt;
  ar& kp.size;
  ar& kp.angle;
  ar& kp.response;
  ar& kp.octave;
  ar& kp.class_id;
}

// cv::Mat serialization ======================================================
template <typename Archive>
inline void serialize(Archive& ar, cv::Mat& m, const unsigned int version) {
  int cols, rows;
  size_t elem_size, elem_type;

  if (Archive::is_saving::value) {
    cols = m.cols;
    rows = m.rows;
    elem_size = m.elemSize();
    elem_type = m.type();
  }

  ar& cols;
  ar& rows;
  ar& elem_size;
  ar& elem_type;

  const size_t data_size = cols * rows * elem_size;

  if (Archive::is_loading::value) {
    m.create(rows, cols, elem_type);
  }

  ar& make_array(m.ptr(), data_size);
}

}  // namespace serialization
}  // namespace boost