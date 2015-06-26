// Copyright 2014 The OSlam Authors. All rights reserved.
#include "clams/serialization/common.h"

#include <Eigen/Core>

#include "clams/serialization/opencv.h"

#define DECLARE_STREAMABLE_OBJECT_SERIALIZATION_IMPL(StreamableObject)         \
template <class Archive>                                                       \
void serialize(Archive& ar, StreamableObject& p,                               \
              const unsigned int file_version) {                               \
  std::stringstream ss;                                                        \
  ss << p;                                                                     \
  std::string s = ss.str();                                                    \
  ar& make_nvp("streamable_object", s);                                        \
  ss.str("");                                                                  \
  if (Archive::is_loading::value) {                                            \
    ss << s;                                                                   \
    ss >> p;                                                                   \
  }                                                                            \
}

namespace boost {
namespace serialization {

}  // namespace serialization
}  // namespace boost
