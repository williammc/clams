// Copyright 2014 The OSlam Authors. All rights reserved.
#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#ifdef _MSC_VER
#include <boost/iostreams/filter/bzip2.hpp>
#else
#include <boost/iostreams/filter/zlib.hpp>
#endif

#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/deque.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/queue.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/unique_ptr.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/version.hpp>

#include <Eigen/Core>

#define DECLARE_STREAMABLE_OBJECT_SERIALIZATION(StreamableObject)              \
template <class Archive>                                                       \
void serialize(Archive& ar, StreamableObject& p, const unsigned int file_version);\

namespace boost {
namespace serialization {
// Eigen matrix serialization ==================================================
template <class Archive, typename _Scalar, int _Rows, int _Cols, int Options_,
          int mr_, int mc_>
inline void serialize(
    Archive& ar, Eigen::Matrix<_Scalar, _Rows, _Cols, Options_, mr_, mc_>& m,
    const unsigned int file_version) {
  int rows = m.rows();
  int cols = m.cols();
  ar& rows;
  ar& cols;

  if (Archive::is_loading::value) m.resize(rows, cols);

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      ar& m(i, j);
    }
  }
}

// Eigen transform serialization ===============================================
template <class Archive, typename Scalar, int DIM, int MODE, int Options>
inline void serialize(
    Archive& ar, Eigen::Transform<Scalar, DIM, MODE, Options>& tr,
    const unsigned int file_version) {
  auto m = tr.matrix();
  ar& m;
  if (Archive::is_loading::value) {
    tr = Eigen::Transform<Scalar, DIM, MODE, Options>(m);
  }
}

}  // namespace serialization
}  // namespace boost

// a work around for template code
#define BOOST_CLASS_VERSION_TEMPLATE1(T, N)                               \
namespace boost {                                                         \
namespace serialization {                                                 \
 template <typename t1>                                                   \
  struct version<T<t1> > {                                                \
    typedef mpl::int_<N> type;                                            \
    typedef mpl::integral_c_tag tag;                                      \
    BOOST_STATIC_CONSTANT(int, value = version::type::value);             \
    BOOST_MPL_ASSERT(                                                     \
        (boost::mpl::less<boost::mpl::int_<N>, boost::mpl::int_<256> >)); \
    /*                                                                    \
    BOOST_MPL_ASSERT((                                                    \
        mpl::equal_to<                                                    \
            :implementation_level<T >,                                    \
            mpl::int_<object_class_info>                                  \
        >::value                                                          \
    ));                                                                   \
    */                                                                    \
  };                                                                      \
 }                                                                        \
}

// a work around for template code
#define BOOST_CLASS_VERSION_TEMPLATE2(T, N)                               \
namespace boost {                                                         \
namespace serialization {                                                 \
template <typename t1, typename t2>                                       \
  struct version<T<t1, t2> > {                                            \
    typedef mpl::int_<N> type;                                            \
    typedef mpl::integral_c_tag tag;                                      \
    BOOST_STATIC_CONSTANT(int, value = version::type::value);             \
    BOOST_MPL_ASSERT(                                                     \
        (boost::mpl::less<boost::mpl::int_<N>, boost::mpl::int_<256> >)); \
    /*                                                                    \
    BOOST_MPL_ASSERT((                                                    \
        mpl::equal_to<                                                    \
            :implementation_level<T >,                                    \
            mpl::int_<object_class_info>                                  \
        >::value                                                          \
    ));                                                                   \
    */                                                                    \
  };                                                                      \
}                                                                         \
}

