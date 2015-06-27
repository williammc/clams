// Copyright 2014 The OSlam Authors. All rights reserved.
#pragma once
#include <boost/filesystem.hpp>

#include "clams/serialization/common.h"
#include "clams/serialization/opencv.h"

#include "clams/common/clams_defer.h"

namespace bfs = boost::filesystem;
// Useful routines for serialization.
namespace clams {

template <typename T, typename Ar>
inline void serialize(T& val, Ar& ar) { val.serialize(ar); }

enum {
  FILE_ARCHIVE_BINARY = 0,
  FILE_ARCHIVE_BINARY_COMPRESSED,
  FILE_ARCHIVE_ASCII,
  FILE_ARCHIVE_XML
};

template <typename T>
inline void SerializeToFile(std::string full_fn, T const& data,
                            int archive_type = FILE_ARCHIVE_BINARY) {
  auto working_path = bfs::path(full_fn).parent_path().string();
  if (!bfs::exists(working_path))
    bfs::create_directory(working_path);

  auto const filename = bfs::path(full_fn).filename().string();

  const bfs::path pre_path = bfs::current_path();
  auto d1 = defer([&pre_path] () {bfs::current_path(pre_path);});

  bfs::current_path(working_path);  // cd to working path

  std::ofstream ofs;
  switch (archive_type) {
    case FILE_ARCHIVE_BINARY: {
      ofs.open(filename, std::ios::binary);
      boost::archive::binary_oarchive ar(ofs);
      ar& data;
      break;
    }
    case FILE_ARCHIVE_BINARY_COMPRESSED: {
      boost::iostreams::filtering_streambuf<boost::iostreams::output> out;

#ifdef _MSC_VER
      out.push(boost::iostreams::bzip2_compressor());
#else
      out.push(boost::iostreams::zlib_compressor(
          boost::iostreams::zlib::best_speed));
#endif
      ofs.open(filename, std::ios::binary);
      out.push(ofs);
      boost::archive::binary_oarchive ar(out);
      ar& data;
      break;
    }
    case FILE_ARCHIVE_ASCII: {
      ofs.open(filename);
      boost::archive::text_oarchive ar(ofs);
      ar& data;
      break;
    }
    default:
      printf("SerializeToFile(): Unknown archive type\n");
      exit(1);
  }
}

template <typename T>
inline void SerializeFromFile(std::string full_fn, T& data,
                              int archive_type = FILE_ARCHIVE_BINARY) {
  if (bfs::is_directory(full_fn) || !bfs::exists(full_fn)) {
    std::cout << "Path is a not path or does not exists! " << full_fn << std::endl;
    printf("Fail to serialize file to %s\n", full_fn.c_str());
    return;
  }
  auto working_path = bfs::path(full_fn).parent_path().string();
  auto const filename = bfs::path(full_fn).filename().string();

  const bfs::path pre_path = bfs::current_path();
  auto d1 = defer([&pre_path] () {bfs::current_path(pre_path);});

  bfs::current_path(working_path);  // cd to working path

  std::ifstream ifs;
  switch (archive_type) {
    case FILE_ARCHIVE_BINARY: {
      ifs.open(filename, std::ios::binary);
      boost::archive::binary_iarchive ar(ifs);
      ar& data;
      break;
    }
    case FILE_ARCHIVE_BINARY_COMPRESSED: {
      boost::iostreams::filtering_streambuf<boost::iostreams::input> in;

#ifdef _MSC_VER
      in.push(boost::iostreams::bzip2_decompressor());
#else
      in.push(boost::iostreams::zlib_decompressor());
#endif
      ifs.open(filename, std::ios::binary);
      in.push(ifs);
      boost::archive::binary_iarchive ar(in);
      ar& data;
      break;
    }
    case FILE_ARCHIVE_ASCII: {
      ifs.open(filename);
      boost::archive::text_iarchive ar(ifs);
      ar& data;
      break;
    }
    default:
      printf("SerializeFromFile(): Unknown archive type\n");
      exit(1);
  }
  ifs.close();
}

template<class ObjectType>
inline bool SerializeToFile(std::string path, std::string filename, ObjectType& obj,
                     int archive_type = FILE_ARCHIVE_BINARY) {
  const bfs::path working_path(path);
  if (!bfs::exists(path))
    bfs::create_directory(path);
  const bfs::path pre_path = bfs::current_path();
  auto d1 = clams::defer([&pre_path] () {bfs::current_path(pre_path);});  // always go back

  // cd to working path
  bfs::current_path(working_path);
  
  try {
    SerializeToFile(filename.c_str(), obj, archive_type);
  } catch (std::exception&) {
    return false;
  }

  return true;
}

template<class ObjectType>
inline bool SerializeFromFile(std::string path, std::string filename, ObjectType& obj,
                       int archive_type = FILE_ARCHIVE_BINARY) {
  if (!bfs::exists(path + "/" + filename)) {
    std::cout << "File does not exists! " << path + "/" + filename << std::endl;
    printf("Fail to deserialize from file\n");
    return false;
  }
  const bfs::path working_path(path);
  const bfs::path pre_path = bfs::current_path();
  auto d1 = defer([&pre_path] () {bfs::current_path(pre_path);});  // always go back
  
  // cd to working path
  bfs::current_path(working_path);

  bfs::path fn(filename);
  if (! (bfs::is_regular_file(fn) && bfs::exists(fn))) {
    std::cout << "Filename is not a file or does not exists! " 
              << filename << std::endl;
    return false;
  }
  
  try {
    SerializeFromFile(filename.c_str(), obj, archive_type);
  } catch (std::exception&) {
    return false;
  }
  return true;
}

}  // namespace clams