cmake_minimum_required(VERSION 2.8.9)
project(clams)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_LIST_DIR})

set(CLAMS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DCLAMS_ROOT="${CLAMS_ROOT}")

# Look for our CMake modules paths
set(CMAKER_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/external/cmaker)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake
                              ${CMAKER_ROOT}/cmake
                              ${CMAKER_ROOT}/cmake/modules
                              ${CMAKE_CURRENT_SOURCE_DIR}/cmake
                              ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
                              ${CMAKE_CURRENT_BINARY_DIR})

if(NOT THE_PROJECT_ROOT OR THE_PROJECT_ROOT STREQUAL "")
  set(THE_PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
endif()

include(CMakerCommonUtil)  # include useful macros
include(CMakerCommonSettings)

set(CLAMS_CXX_FLAGS)
add_definitions(-DCLAMS_ROOT="${CLAMS_ROOT}")

if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  set(CLAMS_CXX_FLAGS " -stdlib=libc++ ")  # prevent default libc++ on new mac
endif()

if(NOT WIN32)
  set(CLAMS_CXX_FLAGS " ${CLAMS_CXX_FLAGS} -std=c++11 ") 
endif()
cmaker_print_status("CLAMS_CXX_FLAGS:${CLAMS_CXX_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CLAMS_CXX_FLAGS}")
cmaker_common_build_setting()

option(OSlam_BUILD_SHARED_LIBS "Build Shared Libraries (preferred and required for the g2o plugin system)" OFF)
set(OSlam_LIB_TYPE STATIC)
if(OSlam_BUILD_SHARED_LIBS)
  set(OSlam_LIB_TYPE SHARED)
  add_definitions(-DCLAMS_SHARED_LIBS)
endif()

## Dependencies ================================================================
# Eigen
set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen-3.2.1)
include_directories(${EIGEN_INCLUDE_DIR})

set(PREBUILTS_DIR C:/Users/Thanh/deps_x86_vc12)

## Ceres Solver 
set(Ceres_DIR ${PREBUILTS_DIR}/ceres-solver/CMake)
cmaker_print_status("EIGEN_INCLUDE_DIR:${EIGEN_INCLUDE_DIR}")
find_package(Ceres REQUIRED)
list(APPEND CLAMS_INC_DIRS ${CERES_INCLUDE_DIRS})
list(APPEND CLAMS_EXTERNAL_LIBS debug ceres-debug optimized ceres)
list(APPEND CLAMS_LIB_DIRS ${Ceres_DIR}/../lib)

# Boost 
set(BOOST_ROOT ${PREBUILTS_DIR}/boost)
set(Boost_INCLUDE_DIR ${BOOST_ROOT})
set(BOOST_LIBRARYDIR ${BOOST_ROOT}/stage/lib)

if(WIN32)
  set(Boost_USE_STATIC_LIBS  ON)
endif(WIN32)

find_package(Boost REQUIRED COMPONENTS system filesystem program_options serialization bzip2)
list(APPEND CLAMS_INC_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND CLAMS_EXTERNAL_LIBS ${Boost_LIBRARIES})
list(APPEND CLAMS_LIB_DIRS ${Boost_LIBRARY_DIR})
cmaker_print_status("Boost libs: ${Boost_LIBRARIES}")


# OpenNI2
find_package(OpenNI2)
if (OPENNI2_FOUND)
  list(APPEND CLAMS_INC_DIRS ${OPENNI2_INCLUDE_DIRS})
  list(APPEND CLAMS_EXTERNAL_LIBS ${OPENNI2_LIBRARIES})
  list(APPEND CLAMS_LIB_DIRS ${OPENNI2_LIB_DIR})
  get_filename_component(OpenNI2_BINARY_DIR "${OPENNI2_LIB_DIR}/../Redist" ABSOLUTE)
  list(APPEND THE_DEPEDENCIES_BINARY_PATHS ${OpenNI2_BINARY_DIR})
endif()

set(THE_DEPEDENCIES_BINARY_PATHS ${THE_DEPEDENCIES_BINARY_PATHS} ${EXTERNAL_BINARY_PATHS} PARENT_SCOPE)
cmaker_print_status("THE_DEPEDENCIES_BINARY_PATHS:${THE_DEPEDENCIES_BINARY_PATHS}")

# PCL
set(FLANN_ROOT "${PREBUILTS_DIR}/flann")
set(VTK_DIR "${PREBUILTS_DIR}/vtk-5.10/lib/vtk-5.10")
set(PCL_DIR "${PREBUILTS_DIR}/pcl/cmake")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
cmaker_cull_library_paths(PCL_LIBRARIES)
list(APPEND EXTERNAL_LIBRARIES ${PCL_LIBRARIES})
set(PCL_BINARY_DIR "${PREBUILTS_DIR}/pcl/bin;${PREBUILTS_DIR}/vtk-5.10/bin;${PREBUILTS_DIR}/flann/bin;")
list(APPEND THE_DEPEDENCIES_BINARY_PATHS ${PCL_BINARY_DIR})

add_definitions(${PCL_DEFINITIONS})

list(APPEND CLAMS_INC_DIRS ${PCL_INCLUDE_DIRS}
                           "${PREBUILTS_DIR}/flann/lib"
                           "${PREBUILTS_DIR}/vtk-5.10/include/vtk-5.10")
list(APPEND CLAMS_EXTERNAL_LIBS ${PCL_LIBRARIES})
list(APPEND CLAMS_LIB_DIRS ${PREBUILTS_DIR}/pcl/lib ${FLANN_ROOT}/lib  
                           "${PREBUILTS_DIR}/vtk-5.10/lib/vtk-5.10")
cmaker_print_status("PCL_LIBRARIES:${PCL_LIBRARIES}")

# OpenCV
find_package(OpenCV REQUIRED)
set(OCV_LIBS)
foreach(NAME ${OpenCV_LIBRARIES})
  set(NAME ${NAME}${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH})
  list(APPEND OCV_LIBS debug;${NAME}d;optimized;${NAME})
endforeach()
list(APPEND CLAMS_INC_DIRS ${OpenCV_INCLUDE_DIRS})
list(APPEND CLAMS_LIB_DIRS ${OpenCV_LIB_DIR})
list(APPEND CLAMS_EXTERNAL_LIBS ${OCV_LIBS})
cmaker_print_status("OpenCV include dirs:${OpenCV_INCLUDE_DIRS}")
cmaker_print_status("OpenCV libs:${OpenCV_LIBRARIES}")
get_filename_component(OpenCV_BINARY_DIR "${OpenCV_LIB_DIR}/../bin" ABSOLUTE)
list(APPEND THE_DEPEDENCIES_BINARY_PATHS ${OpenCV_BINARY_DIR})


##==============================================================================
include_directories(${CLAMS_INC_DIRS})
link_directories(${CLAMS_LIB_DIRS})
set(CLAMS_INC_DIRS_PARENT_SCOPE ${CLAMS_INC_DIRS} PARENT_SCOPE)
set(CLAMS_LIB_DIRS_PARENT_SCOPE ${CLAMS_LIB_DIRS} PARENT_SCOPE)

set(sub_dirs clams clams/common clams/serialization)

set(REG_EXT "[^/]*([.]cpp|[.]c|[.]cc|[.]h|[.]hpp)$")
foreach(sub_dir ${sub_dirs})
  file(GLOB clams_${sub_dir}_sources "${CMAKE_CURRENT_SOURCE_DIR}/${sub_dir}/*.cc")
  file(GLOB clams_${sub_dir}_headers "${CMAKE_CURRENT_SOURCE_DIR}/${sub_dir}/*.h")
  list(APPEND clams_sources ${clams_${sub_dir}_sources}
                            ${clams_${sub_dir}_headers})
  source_group("${sub_dir}" REGULAR_EXPRESSION "${CMAKE_CURRENT_SOURCE_DIR}/${sub_dir}/${REG_EXT}")
endforeach()

##==============================================================================
## library specific settings if any

cmaker_print_status("CMAKE_CXX_FLAGS:${CMAKE_CXX_FLAGS}")
cmaker_print_status("CMAKE_CXX_FLAGS_RELEASE:${CMAKE_CXX_FLAGS_RELEASE}")
add_library(clams ${OSlam_LIB_TYPE} # dll export doesn't work on some static methods???
                      ${clams_sources})
target_link_libraries(clams ${CLAMS_EXTERNAL_LIBS})
set_property(TARGET clams PROPERTY FOLDER "external/clams")
if(SM_USE_COTIRE)
  set_target_properties(clams PROPERTIES COTIRE_UNITY_LINK_LIBRARIES_INIT "COPY")
  cotire(clams)
endif()

set(ALL_LIBRARIES clams ${CLAMS_EXTERNAL_LIBS})
set(CLAMS_ALL_LIBRARIES_PARENT_SCOPE ${ALL_LIBRARIES} PARENT_SCOPE)

##==============================================================================
## programs
file(GLOB prog_srcs ${CMAKE_CURRENT_SOURCE_DIR}/prog/*.cc)
foreach(filename ${prog_srcs})
  get_filename_component(prog_name ${filename} NAME_WE)
  add_executable(${prog_name} ${filename})
  target_link_libraries(${prog_name}  ${ALL_LIBRARIES})
  set_property(TARGET ${prog_name} PROPERTY FOLDER "external/clams/progs")

  if(SM_USE_COTIRE)
    set_target_properties(${prog_name} PROPERTIES COTIRE_UNITY_LINK_LIBRARIES_INIT "COPY")
    cotire(${prog_name})
  endif()
endforeach()

##==============================================================================
file(GLOB test_srcs ${CMAKE_CURRENT_SOURCE_DIR}/clams/test/*.cc)
foreach(filename ${test_srcs})
  get_filename_component(test_name ${filename} NAME_WE)
  add_executable(${test_name} ${filename})
  target_link_libraries(${test_name} clams  ${ALL_LIBRARIES})
  set_property(TARGET ${test_name} PROPERTY FOLDER "external/clams/tests")

  if(SM_USE_COTIRE)
    set_target_properties(${test_name} PROPERTIES COTIRE_UNITY_LINK_LIBRARIES_INIT "COPY")
    cotire(${test_name})
  endif()
endforeach()

# Useful Windows paths settings ================================================
# Useful paths configuration for windows 
if(WIN32)
  if(MSVC)
    if(CMAKE_CL_64)
      set(WIN32_ARCH x64)
    else()
      set(WIN32_ARCH x86)
    endif()
    if(MSVC_VERSION EQUAL 1400)
      set(StructuralModeling_RUNTIME vc8)
    elseif(MSVC_VERSION EQUAL 1500)
      set(StructuralModeling_RUNTIME vc9)
    elseif(MSVC_VERSION EQUAL 1600)
      set(StructuralModeling_RUNTIME vc10)
      get_filename_component(VC_IDE_PATH $ENV{VS100COMNTOOLS}/../IDE ABSOLUTE)
    elseif(MSVC_VERSION EQUAL 1700)
      set(StructuralModeling_RUNTIME vc11)
      get_filename_component(VC_IDE_PATH $ENV{VS110COMNTOOLS}/../IDE ABSOLUTE)
    elseif(MSVC_VERSION EQUAL 1800)
      set(StructuralModeling_RUNTIME vc12)
      get_filename_component(VC_IDE_PATH $ENV{VS120COMNTOOLS}/../IDE ABSOLUTE)
    endif()
    get_filename_component(VC_PATH "${VC_IDE_PATH}/../../VC" ABSOLUTE)
  endif()

  cmaker_print_status("THE_DEPEDENCIES_BINARY_PATHS:${THE_DEPEDENCIES_BINARY_PATHS}")
  list(APPEND THE_DEPEDENCIES_BINARY_PATHS "${VC_IDE_PATH}/../../VC"  
                                           "${VC_IDE_PATH}/../../VC/bin")
  list(APPEND THE_DEPEDENCIES_BINARY_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  list(APPEND THE_DEPEDENCIES_BINARY_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/bin)
  
  configure_file("${CMAKER_ROOT}/scripts/set_paths.bat.in" 
                 "${CMAKE_CURRENT_BINARY_DIR}/set_paths.bat")
  configure_file("${CMAKER_ROOT}/scripts/set_paths.bat.in" 
                 "${THE_LIB_RUNTIME_OUTPUT_DIRECTORY}/set_paths.bat")
  configure_file("${CMAKER_ROOT}/scripts/set_paths_and_run_vc.bat.in"
                 "${CMAKE_CURRENT_BINARY_DIR}/set_paths_and_run_vc.bat")
endif()

set(CLAMS_THE_DEPEDENCIES_BINARY_PATHS_PARENT_SCOPE ${THE_DEPEDENCIES_BINARY_PATHS} PARENT_SCOPE)

# useful configuration (e.x.: put scripts in right places) =====================
if(WIN32)
  configure_file("${CMAKER_ROOT}/scripts/run_ninja_build.bat.in" 
                 "${CMAKE_CURRENT_BINARY_DIR}/run_ninja_build.bat")
  if(NOT EXISTS "${THE_LIB_RUNTIME_OUTPUT_DIRECTORY}/run_windbg.bat")
    configure_file("${CMAKER_ROOT}/scripts/run_windbg.bat.in" 
                 "${THE_LIB_RUNTIME_OUTPUT_DIRECTORY}/run_windbg.bat")
  endif()
  set(THE_BUILD_CMD "run_ninja_build.bat")
  set(THE_FILE_REGEX "^(.*)\\\\(([0-9]*)\\\\)(\\\\s:\\\\s).*error(.*)")
else()
  # configure it to a temp location
  configure_file("${CMAKER_ROOT}/scripts/run_make_build.sh.in" 
                 "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/run_make_build.sh")
  # now copy the temporary into the final destination, setting the permissions
  file(COPY "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/run_make_build.sh"
          DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
          FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ
          GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

  set(THE_BUILD_CMD "run_make_build.sh")
  set(THE_FILE_REGEX "^(.*):([0-9]+):([0-9]+).*error(.*)")
endif()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/Clams.sublime-project.in"
               "${CMAKE_CURRENT_BINARY_DIR}/Clams-${CMAKE_BUILD_TYPE}.sublime-project" @ONLY)