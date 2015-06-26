set(usg_msg
"  Find OpenNI2\n"
"\n"
"  This sets the following variables:\n"
"  OPENNI2_FOUND            - True if OPENNI2 was found.\n"
"  OPENNI2_INCLUDE_DIRS     - Directories containing the OPENNI2 include files.\n"
"  OPENNI2_LIB_DIR          - Libraries directory.\n"
"  OPENNI2_LIBRARIES        - Libraries needed to use OPENNI2.\n"
"  OPENNI2_BINARY_DIR       - Binary directory containing dll files.\n"
"  OPENNI2_DEFINITIONS      - Compiler flags for OPENNI2.\n"
"  \n"
"  For libusb-1.0, add USB_10_ROOT if not found\n"
)
message(STATUS ${usg_msg})

# check for 32 or 64bit
if(NOT WIN32 AND NOT APPLE)
  EXEC_PROGRAM(uname ARGS -m OUTPUT_VARIABLE CMAKE_CUR_PLATFORM)
  if( CMAKE_CUR_PLATFORM MATCHES "x86_64")
    set( HAVE_64_BIT 1 )
  else()
    set( HAVE_64_BIT 0 )
  endif()
else()
  if(CMAKE_CL_64)
    set( HAVE_64_BIT 1 )
  else()
    set( HAVE_64_BIT 0 )
  endif()
endif()

# Find LibUSB
if(NOT WIN32)
  pkg_check_modules(PC_USB_10 libusb-1.0)
  find_path(USB_10_INCLUDE_DIR libusb-1.0/libusb.h
            HINTS ${PC_USB_10_INCLUDEDIR} ${PC_USB_10_INCLUDE_DIRS} "${USB_10_ROOT}" "$ENV{USB_10_ROOT}"
            PATH_SUFFIXES libusb-1.0)

  find_library(USB_10_LIBRARY
               NAMES usb-1.0 
               HINTS ${PC_USB_10_LIBDIR} ${PC_USB_10_LIBRARY_DIRS} "${USB_10_ROOT}" "$ENV{USB_10_ROOT}"
               PATH_SUFFIXES lib)
               
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(USB_10 DEFAULT_MSG USB_10_LIBRARY USB_10_INCLUDE_DIR)
   
  if(NOT USB_10_FOUND)
    message(STATUS "OpenNI2 disabled because libusb-1.0 not found.")     
    return()
  else()
    include_directories(SYSTEM ${USB_10_INCLUDE_DIR})
  endif()
endif(NOT WIN32)

if(HAVE_64_BIT)
  set(OPENNI2_ROOT_HINTS "C:/Program Files/OpenNI2" )
else()
  set(OPENNI2_ROOT_HINTS "C:/Program Files (x86)/OpenNI2")
endif()

find_path(OPENNI2_ROOT "Include/OpenNI.h" PATHS ${OPENNI2_ROOT_HINTS})

#add a hint so that it can find it without the pkg-config
find_path(OPENNI2_INCLUDE_DIR OpenNI.h
          PATHS "${OPENNI2_ROOT}/Include")

#add a hint so that it can find it without the pkg-config
find_library(OPENNI2_LIBRARY 
             NAMES OpenNI2
             PATHS "${OPENNI2_ROOT}/Lib"
                   "$ENV{OPENNI2_LIB}")
set(OPENNI2_BINARY_DIR ${OPENNI2_ROOT}/Redist)

if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY} ${LIBUSB_1_LIBRARIES})
else()
  set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY})
endif()

if(OPENNI2_INCLUDE_DIR AND OPENNI2_LIBRARIES)
  set(OPENNI2_FOUND TRUE)
else()
  set(OPENNI2_FOUND FALSE)
endif()

if(OPENNI2_FOUND)  
  if(NOT OPENNI2_ROOT)
    get_filename_component(OPENNI2_ROOT "${OPENNI2_INCLUDE_DIR}/.." ABSOLUTE)
  endif()
  set(OPENNI2_LIB_DIR "${OPENNI2_ROOT}/Lib")
  # Add the include directories
  set(OPENNI2_INCLUDE_DIRS ${OPENNI2_INCLUDE_DIR})  
  message(STATUS "OpenNI2 found (include: ${OPENNI2_INCLUDE_DIRS}, lib: ${OPENNI2_LIBRARY})")
endif(OPENNI2_FOUND)

