cmake_minimum_required (VERSION 2.6)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
endif (COMMAND CMAKE_POLICY)

##################################################
# project setup

project (MiniTAO)
include_directories (tao)
include (CheckIncludeFileCXX)

##################################################
# turn on coverage, if desired
#
# to switch it on, run cmake with -DCOVERAGE=True

if (COVERAGE)
  add_definitions (-fprofile-arcs)
  add_definitions (-ftest-coverage)
  set (MAYBE_GCOV gcov)
endif (COVERAGE)

##################################################
# find Eigen matrix and linear algebra library
#
# NOTE: the eigen3/ prefixing is a quick hack due to CMake
# quirks. We should actually explicit check for the
# eigen3/Eigen/Core header, and only then tweak the include
# path... but CMake does not seem to tell us where exactly it found
# a header, so we'd have to jump through some hoops via find_path()
# or so.
if (NOT EIGEN3_DIR)
  set (EIGEN3_DIR $ENV{EIGEN3_DIR})
endif (NOT EIGEN3_DIR)
if (NOT EIGEN3_DIR)
  # If we are building within busylizzy...
  set (EIGEN3_DIR ${EIGEN_ROOT_DIR})
endif (NOT EIGEN3_DIR)
if (EIGEN3_DIR)
	message ("using EIGEN3_DIR ${EIGEN3_DIR}")
  list (APPEND CMAKE_REQUIRED_INCLUDES
    ${EIGEN3_DIR}/include ${EIGEN3_DIR}
    ${EIGEN3_DIR}/include/eigen3 ${EIGEN3_DIR}/eigen3)
  include_directories (
    ${EIGEN3_DIR}/include ${EIGEN3_DIR}
    ${EIGEN3_DIR}/include/eigen3 ${EIGEN3_DIR}/eigen3)
else (EIGEN3_DIR)
  # Add standard install location under Debian (and others) and Macports.
  # See note on quick hack above.
  list (APPEND CMAKE_REQUIRED_INCLUDES /usr/include/eigen3 /opt/local/include/eigen3)
  include_directories (/usr/include/eigen3 /opt/local/include/eigen3)
endif (EIGEN3_DIR)
### Would like to check these but CMake seems broken yet again.
#check_include_file_cxx (Eigen/Core HAVE_EIGEN3)
#if (${HAVE_EIGEN3})
#  message ("found eigen3")
#else (${HAVE_EIGEN3})
#  message (FATAL_ERROR "eigen3 not found. Please install it e.g. using `sudo apt-get install libeigen3-dev' or from http://eigen.tuxfamily.org/ and tell me where to find it by passing the -DEIGEN3_DIR:path=/path/to/eigen3")
#endif (${HAVE_EIGEN3})

##################################################
# the minitao library

add_library (
  minitao SHARED
  Model.cpp
  State.cpp
  tao_dump.cpp
  tao_util.cpp
  vector_util.cpp
  strutil.cpp
  tao/tao/dynamics/taoCNode.cpp
  tao/tao/dynamics/taoABJoint.cpp
  tao/tao/dynamics/taoNode.cpp
  tao/tao/dynamics/taoABNode.cpp
  tao/tao/dynamics/taoWorld.cpp
  tao/tao/dynamics/taoJoint.cpp
  tao/tao/dynamics/taoABDynamics.cpp
  tao/tao/dynamics/taoGroup.cpp
  tao/tao/dynamics/taoDynamics.cpp
  tao/tao/matrix/TaoDeMatrix6.cpp
  tao/tao/matrix/TaoDeVector6.cpp
  tao/tao/matrix/TaoDeQuaternionf.cpp
  tao/tao/matrix/TaoDeMatrix3f.cpp
  tao/tao/matrix/TaoDeTransform.cpp
  tao/tao/utility/TaoDeMassProp.cpp
  tao/tao/utility/TaoDeLogger.cpp)
target_link_libraries (minitao ${MAYBE_GCOV})

##################################################
# installation targets

install (DIRECTORY tao/tao/
  DESTINATION include/tao
  FILES_MATCHING PATTERN "*.h")

install (DIRECTORY tao/tao/
  DESTINATION include/tao
  FILES_MATCHING PATTERN "*.inl")

install (TARGETS minitao DESTINATION lib)

subdirs (tests)
