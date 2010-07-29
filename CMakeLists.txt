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
# find Google Testing Framework

if (NOT GTEST_DIR)
  set (GTEST_DIR $ENV{GTEST_DIR})
endif (NOT GTEST_DIR)
if (GTEST_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES ${GTEST_DIR}/include ${GTEST_DIR})
  include_directories (${GTEST_DIR}/include ${GTEST_DIR})
  link_directories (${GTEST_DIR}/lib ${GTEST_DIR})
endif (GTEST_DIR)
check_include_file_cxx (gtest/gtest.h HAVE_GTEST_HEADER)
if (${HAVE_GTEST_HEADER})
  if (GTEST_DIR)
    find_library (HAVE_GTEST_LIB gtest PATHS ${GTEST_DIR} ${GTEST_DIR}/lib)
  else (GTEST_DIR)
    find_library (HAVE_GTEST_LIB gtest)
  endif (GTEST_DIR)
  if (HAVE_GTEST_LIB MATCHES "NOTFOUND")
    message ("WARNING gtest library not found, although the header gtest/gtest.h was found")
  else (HAVE_GTEST_LIB MATCHES "NOTFOUND")
    message ("found gtest")
    set (HAVE_GTEST TRUE)
    add_definitions (-DHAVE_GTEST)
  endif (HAVE_GTEST_LIB MATCHES "NOTFOUND")
else (${HAVE_GTEST_HEADER})
  message ("WARNING did not find gtest")
endif (${HAVE_GTEST_HEADER})

##################################################
# find Eigen matrix and linear algebra library (version 2.x)
#
# NOTE: the eigen2/ prefixing is a quick hack due to CMake
# quirks. We should actually explicit check for the
# eigen2/Eigen/Core header, and only then tweak the include
# path... but CMake does not seem to tell us where exactly it found
# a header, so we'd have to jump through some hoops via find_path()
# or so.
if (NOT EIGEN2_DIR)
  set (EIGEN2_DIR $ENV{EIGEN2_DIR})
endif (NOT EIGEN2_DIR)
if (EIGEN2_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES
    ${EIGEN2_DIR}/include ${EIGEN2_DIR}
    ${EIGEN2_DIR}/include/eigen2 ${EIGEN2_DIR}/eigen2)
  include_directories (
    ${EIGEN2_DIR}/include ${EIGEN2_DIR}
    ${EIGEN2_DIR}/include/eigen2 ${EIGEN2_DIR}/eigen2)
else (EIGEN2_DIR)
  # This is the standard install location under Debian, maybe others
  # as well. See not on quick hack above.
  list (APPEND CMAKE_REQUIRED_INCLUDES /usr/include/eigen2)
  include_directories (/usr/include/eigen2)
endif (EIGEN2_DIR)
check_include_file_cxx (Eigen/Core HAVE_EIGEN2)
if (${HAVE_EIGEN2})
  message ("found eigen2")
else (${HAVE_EIGEN2})
  message (FATAL_ERROR "eigen2 not found. Please install it e.g. using `sudo apt-get install libeigen2-dev' or from http://eigen.tuxfamily.org/ and tell me where to find it by passing the -DEIGEN2_DIR:path=/path/to/eigen2")
endif (${HAVE_EIGEN2})

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
# tests and utilities

if (HAVE_GTEST)
  add_executable (testTAO tao/tests/testTAO.cpp)
  target_link_libraries (testTAO minitao gtest ${MAYBE_GCOV})
endif (HAVE_GTEST)

##################################################
# installation targets

install (DIRECTORY tao/tao/
  DESTINATION include/tao
  FILES_MATCHING PATTERN "*.h")

install (DIRECTORY tao/tao/
  DESTINATION include/tao
  FILES_MATCHING PATTERN "*.inl")

install (TARGETS minitao DESTINATION lib)
