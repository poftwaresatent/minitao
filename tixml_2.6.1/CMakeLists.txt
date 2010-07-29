cmake_minimum_required (VERSION 2.4)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
endif (COMMAND CMAKE_POLICY)

project (tixml261)
add_definitions (-DTIXML_USE_STL)

add_library (
  tixml261 SHARED
  tinystr.cpp
  tinyxml.cpp
  tinyxmlerror.cpp
  tinyxmlparser.cpp)
target_link_libraries (tixml261 ${MAYBE_GCOV})
include_directories (${tixml261_SOURCE_DIR})

install (FILES
  tinyxml.h
  tinystr.h
  DESTINATION include/tixml261)

install (TARGETS
  tixml261
  DESTINATION lib)
