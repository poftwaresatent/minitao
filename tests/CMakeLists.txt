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

# We aren't going to do anything unless we have gtest...

if (HAVE_GTEST)
  
  add_definitions (-DTIXML_USE_STL)
  subdirs (tixml261)
  
  include_directories (. ..)
  
  add_library (minitao_tests util.cpp model_library.cpp sai_brep.cpp sai_brep_parser.cpp)
  target_link_libraries (minitao_tests minitao tixml261 ${MAYBE_GCOV})
  
  add_executable (testTAO testTAO.cpp)
  target_link_libraries (testTAO minitao gtest ${MAYBE_GCOV} -lpthread)
  
  add_executable (testMiniTAO testMiniTAO.cpp)
  target_link_libraries (testMiniTAO minitao_tests gtest ${MAYBE_GCOV} -lpthread)

else (HAVE_GTEST)

  message ("WARNING gtest not found, cannot build minitao/tests!")

endif (HAVE_GTEST)
