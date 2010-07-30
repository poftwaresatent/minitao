include_directories (..)

add_library (minitao_tests util.cpp model_library.cpp sai_brep.cpp sai_brep_parser.cpp)
target_link_libraries (minitao_tests minitao tixml261 ${MAYBE_GCOV})

if (HAVE_GTEST)

  add_executable (testTAO testTAO.cpp)
  target_link_libraries (testTAO minitao gtest ${MAYBE_GCOV})
  
  add_executable (testMiniTAO testMiniTAO.cpp)
  target_link_libraries (testMiniTAO minitao_tests gtest ${MAYBE_GCOV})

else (HAVE_GTEST)
  message ("WARNING gtest not found, cannot build minitao/tests!")
endif (HAVE_GTEST)
