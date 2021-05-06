file(GLOB srcs "*.cpp" "*.hpp")
  
add_executable(run_nao ${srcs})
target_link_libraries(run_nao ${DART_LIBRARIES}
                                myUtils
                                NaoPnC
                                myRobotSystem)

