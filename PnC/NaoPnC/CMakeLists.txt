file(GLOB_RECURSE sources "*.cpp")
file(GLOB_RECURSE headers "*.hpp")

add_library(NaoPnC ${sources})
target_link_libraries(NaoPnC ${DART_LIBRARIES}
                                  myRobotSystem
                                  myUtils
                                  myFilters
                                  myPlanner
                                  myWBC
                                  myTrajectoryManager)