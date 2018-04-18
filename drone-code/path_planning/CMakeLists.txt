# Path planner cmake. Build two targets, one for library and one for
# a test executable that contains a basic test of the interface.

add_library(path_planner path_planner.cpp)
target_include_directories(path_planner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(path_planner_test path_planner_test.cpp)
target_link_libraries(path_planner_test path_planner)

add_executable(path_to_logfile path_to_logfile.cpp)
target_link_libraries(path_to_logfile path_planner)
