cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
project (DroneInspection)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

# Should be enabled later on
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")

add_subdirectory(path_planning)
add_subdirectory(flight_control)
