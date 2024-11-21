cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)

project(RadarControl)

add_subdirectory(flat)
add_subdirectory(radar_control/bin)
add_subdirectory(radar_control/lib)
add_subdirectory(radar_control/radar_controller)
add_subdirectory(tools/bin)
add_subdirectory(tools/defense)
add_subdirectory(tools/simulator)
add_subdirectory(tools/visualizer)
add_subdirectory(util)
