cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -g")

project(RadarControl)

add_subdirectory(proto)
add_subdirectory(radar_control/bin)
add_subdirectory(radar_control/lib)
add_subdirectory(radar_control/radar_controller)
add_subdirectory(simulator)
add_subdirectory(ut)
add_subdirectory(util)
