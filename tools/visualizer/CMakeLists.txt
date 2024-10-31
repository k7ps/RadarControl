set(VIS_LIB_HEADERS
    visualizer.h
)

set(VIS_LIB_SOURCES
    visualizer.cpp
)

find_package(raylib REQUIRED PATHS "/home/k1ps/raylib-5.0")

include_directories(/home/k1ps/raylib-cpp-5.0.2/include)

add_library(visualizer_lib STATIC ${VIS_LIB_HEADERS} ${VIS_LIB_SOURCES})
target_include_directories(visualizer_lib PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(visualizer_lib raylib)

set_target_properties(visualizer_lib PROPERTIES LINK_SEARCH_START_STATIC ON)
