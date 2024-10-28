set(VIS_LIB_HEADERS
    visualizer.h
)

set(VIS_LIB_SOURCES
    visualizer.cpp
)

add_library(visualizer_lib STATIC ${VIS_LIB_HEADERS} ${VIS_LIB_SOURCES})

target_include_directories(visualizer_lib PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(visualizer_lib PRIVATE radar_control_lib util_lib)
