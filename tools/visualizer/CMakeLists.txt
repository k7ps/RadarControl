set(VIS_LIB_HEADERS
    visualizer.h
)

set(VIS_LIB_SOURCES
    visualizer.cpp
)

add_library(visualizer_lib STATIC ${VIS_LIB_HEADERS} ${VIS_LIB_SOURCES})
target_sources(visualizer_lib PUBLIC ${VIS_LIB_HEADERS} ${VIS_LIB_SOURCES})
