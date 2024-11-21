set(RC_LIB_HEADERS
    data.h
    calculations.h
)

set(RC_LIB_SOURCES
    calculations.cpp
)

add_library(radar_control_lib STATIC ${RC_LIB_HEADERS} ${RC_LIB_SOURCES})

target_include_directories(radar_control_lib PRIVATE ${CMAKE_SOURCE_DIR})
