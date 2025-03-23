set(RC_HEADERS
    calculations.h
    data.h
    radar_controller.h
)

set(RC_SOURCES
    calculations.cpp
    radar_controller.cpp
)

add_library(radar_control STATIC ${RC_HEADERS} ${RC_SOURCES})

target_include_directories(radar_control PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(radar_control PRIVATE util_lib)
